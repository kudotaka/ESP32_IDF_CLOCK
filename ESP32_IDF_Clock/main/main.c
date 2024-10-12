#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

#if (  CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT \
    || CONFIG_SOFTWARE_INTERNAL_BUTTON_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_SK6812_SUPPORT \
    || CONFIG_SOFTWARE_SENSOR_SHT3X \
    || CONFIG_SOFTWARE_SENSOR_SHT4X \
    || CONFIG_SOFTWARE_SENSOR_BMP280 \
    || CONFIG_SOFTWARE_SENSOR_QMP6988 \
    || CONFIG_SOFTWARE_SENSOR_BME680 \
    || CONFIG_SOFTWARE_SENSOR_ADT7410 \
    || CONFIG_SOFTWARE_SENSOR_SCD30 \
    || CONFIG_SOFTWARE_SENSOR_SCD40 \
    || CONFIG_SOFTWARE_SENSOR_MHZ19C \
    || CONFIG_SOFTWARE_SENSOR_BH1750 \
    || CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_SK6812_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_BUTTON_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_LED_SUPPORT )
#include "devunit.h"
#endif

static const char *TAG = "MY-MAIN";

#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
void RtcInterruptInit(void);
void RtcStartClockInit(void);
#if CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT
static bool g_clockout_status = false;
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT

static void obtain_time(void);
static void clock_main(void);

i2c_master_bus_handle_t i2c0_master_bus_handle;


void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGD(TAG, "Notification of a time synchronization event");
}

static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK( esp_netif_init() );

    while ( Wifi_IsConnected() != ESP_OK )
    {
        vTaskDelay( pdMS_TO_TICKS(5000) );
    }

    ESP_LOGI(TAG, "Initializing and starting SNTP to... %s", CONFIG_NTP_SERVER_NAME);

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_NTP_SERVER_NAME);
    config.sync_cb = time_sync_notification_cb;     // Note: This is only needed if we want
    esp_netif_sntp_init(&config);

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 3;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }
    time(&now);
    localtime_r(&now, &timeinfo);
#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
    if (PCF8563_isInitialized() == true)
    {
        rtc_date_t rtcdate;
        rtcdate.year = timeinfo.tm_year+1900;
        rtcdate.month = timeinfo.tm_mon+1;
        rtcdate.day = timeinfo.tm_mday;
        rtcdate.hour = timeinfo.tm_hour;
        rtcdate.minute = timeinfo.tm_min;
        rtcdate.second = timeinfo.tm_sec;
        PCF8563_SetTime(&rtcdate);
    }
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT

    esp_netif_sntp_deinit();
}

void sntp_main()
{
    // Set timezone to Japan Standard Time and print local time
    setenv("TZ", "JST-9", 1);
    tzset();

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    char strftime_buf[64];
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Japan is: %s", strftime_buf);
}


#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
static QueueHandle_t gpio_evt_clock_queue = NULL;
static QueueHandle_t gpio_evt_int_queue = NULL;
static void IRAM_ATTR gpio_clock_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_clock_queue, &gpio_num, NULL);
}

static void IRAM_ATTR gpio_int_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_int_queue, &gpio_num, NULL);
}

static void gpio_clock_task(void* arg)
{
#if CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
    bool bTm1637Init = false;
    DigitDisplay_t* digitdisplay_1;
    uint8_t listDisp_1[XDIGIT_DISPLAY_DIGIT_COUNT];
    Tm1637_Init();
    if (Tm1637_Enable(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN) == ESP_OK) {
        digitdisplay_1 = Tm1637_Attach(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN, BRIGHT_DARKEST, XDIGIT_DISPLAY_DIGIT_COUNT);
        bTm1637Init = true;
        Tm1637_ClearDisplay(digitdisplay_1);
    } else {
        ESP_LOGE(TAG, "Digit Display Tm1637_Enable Error");
    }
#endif //CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT

    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_clock_queue, &io_num, portMAX_DELAY)) {
//            ESP_LOGI(TAG, "CLOCK GPIO[%"PRIu32"] intr, val: %d", io_num, gpio_get_level(io_num));
            rtc_date_t rtcdate;
            PCF8563_GetTime(&rtcdate);
#if CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
            if (bTm1637Init == true)
            {
                uint8_t hour10 = rtcdate.hour / 10;
                uint8_t hour01 = ( rtcdate.hour % 10 );
                uint8_t minute10 = rtcdate.minute / 10;
                uint8_t minute01 = ( rtcdate.minute % 10 );
                uint8_t second10 = rtcdate.second / 10;
                uint8_t second01 = ( rtcdate.second % 10 );
                listDisp_1[0] = hour10;
                listDisp_1[1] = hour01;
                listDisp_1[2] = minute10;
                listDisp_1[3] = minute01;
                listDisp_1[4] = second10;
                listDisp_1[5] = second01;
//                Tm1637_DisplayAll(digitdisplay_1, listDisp_1);
                Tm1637_DisplayClockDot(digitdisplay_1, listDisp_1);
            }
            else
            {
#endif //CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
                ESP_LOGI(TAG, "%04d/%02d/%02d %02d:%02d:%02d", rtcdate.year, rtcdate.month, rtcdate.day, rtcdate.hour, rtcdate.minute, rtcdate.second);
#if CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
            }
#endif //CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
        }
    }
}

void RtcSntpUpdateTime()
{
    Wifi_Start();
    sntp_main();
    Wifi_Stop();
}

static void gpio_int_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_int_queue, &io_num, portMAX_DELAY)) {
//            ESP_LOGI(TAG, "INT GPIO[%"PRIu32"] intr, val: %d", io_num, gpio_get_level(io_num));
            RtcSntpUpdateTime();
        }
    }
}

void RtcInterruptInit()
{
    ESP_LOGD(TAG, "start RtcInterruptInit()");

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = ((1ULL<<PCF8563__CLOCK_PIN) | (1ULL<<PCF8563__INT_PIN));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_evt_clock_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_evt_int_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_clock_task, "gpio_clock_task", 4096 * 1, NULL, 10, NULL);
    xTaskCreate(gpio_int_task, "gpio_int_task", 4096 * 1, NULL, 10, NULL);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PCF8563__CLOCK_PIN, gpio_clock_isr_handler, (void*) PCF8563__CLOCK_PIN));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PCF8563__INT_PIN, gpio_int_isr_handler, (void*) PCF8563__INT_PIN));
    ESP_ERROR_CHECK(gpio_intr_enable(PCF8563__CLOCK_PIN));
    ESP_ERROR_CHECK(gpio_intr_enable(PCF8563__INT_PIN));
    ESP_ERROR_CHECK(gpio_set_intr_type(PCF8563__CLOCK_PIN, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(PCF8563__INT_PIN, GPIO_INTR_NEGEDGE));

}
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT

void RtcStartClockInit()
{
    ESP_LOGD(TAG, "start RtcStartClockInit()");
#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
#if CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT

    while (!PCF8563_isInitialized()) {
        vTaskDelay( pdMS_TO_TICKS(5000) );
    }
    g_clockout_status = 1;
    PCF8563_ClockOutForTrimmer(g_clockout_status);
    ESP_LOGI(TAG, "PCF8563_ClockOutForTrimmer: %d", g_clockout_status);

#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
}

void RtcStartWifiSntpInit()
{
    ESP_LOGI(TAG, "start RtcStartWifiSntpInit()");
#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
    while (!PCF8563_isInitialized()) {
        vTaskDelay( pdMS_TO_TICKS(5000) );
    }
    int8_t minute = 56;
    int8_t hour = -1;
    int8_t day = -1;
    int8_t week = -1;
    PCF8563_SetAlarmIRQ(minute, hour, day, week);
    ESP_LOGI(TAG, "PCF8563_SetAlarmIRQ: minute %d, hour %d, day %d, week %d", minute, hour, day, week);
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
}


i2c_master_bus_handle_t i2c0_master_bus_handle;
static void clock_main()
{
    ESP_LOGD(TAG, "start EXTERNAL I2C");
    esp_err_t ret = ESP_OK;
    i2c_master_bus_config_t i2c_mst_config_0 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C0_MASTER_PORT,
        .scl_io_num = I2C0_MASTER_SCL_PIN,
        .sda_io_num = I2C0_MASTER_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ret = i2c_new_master_bus(&i2c_mst_config_0, &i2c0_master_bus_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGD(TAG, "i2c_new_master_bus is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_new_master_bus error");
    }

#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
    ret = PCF8563_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "PCF8563_Init() is OK!");
        Wifi_Init();
        RtcSntpUpdateTime();

        RtcInterruptInit();
        RtcStartClockInit();
        RtcStartWifiSntpInit();
    }
    else
    {
        ESP_LOGE(TAG, "PCF8563_Init Error");
    }
#endif
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void app_main(void)
{
    ESP_LOGI(TAG, "app_main() start.");
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("MY-MAIN", ESP_LOG_INFO);
    esp_log_level_set("MY-WIFI", ESP_LOG_INFO);

#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
    clock_main();
#endif //CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT

}
