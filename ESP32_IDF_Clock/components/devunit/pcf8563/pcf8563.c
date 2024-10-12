#include "pcf8563.h"
#include <string.h>

#define PCF8563_ADDR (0x51)
#define PCF8563_STANDARD_BAUD (100000)
#define PCF8563_TIMEOUT_VALUE_MS (100)

static const char *TAG = "MY-PCF8563";

static i2c_master_dev_handle_t dev_pcf8563_device_handle;

bool bInitialized = false;


static esp_err_t I2CWrite(uint8_t addr, uint8_t* buf, uint8_t len) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1 + len];
    cmd[0] = addr;
    for (uint8_t i = 0; i < len; i++)
    {
        cmd[i + 1] = buf[i];
    }

    ret = i2c_master_transmit(dev_pcf8563_device_handle, cmd, 1 + len, PCF8563_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK)
    {
//        ESP_LOGI(TAG, "I2CWrite i2c_master_transmit(1) is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2CWrite I2C i2c_master_transmit(1) error");
    }
    return ret;
}

static esp_err_t I2CRead(uint8_t addr, uint8_t* buf, uint8_t len) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[] = { 0 };
    cmd[0] = addr;

    ret = i2c_master_transmit_receive(dev_pcf8563_device_handle, cmd, 1, buf, len, PCF8563_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK)
    {
//        ESP_LOGI(TAG, "I2CRead i2c_master_transmit_receive is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2CRead I2C i2c_master_transmit_receive error");
    }
    return ret;
}

static uint8_t Byte2BCD(uint8_t data) {
    return ((data / 10) << 4) + data % 10;
}

static uint8_t BCD2Byte(uint8_t data) {
    return (data >> 4) * 10 + (data & 0x0f);
}

bool PCF8563_isInitialized(void) {
    return bInitialized;
}

esp_err_t PCF8563_SetTime(rtc_date_t* data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t time_buf[7];
    time_buf[0] = Byte2BCD(data->second);
    time_buf[1] = Byte2BCD(data->minute);
    time_buf[2] = Byte2BCD(data->hour);
    time_buf[3] = Byte2BCD(data->day);
    time_buf[5] = Byte2BCD(data->month) | (data->year >= 2000 ? 0x00 : 0x80);
    time_buf[6] = Byte2BCD(data->year % 100);

    return I2CWrite(0x02, time_buf, 7);
}

esp_err_t PCF8563_GetTime(rtc_date_t* data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret;
    uint8_t time_buf[7];
    ret = I2CRead(0x02, time_buf, 7);
    data->second = BCD2Byte(time_buf[0] & 0x7f);
    data->minute = BCD2Byte(time_buf[1] & 0x7f);
    data->hour = BCD2Byte(time_buf[2] & 0x3f);
    data->day = BCD2Byte(time_buf[3] & 0x3f);
    data->month = BCD2Byte(time_buf[5] & 0x1f);
    data->year = BCD2Byte(time_buf[6]) + (time_buf[5] & 0x80 ? 1900 : 2000);
//    ESP_LOGI(TAG, "PCF8563_GetTime %04d/%02d/%02d %02d:%02d:%02d", data->year, data->month, data->day, data->hour, data->minute, data->second);
    return ret;
}

// -1 :disable
esp_err_t PCF8563_SetAlarmIRQ(int8_t minute, int8_t hour, int8_t day, int8_t week) {
    esp_err_t ret;
    uint8_t irq_enable = false;
    uint8_t out_buf[4] = { 0x80, 0x80, 0x80, 0x80 };
    if(minute >= 0) {
        irq_enable = true;
        out_buf[0] = Byte2BCD(minute) & 0x7f;
    }

    if(hour >= 0) {
        irq_enable = true;
        out_buf[1] = Byte2BCD(hour) & 0x3f;
    }

    if(day >= 0) {
        irq_enable = true;
        out_buf[2] = Byte2BCD(day) & 0x3f;
    }

    if(week >= 0) {
        irq_enable = true;
        out_buf[3] = Byte2BCD(week) & 0x07;
    }

    uint8_t reg_value = 0;
    ret = I2CRead(0x01, &reg_value, 1);
    if (irq_enable) {
        reg_value |= (1 << 1);
    } else {
        reg_value &= ~(1 << 1);
    }

    ret = I2CWrite(0x09, out_buf, 4);
    ret = I2CWrite(0x01, &reg_value, 1);
    return ret;
}

// -1: disable
int16_t PCF8563_SetTimerIRQ(int16_t value) {
    uint8_t reg_value = 0;
    I2CRead(0x01, &reg_value, 1);

    if (value < 0) {
        reg_value &= ~(1 << 0);
        I2CWrite(0x01, &reg_value, 1);
        reg_value = 0x03;
        I2CWrite(0x0E, &reg_value, 1);
        return -1;
    }

    uint8_t type_value = 2;
    uint8_t div = 1;
    if (value > 255) {
        div = 60;
        type_value = 0x83;
    } else {
        type_value = 0x82;
    }
    value = (value / div) & 0xFF;
    I2CWrite(0x0F, (uint8_t *)&value, 1);
    I2CWrite(0x0E, &type_value, 1);

    reg_value |= (1 << 0);
    reg_value &= ~(1 << 7);
    I2CWrite(0x01, &reg_value, 1);
    return value * div;
}

int16_t PCF8563_GetTimerTime() {
    uint8_t value = 0;
    uint8_t type_value = 0;
    I2CRead(0x0f, &value, 1);
    I2CRead(0x0e, &type_value, 1);

    if ((type_value & 0x03) == 3) {
        return value * 60;
    } else {
        return value;
    }
}

uint8_t PCF8563_GetIRQ() {
    uint8_t data;
    I2CRead(0x01, &data, 1);
    return data;
}

esp_err_t PCF8563_ClearIRQ() {
    esp_err_t ret;
    uint8_t data;
    uint8_t cmd[] = { 0 };
    ret = I2CRead(0x01, &data, 1);
    cmd[0] = data & 0xf3;
    ret = I2CWrite(0x01, cmd, 1);

    return ret;
}

#if CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT
esp_err_t PCF8563_ClockOutForTrimmer(bool enable_clko)
{
    esp_err_t ret = ESP_OK;
    uint8_t clock_out = 3; //fco_1Hz
    uint8_t data = 0;
    uint8_t cmd[] = { 0 };
    if (enable_clko)
    {
        data = clock_out;
        data |= 0x80;
        cmd[0] = data;
        ret = I2CWrite(0x0D, cmd, 1);
    }
    else
    {
        data = clock_out;
        cmd[0] = data;
        ret = I2CWrite(0x0D, cmd, 1);
    }

    return ret;
}
#endif

esp_err_t PCF8563_DeInit() {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_pcf8563_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_rm_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_rm_device error");
    }   
    return ret;
}

esp_err_t PCF8563_Init(i2c_master_bus_handle_t i2c_master_bus_handle) {
    esp_err_t ret = ESP_OK;
    i2c_device_config_t dev_pcf8563_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCF8563_ADDR,
        .scl_speed_hz = PCF8563_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_pcf8563_cfg, &dev_pcf8563_device_handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_add_device error");
    }

    uint8_t cmd[] = { 0 };
    cmd[0] = 0x00;
    ret |= I2CWrite(0x00, cmd, 1);
    ret |= I2CWrite(0x01, cmd, 1);
    ret |= I2CWrite(0x0D, cmd, 1);

    if (ret == ESP_OK)
    {
        bInitialized = true;
    }
    return ret;
}