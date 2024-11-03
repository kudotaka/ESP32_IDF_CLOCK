#include "ht16k33.h"
#include <string.h>

#define HT16K33_ADDR (0x70)
#define HT16K33_STANDARD_BAUD (400000)
#define HT16K33_TIMEOUT_VALUE_MS (100)

#define HT16K33_CMD_POWER_OFF  (0x20)
#define HT16K33_CMD_POWER_ON   (0x21)
#define HT16K33_CMD_DISPLAY_OFF  (0x80)
#define HT16K33_CMD_DISPLAY_ON   (0x81)
#define HT16K33_CMD_BLINK_OFF   (0x81)
#define HT16K33_CMD_BLINK_2HZ   (0x83)
#define HT16K33_CMD_BLINK_1HZ   (0x85)
#define HT16K33_CMD_BLINK_05HZ   (0x87)
#define HT16K33_CMD_BRIGHTNESS_BASE   (0xE0)

#define HT16K33_BRIGHTNESS_MAX   (0x05)

static const char *TAG = "MY-HT16K33";

static i2c_master_dev_handle_t dev_ht16k33_device_handle;

static esp_err_t I2CWriteWithAddr(uint8_t addr, uint8_t* buf, uint8_t len) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1 + len];
    cmd[0] = addr;
    for (uint8_t i = 0; i < len; i++)
    {
        cmd[i + 1] = buf[i];
    }

    ret = i2c_master_transmit(dev_ht16k33_device_handle, cmd, 1 + len, HT16K33_TIMEOUT_VALUE_MS);
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

static esp_err_t I2CWrite(uint8_t* cmd, uint8_t len)
{
    esp_err_t ret = ESP_OK;

    ret = i2c_master_transmit(dev_ht16k33_device_handle, cmd, len, HT16K33_TIMEOUT_VALUE_MS);
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

    ret = i2c_master_transmit_receive(dev_ht16k33_device_handle, cmd, 1, buf, len, HT16K33_TIMEOUT_VALUE_MS);
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


esp_err_t HT16K33_DisplayFromRawData(uint8_t com, uint8_t raw1, uint8_t raw2)
{
    uint8_t cmd[] = { 0, 0 };
    cmd[0] = raw1;
    cmd[1] = raw2;
    return I2CWriteWithAddr(0x00 | com * 2, cmd, 2);
}

esp_err_t HT16K33_DisplayFromRawDataAt1Byte(uint8_t com, uint8_t half, uint8_t raw1)
{
    uint8_t cmd[] = { 0 };
    cmd[0] = raw1;
    return I2CWriteWithAddr(0x00 | com * 2 | half, cmd, 1);
}

esp_err_t HT16K33_ParseFloatToDigit2Point1(float value, uint8_t segments_data[], uint8_t size)
{
    if ( size < 3)
    {
        return ESP_ERR_INVALID_ARG;
    }
    int32_t convertValue = value * 10;
    if (convertValue > 999)
    {
        segments_data[0] = segments_data[1] = segments_data[2] = convertCharToSegments('^');
    }
    else if (convertValue > 99)
    {
        segments_data[0] = convertNumberToSegments(convertValue / 100);
        segments_data[1] = (convertNumberToSegments(( convertValue % 100 ) / 10)) | HT16K33_SEG_DP;
        segments_data[2] = convertNumberToSegments(( convertValue % 100 ) % 10);
    }
    else if (convertValue > 0)
    {
        segments_data[0] = convertCharToSegments(' ');
        segments_data[1] = (convertNumberToSegments(( convertValue % 100 ) / 10)) | HT16K33_SEG_DP;
        segments_data[2] = convertNumberToSegments(( convertValue % 100 ) % 10);
    }
    else if (convertValue == 0)
    {
        segments_data[0] = convertCharToSegments(' ');
        segments_data[1] = (convertCharToSegments('0')) | HT16K33_SEG_DP;
        segments_data[2] = convertCharToSegments('0');
    }
    else if (convertValue > -100 )
    {
        int32_t convertAbsValue = abs(convertValue);
        segments_data[0] = convertCharToSegments('-');
        segments_data[1] = (convertNumberToSegments(convertAbsValue / 10)) | HT16K33_SEG_DP;
        segments_data[2] = convertNumberToSegments(convertAbsValue % 10);
    }
    else
    {
        segments_data[0] = segments_data[1] = segments_data[2] = convertCharToSegments('_');
    }

    return ESP_OK;
}

esp_err_t HT16K33_ParseTimeToSecondAndPulse(uint8_t second, uint8_t segments_data[], uint8_t size)
{
    if ( size < 1)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (second > 60)
    {
        return ESP_ERR_INVALID_ARG;
    }

//    uint8_t second10 = second / 10;
    uint8_t second01 = second % 10;
    if (second01 % 2 == 0)
    {
        //pulse OFF
        segments_data[0] = 0;
    }
    else
    {
        //pulse ON
        segments_data[0] = 1;
    }

    return ESP_OK;
}

esp_err_t HT16K33_ParseTimeToMinute(uint8_t minute, uint8_t segments_data[], uint8_t size)
{
    if ( size < 2)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (minute > 60)
    {
        return ESP_ERR_INVALID_ARG;
    }

    segments_data[0] = convertNumberToSegments(minute / 10);
    segments_data[1] = convertNumberToSegments(minute % 10);

    return ESP_OK;
}

esp_err_t HT16K33_ParseTimeToHour(uint8_t hour, uint8_t segments_data[], uint8_t size)
{
    if ( size < 2)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (hour > 23)
    {
        return ESP_ERR_INVALID_ARG;
    }

    segments_data[0] = convertNumberToSegments(hour / 10);
    segments_data[1] = convertNumberToSegments(hour % 10);

    return ESP_OK;
}

esp_err_t HT16K33_ParseTimeToDigitClockAndPulse(uint8_t hour, uint8_t minute, uint8_t second, uint8_t segments_data[], uint8_t size)
{
    if ( size < 5)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (hour > 23 || minute > 60 || second > 60)
    {
        return ESP_ERR_INVALID_ARG;
    }

    HT16K33_ParseTimeToHour(hour, &segments_data[0], 2);
    HT16K33_ParseTimeToMinute(minute, &segments_data[2], 2);
    HT16K33_ParseTimeToSecondAndPulse(second, &segments_data[4], 1);

    return ESP_OK;
}

esp_err_t HT16K33_ShowTest()
{
    ESP_LOGI(TAG, "start HT16K33_Show()");

    uint8_t parseArray[3] = { 0 };
    float floatArray[] = {100.0,99.9,10.0,9.9,0.1,0.0,-0.1,-9.9,-10.0,-99.9,-100.0};
    uint8_t floatArraySize = sizeof(floatArray)/sizeof(float);
    uint8_t clockArray[5] = { 0 };
    uint16_t MAX_TIME = 24 * 60;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    for (uint16_t time = 0; time < MAX_TIME; time++)
    {
        HT16K33_ParseFloatToDigit2Point1(floatArray[time % floatArraySize], parseArray, sizeof(floatArray)/sizeof(float));
        hour = time / 60;
        minute = time % 60;
        second = time % 60;
        HT16K33_ParseTimeToDigitClockAndPulse(hour, minute, second, clockArray, sizeof(clockArray)/sizeof(uint8_t));
        HT16K33_DisplayFromRawData(HT16K33_COM0, clockArray[0], parseArray[0]);
        HT16K33_DisplayFromRawData(HT16K33_COM1, clockArray[1], parseArray[1]);
        HT16K33_DisplayFromRawData(HT16K33_COM2, clockArray[2], parseArray[2]);
        HT16K33_DisplayFromRawData(HT16K33_COM3, clockArray[3], convertCharToSegments(' '));
        if (clockArray[4] == 0)
        {
            HT16K33_DisplayFromRawData(HT16K33_COM4, convertCharToSegments(' '), convertCharToSegments(' '));
        }
        else
        {
            HT16K33_DisplayFromRawData(HT16K33_COM4, convertCharToSegments(':'), convertCharToSegments(' '));
        }
        vTaskDelay( pdMS_TO_TICKS(200) );
    }
    vTaskDelay( pdMS_TO_TICKS(1000) );
    HT16K33_DisplayClear();


    HT16K33_DisplayFromRawData(HT16K33_COM0, convertNumberToSegments(1), convertNumberToSegments(2));
    HT16K33_DisplayFromRawData(HT16K33_COM1, convertCharToSegments('5'), convertCharToSegments('7'));
    HT16K33_DisplayFromRawData(HT16K33_COM2, convertNumberToSegments(3), convertNumberToSegments(4));
    HT16K33_DisplayFromRawData(HT16K33_COM3, convertNumberToSegments(6), convertNumberToSegments(8));
    HT16K33_DisplayFromRawData(HT16K33_COM4, convertCharToSegments(':'), convertNumberToSegments(-1));
    for (uint8_t i = 0; i < HT16K33_BRIGHTNESS_MAX; i++)
    {
        HT16K33_SetBrigtness(i);
        vTaskDelay( pdMS_TO_TICKS(1000) );
    }
    HT16K33_DisplayClear();

    for (uint8_t i = 0; i < 10; i++)
    {
        HT16K33_DisplayFromRawData(HT16K33_COM0, convertNumberToSegments(i), convertNumberToSegments(i));
        vTaskDelay( pdMS_TO_TICKS(1000) );
    }
    HT16K33_DisplayClear();
    char charArray[] = {'0','1','2','3','4','5','6','7','8','9','-','^','_'};
    for (uint8_t i = 0; i < sizeof(charArray)/sizeof(char); i++)
    {
        HT16K33_DisplayFromRawData(HT16K33_COM1, convertCharToSegments(charArray[i]) | HT16K33_SEG_DP, convertCharToSegments(charArray[i]) | HT16K33_SEG_DP);
        vTaskDelay( pdMS_TO_TICKS(1000) );
    }
    HT16K33_DisplayClear();


    return ESP_OK;
}

esp_err_t HT16K33_SetBlink(uint8_t b)
{
    uint8_t cmd[] = { 0 };
    cmd[0] = b;
    return I2CWrite(cmd, 1);
}

esp_err_t HT16K33_SetBrigtness(uint8_t b)
{
    uint8_t cmd[] = { 0 };
    if (b > HT16K33_BRIGHTNESS_MAX)
    {
        cmd[0] = HT16K33_CMD_BRIGHTNESS_BASE | HT16K33_BRIGHTNESS_MAX;
        return I2CWrite(cmd, 1);
    }
    cmd[0] = HT16K33_CMD_BRIGHTNESS_BASE | b;
    return I2CWrite(cmd, 1);
}

esp_err_t HT16K33_DisplayClear()
{
    esp_err_t ret = ESP_OK;
    ret |= HT16K33_DisplayFromRawData(HT16K33_COM0, HT16K33_SEG_OFF, HT16K33_SEG_OFF);
    ret |= HT16K33_DisplayFromRawData(HT16K33_COM1, HT16K33_SEG_OFF, HT16K33_SEG_OFF);
    ret |= HT16K33_DisplayFromRawData(HT16K33_COM2, HT16K33_SEG_OFF, HT16K33_SEG_OFF);
    ret |= HT16K33_DisplayFromRawData(HT16K33_COM3, HT16K33_SEG_OFF, HT16K33_SEG_OFF);
    ret |= HT16K33_DisplayFromRawData(HT16K33_COM4, HT16K33_SEG_OFF, HT16K33_SEG_OFF);
    ret |= HT16K33_DisplayFromRawData(HT16K33_COM5, HT16K33_SEG_OFF, HT16K33_SEG_OFF);
    ret |= HT16K33_DisplayFromRawData(HT16K33_COM6, HT16K33_SEG_OFF, HT16K33_SEG_OFF);
    ret |= HT16K33_DisplayFromRawData(HT16K33_COM7, HT16K33_SEG_OFF, HT16K33_SEG_OFF);
    return ret;
}

esp_err_t HT16K33_DisplayOn()
{
    esp_err_t ret = ESP_OK;
    uint8_t cmd[] = { 0 };
    cmd[0] = HT16K33_CMD_POWER_ON;
    ret |= I2CWrite(cmd, 1);
    cmd[0] = HT16K33_CMD_DISPLAY_ON;
    ret |= I2CWrite(cmd, 1);
    ret |= HT16K33_SetBrigtness(1);
    return ret;
}

esp_err_t HT16K33_DisplayOff()
{
    esp_err_t ret = ESP_OK;
    uint8_t cmd[] = { 0 };
    cmd[0] = HT16K33_CMD_DISPLAY_OFF;
    ret |= I2CWrite(cmd, 1);
    cmd[0] = HT16K33_CMD_POWER_OFF;
    ret |= I2CWrite(cmd, 1);
    return ret;
}

esp_err_t HT16K33_CheckDevice()
{
    uint8_t reg_value = 0;
    return I2CRead(0x60, &reg_value, 1);
}

esp_err_t HT16K33_DeInit()
{
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_ht16k33_device_handle);
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

esp_err_t HT16K33_Init(i2c_master_bus_handle_t i2c_master_bus_handle)
{
    esp_err_t ret = ESP_OK;
    i2c_device_config_t dev_ht16k33_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = HT16K33_ADDR,
        .scl_speed_hz = HT16K33_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_ht16k33_cfg, &dev_ht16k33_device_handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_add_device error");
    }

    ret = HT16K33_CheckDevice();
    if (ret == ESP_OK)
    {
        ret |= HT16K33_DisplayOff();
        ret |= HT16K33_DisplayOn();
        ret |= HT16K33_DisplayClear();
    }
    return ret;
}