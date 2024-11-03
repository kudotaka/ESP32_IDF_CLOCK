#include "freertos/FreeRTOS.h"
#include "math.h"
#include "sht4x.h"

#define SHT4X_ADDR (0x44) /**< SHT4x I2C Address */
#define SHT4X_STANDARD_BAUD (400000)
#define SHT4X_TIMEOUT_VALUE_MS (100)
static i2c_master_dev_handle_t dev_sht4x_device_handle;

static const char *TAG = "MY-SHT4X";

float cTemp    = 0;
float humidity = 0;
sht4x_precision_t _precision = SHT4X_HIGH_PRECISION;
sht4x_heater_t _heater       = SHT4X_NO_HEATER;


static esp_err_t I2CWrite(uint8_t* cmd, uint8_t len) {
    esp_err_t ret = ESP_OK;

    ret = i2c_master_transmit(dev_sht4x_device_handle, cmd, len, SHT4X_TIMEOUT_VALUE_MS);
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

static esp_err_t I2CRead(uint8_t* buf, uint8_t len) {
    esp_err_t ret = ESP_OK;

    ret = i2c_master_receive(dev_sht4x_device_handle, buf, len, SHT4X_TIMEOUT_VALUE_MS);
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

uint8_t crc8(const uint8_t *data, int len) {
    /*
     *
     * CRC-8 formula from page 14 of SHT spec pdf
     *
     * Test data 0xBE, 0xEF should yield 0x92
     *
     * Initialization data 0xFF
     * Polynomial 0x31 (x8 + x5 +x4 +1)
     * Final XOR 0x00
     */

    const uint8_t POLYNOMIAL = (0x31);
    uint8_t crc = (0xFF);

    for (int j = len; j; --j) {
        crc ^= *data++;

        for (int i = 8; i; --i) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}

float Sht4x_GetTemperature() {
    return cTemp;
}

int32_t Sht4x_GetIntTemperature() {
    return (int32_t)cTemp;
}

float Sht4x_GetHumidity() {
    return humidity;
}

int32_t Sht4x_GetIntHumidity() {
    return (int32_t)humidity;
}

esp_err_t Sht4x_Read() {
    esp_err_t ret = ESP_OK;
    uint8_t readbuffer[6];
    uint8_t cmd[1]       = { SHT4x_NOHEAT_HIGHPRECISION };
    uint16_t duration = 10;

    if (_heater == SHT4X_NO_HEATER) {
        if (_precision == SHT4X_HIGH_PRECISION) {
            cmd[0]   = SHT4x_NOHEAT_HIGHPRECISION;
            duration = 10;
        }
        if (_precision == SHT4X_MED_PRECISION) {
            cmd[0]   = SHT4x_NOHEAT_MEDPRECISION;
            duration = 5;
        }
        if (_precision == SHT4X_LOW_PRECISION) {
            cmd[0]   = SHT4x_NOHEAT_LOWPRECISION;
            duration = 2;
        }
    }

    if (_heater == SHT4X_HIGH_HEATER_1S) {
        cmd[0]   = SHT4x_HIGHHEAT_1S;
        duration = 1100;
    }
    if (_heater == SHT4X_HIGH_HEATER_100MS) {
        cmd[0]   = SHT4x_HIGHHEAT_100MS;
        duration = 110;
    }

    if (_heater == SHT4X_MED_HEATER_1S) {
        cmd[0]   = SHT4x_MEDHEAT_1S;
        duration = 1100;
    }
    if (_heater == SHT4X_MED_HEATER_100MS) {
        cmd[0]   = SHT4x_MEDHEAT_100MS;
        duration = 110;
    }

    if (_heater == SHT4X_LOW_HEATER_1S) {
        cmd[0]   = SHT4x_LOWHEAT_1S;
        duration = 1100;
    }
    if (_heater == SHT4X_LOW_HEATER_100MS) {
        cmd[0]   = SHT4x_LOWHEAT_100MS;
        duration = 110;
    }

    ret = I2CWrite(cmd, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Sht4x_Read() I2CWrite error");
        return ret;
    } 

    vTaskDelay( pdMS_TO_TICKS(duration) );

    ret = I2CRead(readbuffer, 6);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Sht4x_Read() I2CRead error");
        return ret;
    } 

    if (readbuffer[2] != crc8(readbuffer, 2) ||
        readbuffer[5] != crc8(readbuffer + 3, 2)) {
        return ESP_ERR_INVALID_CRC;
    }

    float t_ticks  = (uint16_t)readbuffer[0] * 256 + (uint16_t)readbuffer[1];
    float rh_ticks = (uint16_t)readbuffer[3] * 256 + (uint16_t)readbuffer[4];

    cTemp    = -45 + 175 * t_ticks / 65535;
    humidity = -6 + 125 * rh_ticks / 65535;
    humidity = fmin(fmax(humidity, (float)0.0), (float)100.0);
    return ret;
}

void Sht4x_SetPrecision(sht4x_precision_t prec) {
    _precision = prec;
}

sht4x_precision_t Sht4x_GetPrecision(void) {
    return _precision;
}

void Sht4x_SetHeater(sht4x_heater_t heat) {
    _heater = heat;
}

sht4x_heater_t Sht4x_GetHeater(void) {
    return _heater;
}

esp_err_t Sht4x_SoftReset() {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1] = {0x94};

    ret = I2CWrite(cmd, 1);
    vTaskDelay( pdMS_TO_TICKS(100) );

    return ret;
}

esp_err_t Sht4x_DeInit() {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_sht4x_device_handle);
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

esp_err_t Sht4x_Init(i2c_master_bus_handle_t i2c_master_bus_handle) {
    ESP_LOGI(TAG, "Sht4x_Init Init()");
    esp_err_t ret = ESP_OK;

    i2c_device_config_t dev_sht4x_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT4X_ADDR,
        .scl_speed_hz = SHT4X_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_sht4x_cfg, &dev_sht4x_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_add_device error");
    }

    // device check
    ret = Sht4x_SoftReset();
    return ret;
}
