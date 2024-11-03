// https://github.com/m5stack/M5Unit-ENV/blob/master/src/SHT4X.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#define SHT4x_NOHEAT_HIGHPRECISION \
    (0xFD) /**< High precision measurement, no heater */
#define SHT4x_NOHEAT_MEDPRECISION \
    (0xF6) /**< Medium precision measurement, no heater */
#define SHT4x_NOHEAT_LOWPRECISION \
    (0xE0) /**< Low precision measurement, no heater */

#define SHT4x_HIGHHEAT_1S \
    (0x39) /**< High precision measurement, high heat for 1 sec */
#define SHT4x_HIGHHEAT_100MS \
    (0x32) /**< High precision measurement, high heat for 0.1 sec */
#define SHT4x_MEDHEAT_1S \
    (0x2F) /**< High precision measurement, med heat for 1 sec */
#define SHT4x_MEDHEAT_100MS \
    (0x24) /**< High precision measurement, med heat for 0.1 sec */
#define SHT4x_LOWHEAT_1S \
    (0x1E) /**< High precision measurement, low heat for 1 sec */
#define SHT4x_LOWHEAT_100MS \
    (0x15) /**< High precision measurement, low heat for 0.1 sec */

#define SHT4x_READSERIAL (0x89) /**< Read Out of Serial Register */
#define SHT4x_SOFTRESET  (0x94) /**< Soft Reset */

typedef enum {
    SHT4X_HIGH_PRECISION,
    SHT4X_MED_PRECISION,
    SHT4X_LOW_PRECISION,
} sht4x_precision_t;

/** Optional pre-heater configuration setting */
typedef enum {
    SHT4X_NO_HEATER,
    SHT4X_HIGH_HEATER_1S,
    SHT4X_HIGH_HEATER_100MS,
    SHT4X_MED_HEATER_1S,
    SHT4X_MED_HEATER_100MS,
    SHT4X_LOW_HEATER_1S,
    SHT4X_LOW_HEATER_100MS,
} sht4x_heater_t;

esp_err_t Sht4x_Init(i2c_master_bus_handle_t i2c_master_bus_handle);
esp_err_t Sht4x_DeInit();
esp_err_t Sht4x_Read();
void Sht4x_SetPrecision(sht4x_precision_t prec);
sht4x_precision_t Sht4x_GetPrecision();
void Sht4x_SetHeater(sht4x_heater_t heat);
sht4x_heater_t Sht4x_GetHeater();
float Sht4x_GetTemperature();
int32_t Sht4x_GetIntTemperature();
float Sht4x_GetHumidity();
int32_t Sht4x_GetIntHumidity();


#ifdef __cplusplus
}
#endif
