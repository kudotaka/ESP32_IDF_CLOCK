#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

typedef struct _rtc_data_t {
    uint16_t year;  //Date year.
    uint8_t month;  //Date month.
    uint8_t day;    //Date day.
    uint8_t hour;   //Time hour.
    uint8_t minute; //Time minute.
    uint8_t second; //Time second.
} rtc_date_t;

esp_err_t PCF8563_Init(i2c_master_bus_handle_t i2c_master_bus_handle);
esp_err_t PCF8563_DeInit();
esp_err_t PCF8563_SetTime(rtc_date_t* data);
esp_err_t PCF8563_GetTime(rtc_date_t* data);
esp_err_t PCF8563_SetAlarmIRQ(int8_t minute, int8_t hour, int8_t day, int8_t week);
int16_t PCF8563_SetTimerIRQ(int16_t value);
int16_t PCF8563_GetTimerTime();
uint8_t PCF8563_GetIRQ();
esp_err_t PCF8563_ClearIRQ();

#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
bool PCF8563_isInitialized(void);
#endif

#if CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT
esp_err_t PCF8563_ClockOutForTrimmer(bool enable_clko);
#endif

#ifdef __cplusplus
}
#endif