#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "mysystem.h"

#define HT16K33_COM0    (0x00)
#define HT16K33_COM1    (0x01)
#define HT16K33_COM2    (0x02)
#define HT16K33_COM3    (0x03)
#define HT16K33_COM4    (0x04)
#define HT16K33_COM5    (0x05)
#define HT16K33_COM6    (0x06)
#define HT16K33_COM7    (0x07)

#define HT16K33_SEG_OFF (0x00)
#define HT16K33_SEG_A   (0x01)
#define HT16K33_SEG_B   (0x02)
#define HT16K33_SEG_C   (0x04)
#define HT16K33_SEG_D   (0x08)
#define HT16K33_SEG_E   (0x10)
#define HT16K33_SEG_F   (0x20)
#define HT16K33_SEG_G   (0x40)
#define HT16K33_SEG_DP  (0x80)

#define HT16K33_SEG_D1  (0x01)
#define HT16K33_SEG_D2  (0x02)
#define HT16K33_SEG_D3  (0x04)
#define HT16K33_SEG_L1  (0x01)
#define HT16K33_SEG_L2  (0x02)
#define HT16K33_SEG_L3  (0x04)

#define HT16K33_COM_FIRST_HALF    (0x00)
#define HT16K33_COM_SECOND_HALF   (0x01)

esp_err_t HT16K33_Init(i2c_master_bus_handle_t i2c_master_bus_handle);
esp_err_t HT16K33_DeInit();
esp_err_t HT16K33_SetBrigtness(uint8_t b);
esp_err_t HT16K33_DisplayClear();
esp_err_t HT16K33_DisplayFromRawData(uint8_t com, uint8_t raw1, uint8_t raw2);
esp_err_t HT16K33_DisplayFromRawDataAt1Byte(uint8_t com, uint8_t half, uint8_t raw1);
esp_err_t HT16K33_ParseFloatToDigit2Point1(float value, uint8_t segments_data[], uint8_t size);
esp_err_t HT16K33_ParseTimeToDigitClockAndPulse(uint8_t hour, uint8_t minute, uint8_t second, uint8_t segments_data[], uint8_t size);
esp_err_t HT16K33_ParseTimeToSecondAndPulse(uint8_t second, uint8_t segments_data[], uint8_t size);
esp_err_t HT16K33_ParseTimeToMinute(uint8_t minute, uint8_t segments_data[], uint8_t size);
esp_err_t HT16K33_ParseTimeToHour(uint8_t hour, uint8_t segments_data[], uint8_t size);

esp_err_t HT16K33_ShowTest();
