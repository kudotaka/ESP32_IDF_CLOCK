#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"

#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
esp_err_t Wifi_Init(void);
esp_err_t Wifi_DeInit(void);

esp_err_t Wifi_Start(void);
esp_err_t Wifi_Stop(void);
esp_err_t Wifi_Connect(void);
esp_err_t Wifi_Disconnect(void);
esp_err_t Wifi_IsConnected(void);
#endif

#ifdef __cplusplus
}
#endif