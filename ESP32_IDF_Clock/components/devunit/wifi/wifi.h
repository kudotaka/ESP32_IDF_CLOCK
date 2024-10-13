#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"

#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
esp_err_t wifi_isConnected(void);
void wifi_initialise(void);
void wifi_stop(void);
#endif

#ifdef __cplusplus
}
#endif