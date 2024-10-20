#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_chip_info.h"

esp_err_t createHostnameFromChipAndMacAddress(char *hostname, uint8_t size);

#ifdef __cplusplus
}
#endif