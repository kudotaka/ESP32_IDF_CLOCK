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
uint8_t convertCharToSegments(char c);
uint8_t convertNumberToSegments(int8_t number);

#ifdef __cplusplus
}
#endif