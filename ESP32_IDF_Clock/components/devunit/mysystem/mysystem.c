#include "freertos/FreeRTOS.h"
#include "mysystem.h"

static const char *TAG = "MY-SYSTEM";

const char* getChipModelName()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    switch (chip_info.model)
    {
        case CHIP_ESP32:
            return "ESP32";
        case CHIP_ESP32S2:
            return "ESP32S2";
        case CHIP_ESP32S3:
            return "ESP32S3";
        case CHIP_ESP32C3:
            return "ESP32C3";
        case CHIP_ESP32C2:
            return "ESP32C2";
        case CHIP_ESP32C6:
            return "ESP32C6";
        case CHIP_ESP32H2:
            return "ESP32H2";
        case CHIP_ESP32P4:
            return "ESP32P4";
        case CHIP_ESP32C61:
            return "ESP32C61";
//        case CHIP_POSIX_LINUX:
//            return "ESP32-simulator";
        default:
            return "ESP-UNKNOWN";
    }
}

esp_err_t createHostnameFromChipAndMacAddress(char *hostname, uint8_t size)
{
    uint8_t base_mac_addr[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(base_mac_addr, ESP_MAC_BASE));
    ESP_LOGI(TAG, "Using \"%X:%X:%X:%X:%X:%X\" as base MAC address",
             base_mac_addr[0], base_mac_addr[1], base_mac_addr[2], base_mac_addr[3], base_mac_addr[4], base_mac_addr[5]);

    snprintf(hostname, size, "%s-%x%x%x", getChipModelName(), base_mac_addr[3], base_mac_addr[4], base_mac_addr[5]);
    return ESP_OK;
}
