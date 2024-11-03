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
    ESP_LOGI(TAG, "Using \"%02X:%02X:%02X:%02X:%02X:%02X\" as base MAC address",
             base_mac_addr[0], base_mac_addr[1], base_mac_addr[2], base_mac_addr[3], base_mac_addr[4], base_mac_addr[5]);

    snprintf(hostname, size, "%s-%02x%02x%02x", getChipModelName(), base_mac_addr[3], base_mac_addr[4], base_mac_addr[5]);
    return ESP_OK;
}

uint8_t convertNumberToSegments(int8_t number)
{
    switch (number) {
        case 0 : return 0x3f;
        case 1 : return 0x06;
        case 2 : return 0x5b;
        case 3 : return 0x4f;
        case 4 : return 0x66;
        case 5 : return 0x6d;
        case 6 : return 0x7d;
        case 7 : return 0x07;
        case 8 : return 0x7f;
        case 9 : return 0x6f;
    }
    return 0x00;
}

uint8_t convertCharToSegments(char c)
{
    switch (c) {
        case '0' : return 0x3f;
        case '1' : return 0x06;
        case '2' : return 0x5b;
        case '3' : return 0x4f;
        case '4' : return 0x66;
        case '5' : return 0x6d;
        case '6' : return 0x7d;
        case '7' : return 0x07;
        case '8' : return 0x7f;
        case '9' : return 0x6f;
        case ':' : return 0x03;
        case '_' : return 0x08;
        case '^' : return 0x01; // ￣
        case '-' : return 0x40;
        case '*' : return 0x63; // °
        case ' ' : return 0x00; // space
        case 'A' : return 0x77; // upper case A
        case 'a' : return 0x5f; // lower case a
        case 'B' :              // lower case b
        case 'b' : return 0x7c; // lower case b
        case 'C' : return 0x39; // upper case C
        case 'c' : return 0x58; // lower case c
        case 'D' :              // lower case d
        case 'd' : return 0x5e; // lower case d
        case 'E' :              // upper case E
        case 'e' : return 0x79; // upper case E
        case 'F' :              // upper case F
        case 'f' : return 0x71; // upper case F
        case 'G' :              // upper case G
        case 'g' : return 0x35; // upper case G
        case 'H' : return 0x76; // upper case H
        case 'h' : return 0x74; // lower case h
        case 'I' : return 0x06; // 1
        case 'i' : return 0x04; // lower case i
        case 'J' : return 0x1e; // upper case J
        case 'j' : return 0x16; // lower case j
        case 'K' :              // upper case K
        case 'k' : return 0x75; // upper case K
        case 'L' :              // upper case L
        case 'l' : return 0x38; // upper case L
        case 'M' :              // twice tall n
        case 'm' : return 0x37; // twice tall ∩
        case 'N' :              // lower case n
        case 'n' : return 0x54; // lower case n
        case 'O' :              // lower case o
        case 'o' : return 0x5c; // lower case o
        case 'P' :              // upper case P
        case 'p' : return 0x73; // upper case P
        case 'Q' : return 0x7b; // upper case Q
        case 'q' : return 0x67; // lower case q
        case 'R' :              // lower case r
        case 'r' : return 0x50; // lower case r
        case 'S' :              // 5
        case 's' : return 0x6d; // 5
        case 'T' :              // lower case t
        case 't' : return 0x78; // lower case t
        case 'U' :              // lower case u
        case 'u' : return 0x1c; // lower case u
        case 'V' :              // twice tall u
        case 'v' : return 0x3e; // twice tall u
        case 'W' : return 0x7e; // upside down A
        case 'w' : return 0x2a; // separated w
        case 'X' :              // upper case H
        case 'x' : return 0x76; // upper case H
        case 'Y' :              // lower case y
        case 'y' : return 0x6e; // lower case y
        case 'Z' :              // separated Z
        case 'z' : return 0x1b; // separated Z
    }
    return 0x00;
}
