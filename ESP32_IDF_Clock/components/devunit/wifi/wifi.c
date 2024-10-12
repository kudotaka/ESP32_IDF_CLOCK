#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
#include "wifi.h"
#endif

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define ESP_WIFI_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

static const char *TAG = "MY-WIFI";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_START_BIT      BIT2
#define WIFI_STOP_BIT       BIT3
#define WIFI_CONNECT_BIT    BIT4
#define WIFI_DISCONNECT_BIT BIT5
static int s_retry_num = 0;


esp_err_t Wifi_Disconnect()
{
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECT_BIT);
    xEventGroupSetBits(s_wifi_event_group, WIFI_DISCONNECT_BIT);
    ESP_ERROR_CHECK(esp_wifi_disconnect() );
    return ESP_OK;
}

esp_err_t Wifi_Connect()
{
    xEventGroupClearBits(s_wifi_event_group, WIFI_DISCONNECT_BIT);
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECT_BIT);
    ESP_ERROR_CHECK(esp_wifi_connect() );
    return ESP_OK;
}

esp_err_t Wifi_Stop()
{
    esp_err_t ret = ESP_OK;
    EventBits_t status = xEventGroupGetBits(s_wifi_event_group);
    if ( !(status & WIFI_DISCONNECT_BIT) )
    {
        ret = Wifi_Disconnect();
    }
    if (ret == ESP_OK)
    {
        xEventGroupClearBits(s_wifi_event_group, WIFI_START_BIT | WIFI_CONNECT_BIT | WIFI_DISCONNECT_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_STOP_BIT);
        ESP_ERROR_CHECK(esp_wifi_stop() );
    }
    return ret;
}

esp_err_t Wifi_Start()
{
    xEventGroupClearBits(s_wifi_event_group, WIFI_STOP_BIT | WIFI_CONNECT_BIT | WIFI_DISCONNECT_BIT);
    xEventGroupSetBits(s_wifi_event_group, WIFI_START_BIT);
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI(TAG, "wifi_init_sta finished.");
    // Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
    // number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    // xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
    // happened.
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", CONFIG_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", CONFIG_ESP_WIFI_SSID);
        return ESP_ERR_NOT_FINISHED;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t Wifi_IsConnected(void)
{
    EventBits_t status = xEventGroupGetBits(s_wifi_event_group);
    if ((status & WIFI_CONNECTED_BIT) && !(status & WIFI_FAIL_BIT)) {
        ESP_LOGD(TAG, "connected! %ld", status);
        return ESP_OK;
    } else {
        ESP_LOGD(TAG, "disconnected. %ld", status);
    }
    return ESP_ERR_INVALID_STATE;
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    EventBits_t status = xEventGroupGetBits(s_wifi_event_group);
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        Wifi_Connect();
        ESP_LOGI(TAG, "catch WIFI_EVENT_STA_START");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_STOP) {
        ESP_LOGI(TAG, "catch WIFI_EVENT_STA_STOP");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if ( (status & WIFI_DISCONNECT_BIT) ) {
            ESP_LOGI(TAG, "WIFI_DISCONNECT_BIT");
        }
        else
        {
            if (s_retry_num < CONFIG_ESP_WIFI_MAXIMUM_RETRY) {
                Wifi_Connect();
                s_retry_num++;
                ESP_LOGI(TAG, "retry to connect to the AP");
            } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            ESP_LOGI(TAG,"connect to the AP fail");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            // Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
            // If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
            // to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
            // WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = ESP_WIFI_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
}

esp_err_t Wifi_DeInit()
{
    return esp_wifi_deinit();
}

esp_err_t Wifi_Init()
{
    ESP_LOGI(TAG, "Wifi_Init() start.");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    return ret;
}