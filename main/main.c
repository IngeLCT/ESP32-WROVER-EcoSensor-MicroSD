#include <stdio.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "captive_manager.h"

static const char *TAG = "EcoSensor";

// Configuración editable desde aquí
static const char *MDNS_HOSTNAME = "ecosensor01";
static const char *AP_SSID = "EcoSensor-01";
static const char *AP_PASS = "LCT3180940";

static void wifi_event_handler(void *arg,
                               esp_event_base_t base,
                               int32_t id,
                               void *data) {
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        captive_manager_notify_sta_got_ip();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)data;
        captive_manager_notify_sta_disconnected(disc ? disc->reason : -1);
    }
}

void app_main(void)
{
    captive_manager_cfg_t cfg = {
        .ap_ssid = AP_SSID,
        .ap_pass = AP_PASS,
        .max_scan_aps = CONFIG_CAPTIVE_MANAGER_MAX_SCAN_APS,
        .conn_max_attempts = CONFIG_CAPTIVE_MANAGER_CONN_MAX_ATTEMPTS,
        .conn_retry_delay_ms = CONFIG_CAPTIVE_MANAGER_CONN_RETRY_DELAY_MS,
        .boot_grace_ms = CONFIG_CAPTIVE_MANAGER_BOOT_GRACE_MS,
        .mdns_hostname = MDNS_HOSTNAME,
    };

    ESP_ERROR_CHECK(captive_manager_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(captive_manager_start());

    ESP_LOGI(TAG, "WiFi manager iniciado");
    ESP_LOGI(TAG, "mDNS: %s.local", MDNS_HOSTNAME);
    ESP_LOGI(TAG, "AP temporal: SSID=%s", AP_SSID);

    while (1) {
        ESP_LOGI(TAG, "Estado captive_manager: %s",
                 captive_manager_state_str(captive_manager_get_state()));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
