#include <stdio.h>
#include <time.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "captive_manager.h"
#include "sd_store.h"
#include "sensors.h"

static const char *TAG = "EcoSensor";

static const char *MDNS_HOSTNAME = "ecosensor01";
static const char *AP_SSID = "EcoSensor-01";
static const char *AP_PASS = "LCT3180940";

#define LOG_EACH_SAMPLE          1
#define SENSOR_TASK_STACK        8192
#define SENSOR_START_TASK_STACK  4096
#define SAMPLE_DELAY_MS          5000
#define SAMPLES_PER_AVG_WINDOW   12

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

static TaskHandle_t s_sensor_task_handle = NULL;
static bool s_sensors_started = false;

static void publish_latest_average(const SensorData *avg) {
    if (!avg) {
        return;
    }

    captive_manager_readings_t snapshot = {0};
    snapshot.valid = true;
    snapshot.window_s = (SAMPLE_DELAY_MS * SAMPLES_PER_AVG_WINDOW) / 1000;
    snapshot.co2 = avg->co2;
    snapshot.pm1p0 = avg->pm1p0;
    snapshot.pm2p5 = avg->pm2p5;
    snapshot.pm4p0 = avg->pm4p0;
    snapshot.pm10p0 = avg->pm10p0;
    snapshot.voc = avg->voc;
    snapshot.nox = avg->nox;
    snapshot.temp = avg->avg_temp;
    snapshot.hum = avg->avg_hum;

    if (captive_manager_time_is_valid()) {
        time_t now = time(NULL);
        struct tm utc_tm = {0};
        gmtime_r(&now, &utc_tm);
        strftime(snapshot.timestamp, sizeof(snapshot.timestamp), "%Y-%m-%dT%H:%M:%SZ", &utc_tm);
    }

    uint32_t measurement_id = 0;
    if (sd_store_append_reading(&snapshot, &measurement_id) == ESP_OK) {
        snapshot.id = measurement_id;
        snapshot.measurement_id = measurement_id;
    }

    captive_manager_set_last_readings(&snapshot);
}

static void sensor_task(void *pv) {
    const TickType_t sample_delay_ticks = pdMS_TO_TICKS(SAMPLE_DELAY_MS);

    int sample_slot = 0;
    uint32_t sum_co2 = 0;
    int scd40_ok_count = 0;
    int sen55_ok_count = 0;

    double sum_pm1p0 = 0;
    double sum_pm2p5 = 0;
    double sum_pm4p0 = 0;
    double sum_pm10p0 = 0;
    double sum_voc = 0;
    double sum_nox = 0;
    double sum_scd_temp = 0;
    double sum_scd_hum = 0;
    double sum_sen_temp = 0;
    double sum_sen_hum = 0;
    double sum_avg_temp = 0;
    double sum_avg_hum = 0;

    while (1) {
        SensorData data = {0};

        esp_err_t scd_ret = sensors_read_scd40(&data);
        int scd_diag = sensors_get_last_scd40_diag();
        if (scd_ret == ESP_OK) {
            sum_co2 += data.co2;
            sum_scd_temp += data.scd_temp;
            sum_scd_hum += data.scd_hum;
            scd40_ok_count++;
        }

        esp_err_t sen_ret = sensors_read_sen55(&data);
        int sen_diag = sensors_get_last_sen55_diag();
        if (sen_ret == ESP_OK) {
            sum_pm1p0 += data.pm1p0;
            sum_pm2p5 += data.pm2p5;
            sum_pm4p0 += data.pm4p0;
            sum_pm10p0 += data.pm10p0;
            sum_voc += data.voc;
            sum_nox += data.nox;
            sum_sen_temp += data.sen_temp;
            sum_sen_hum += data.sen_hum;
            sum_avg_temp += data.avg_temp;
            sum_avg_hum += data.avg_hum;
            sen55_ok_count++;
        }

#if LOG_EACH_SAMPLE
        ESP_LOGI(TAG,
                 "Muestra %d/%d | SCD40 co2=%u diag=%02d ret=%s | SEN55 pm2.5=%.2f temp=%.2f hum=%.2f diag=%02d ret=%s",
                 sample_slot + 1,
                 SAMPLES_PER_AVG_WINDOW,
                 data.co2,
                 scd_diag,
                 esp_err_to_name(scd_ret),
                 data.pm2p5,
                 data.avg_temp,
                 data.avg_hum,
                 sen_diag,
                 esp_err_to_name(sen_ret));
#endif

        sample_slot++;

        if (sample_slot >= SAMPLES_PER_AVG_WINDOW) {
            SensorData window_avg = {0};

            if (scd40_ok_count > 0) {
                double denom = (double)scd40_ok_count;
                window_avg.co2 = (uint16_t)(sum_co2 / scd40_ok_count);
                window_avg.scd_temp = (float)(sum_scd_temp / denom);
                window_avg.scd_hum = (float)(sum_scd_hum / denom);
            }

            if (sen55_ok_count > 0) {
                double denom = (double)sen55_ok_count;
                window_avg.pm1p0 = (float)(sum_pm1p0 / denom);
                window_avg.pm2p5 = (float)(sum_pm2p5 / denom);
                window_avg.pm4p0 = (float)(sum_pm4p0 / denom);
                window_avg.pm10p0 = (float)(sum_pm10p0 / denom);
                window_avg.voc = (float)(sum_voc / denom);
                window_avg.nox = (float)(sum_nox / denom);
                window_avg.sen_temp = (float)(sum_sen_temp / denom);
                window_avg.sen_hum = (float)(sum_sen_hum / denom);
                window_avg.avg_temp = (float)(sum_avg_temp / denom);
                window_avg.avg_hum = (float)(sum_avg_hum / denom);
            }

            char json[320];
            sensors_format_json(&window_avg, json, sizeof(json));
            publish_latest_average(&window_avg);
            ESP_LOGI(TAG, "Promedio 1 min: %s", json);

            sample_slot = 0;
            sum_co2 = 0;
            scd40_ok_count = 0;
            sen55_ok_count = 0;
            sum_pm1p0 = 0;
            sum_pm2p5 = 0;
            sum_pm4p0 = 0;
            sum_pm10p0 = 0;
            sum_voc = 0;
            sum_nox = 0;
            sum_scd_temp = 0;
            sum_scd_hum = 0;
            sum_sen_temp = 0;
            sum_sen_hum = 0;
            sum_avg_temp = 0;
            sum_avg_hum = 0;
        }

        vTaskDelay(sample_delay_ticks);
    }
}

static void sensor_start_task(void *pv) {
    while (1) {
        captive_state_t st = captive_manager_get_state();
        if (st == CAP_STATE_OPERATIONAL && !captive_manager_time_is_valid()) {
            ESP_LOGI(TAG, "WiFi operativo; esperando configuracion de fecha/hora via POST /config");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (st == CAP_STATE_OPERATIONAL) {
            ESP_LOGI(TAG, "WiFi, fecha y hora configurados; iniciando sensores");

            esp_err_t sret = sensors_init_all();
            if (sret != ESP_OK) {
                ESP_LOGE(TAG, "Fallo al inicializar sensores: %s", esp_err_to_name(sret));
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }

            xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK, NULL, 5, &s_sensor_task_handle);
            s_sensors_started = true;
            captive_manager_set_sensors_started(true);
            ESP_LOGI(TAG, "Sensores y promedio iniciados despues de la configuracion WiFi");
            vTaskDelete(NULL);
            return;
        }

        ESP_LOGI(TAG, "Esperando WiFi y fecha/hora para iniciar sensores... estado=%s time_valid=%s",
                 captive_manager_state_str(st),
                 captive_manager_time_is_valid() ? "true" : "false");
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    captive_manager_set_sensors_started(false);
    ESP_ERROR_CHECK(captive_manager_start());

    esp_err_t sd_ret = sd_store_init();
    if (sd_ret != ESP_OK) {
        ESP_LOGW(TAG, "SD no disponible; lecturas solo en memoria hasta resolver SD: %s", esp_err_to_name(sd_ret));
    }

    ESP_LOGI(TAG, "WiFi manager iniciado");
    ESP_LOGI(TAG, "mDNS: %s.local", MDNS_HOSTNAME);
    ESP_LOGI(TAG, "AP temporal: SSID=%s", AP_SSID);

    xTaskCreate(sensor_start_task,
                "sensor_start_task",
                SENSOR_START_TASK_STACK,
                NULL,
                5,
                NULL);

    while (1) {
        ESP_LOGI(TAG, "Estado captive_manager: %s | time_valid=%s | sensores=%s",
                 captive_manager_state_str(captive_manager_get_state()),
                 captive_manager_time_is_valid() ? "true" : "false",
                 s_sensors_started ? "iniciados" : "esperando");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
