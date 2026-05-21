#include <stdio.h>
#include <time.h>

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "captive_manager.h"
#include "sd_store.h"
#include "sensors.h"

static const char *TAG = "EcoSensor";

static const char *MDNS_HOSTNAME = "ecosensor02";
static const char *AP_SSID = "EcoSensor-02";
static const char *AP_PASS = "LCT3180940";

#define LOG_EACH_SAMPLE          1
#define SENSOR_TASK_STACK        8192
#define SENSOR_START_TASK_STACK  4096
#define SAMPLE_DELAY_MS          5000
#define SAMPLES_PER_AVG_WINDOW   60
#define SEN51_VOC_NOX_WARMUP_MS  120000
#define BOARD_POWERON_PIN        GPIO_NUM_12

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

static void enable_board_peripherals_power(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BOARD_POWERON_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_POWERON_PIN, 1));
    ESP_LOGI(TAG, "Alimentacion de perifericos habilitada en GPIO%d para SD/modem", BOARD_POWERON_PIN);
    vTaskDelay(pdMS_TO_TICKS(150));
}

static void publish_latest_average(const SensorData *avg) {
    if (!avg) {
        return;
    }

    captive_manager_readings_t snapshot = {0};
    snapshot.valid = true;
    snapshot.window_s = (SAMPLE_DELAY_MS * SAMPLES_PER_AVG_WINDOW) / 1000;
    snapshot.boot_id = captive_manager_boot_id();
    snapshot.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    snapshot.co2 = avg->co2;
    snapshot.pm1p0 = avg->pm1p0;
    snapshot.pm2p5 = avg->pm2p5;
    snapshot.pm4p0 = avg->pm4p0;
    snapshot.pm10p0 = avg->pm10p0;
    snapshot.voc = avg->voc;
    snapshot.nox = avg->nox;
    snapshot.temp = avg->avg_temp;
    snapshot.hum = avg->avg_hum;

    snapshot.time_valid = captive_manager_time_is_valid();
    if (snapshot.time_valid) {
        time_t now = time(NULL);
        struct tm utc_tm = {0};
        gmtime_r(&now, &utc_tm);
        strftime(snapshot.timestamp, sizeof(snapshot.timestamp), "%Y-%m-%dT%H:%M:%SZ", &utc_tm);
    }

    uint32_t measurement_id = 0;
    esp_err_t sd_ret = sd_store_append_reading(&snapshot, &measurement_id);
    if (sd_ret == ESP_OK) {
        snapshot.id = measurement_id;
        snapshot.measurement_id = measurement_id;
        char detail[96];
        snprintf(detail, sizeof(detail), "id=%lu time_valid=%s", (unsigned long)measurement_id, snapshot.time_valid ? "true" : "false");
        captive_manager_record_debug_event("measurement_saved", detail);
        ESP_LOGI(TAG, "Medicion publicada y guardada en SD con id=%lu", (unsigned long)measurement_id);
    } else {
        char detail[96];
        snprintf(detail, sizeof(detail), "sd_ret=%s", esp_err_to_name(sd_ret));
        captive_manager_record_debug_event("measurement_not_saved", detail);
        ESP_LOGW(TAG, "Medicion publicada SIN guardar en SD: %s", esp_err_to_name(sd_ret));
    }

    captive_manager_set_last_readings(&snapshot);
}

static void sensor_task(void *pv) {
    const TickType_t sample_delay_ticks = pdMS_TO_TICKS(SAMPLE_DELAY_MS);
    const TickType_t voc_nox_warmup_ticks = pdMS_TO_TICKS(SEN51_VOC_NOX_WARMUP_MS);
    const TickType_t sensor_start_tick = xTaskGetTickCount();

    int sample_slot = 0;
    uint32_t sum_co2 = 0;
    int scd40_ok_count = 0;
    int scd40_error_count = 0;
    int sen55_ok_count = 0;
    int voc_nox_ok_count = 0;
    bool voc_nox_warmup_logged = false;

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
        } else {
            scd40_error_count++;
        }

        esp_err_t sen_ret = sensors_read_sen55(&data);
        int sen_diag = sensors_get_last_sen55_diag();
        bool voc_nox_ready_now = (xTaskGetTickCount() - sensor_start_tick) >= voc_nox_warmup_ticks;
        if (sen_ret == ESP_OK) {
            sum_pm1p0 += data.pm1p0;
            sum_pm2p5 += data.pm2p5;
            sum_pm4p0 += data.pm4p0;
            sum_pm10p0 += data.pm10p0;

            if (voc_nox_ready_now) {
                sum_voc += data.voc;
                sum_nox += data.nox;
                voc_nox_ok_count++;
            } else if (!voc_nox_warmup_logged) {
                ESP_LOGI(TAG,
                         "SEN51 VOC/NOx en estabilizacion; se ignoraran los primeros %d s tras iniciar sensores",
                         SEN51_VOC_NOX_WARMUP_MS / 1000);
                voc_nox_warmup_logged = true;
            }

            sum_sen_temp += data.sen_temp;
            sum_sen_hum += data.sen_hum;
            sum_avg_temp += data.avg_temp;
            sum_avg_hum += data.avg_hum;
            sen55_ok_count++;
        }

        captive_manager_sensor_debug_t debug = {
            .sample_slot = (uint32_t)(sample_slot + 1),
            .samples_per_window = SAMPLES_PER_AVG_WINDOW,
            .scd40_ok_count = (uint32_t)scd40_ok_count,
            .scd40_error_count = (uint32_t)scd40_error_count,
            .sen55_ok_count = (uint32_t)sen55_ok_count,
            .voc_nox_ok_count = (uint32_t)voc_nox_ok_count,
            .voc_nox_ready = voc_nox_ready_now,
            .scd40_ret = (int)scd_ret,
            .sen55_ret = (int)sen_ret,
            .scd40_diag = scd_diag,
            .sen55_diag = sen_diag,
            .scd40_raw_co2 = sensors_get_last_scd40_raw_co2(),
            .scd40_raw_temp = sensors_get_last_scd40_raw_temp(),
            .scd40_raw_hum = sensors_get_last_scd40_raw_hum(),
            .scd40_last_temp = sensors_get_last_scd40_temp(),
            .scd40_last_hum = sensors_get_last_scd40_hum(),
            .last_sample_uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL),
        };
        snprintf(debug.scd40_raw_bytes, sizeof(debug.scd40_raw_bytes), "%s", sensors_get_last_scd40_raw_bytes());
        snprintf(debug.scd40_error, sizeof(debug.scd40_error), "%s", sensors_get_last_scd40_error());
        captive_manager_update_sensor_debug(&debug);

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
                if (voc_nox_ok_count > 0) {
                    double voc_nox_denom = (double)voc_nox_ok_count;
                    window_avg.voc = (float)(sum_voc / voc_nox_denom);
                    window_avg.nox = (float)(sum_nox / voc_nox_denom);
                }
                window_avg.sen_temp = (float)(sum_sen_temp / denom);
                window_avg.sen_hum = (float)(sum_sen_hum / denom);
                window_avg.avg_temp = (float)(sum_avg_temp / denom);
                window_avg.avg_hum = (float)(sum_avg_hum / denom);
            }

            char json[320];
            sensors_format_json(&window_avg, json, sizeof(json));
            publish_latest_average(&window_avg);
            ESP_LOGI(TAG,
                     "Promedio %d s: %s (VOC/NOx muestras validas=%d)",
                     (SAMPLE_DELAY_MS * SAMPLES_PER_AVG_WINDOW) / 1000,
                     json,
                     voc_nox_ok_count);

            sample_slot = 0;
            sum_co2 = 0;
            scd40_ok_count = 0;
            scd40_error_count = 0;
            sen55_ok_count = 0;
            voc_nox_ok_count = 0;
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
        if (st == CAP_STATE_OPERATIONAL) {
            ESP_LOGI(TAG,
                     "WiFi operativo; iniciando sensores (time_valid=%s)",
                     captive_manager_time_is_valid() ? "true" : "false");

            esp_err_t sret = sensors_init_all();
            if (sret != ESP_OK) {
                char detail[96];
                snprintf(detail, sizeof(detail), "sensors_init=%s", esp_err_to_name(sret));
                captive_manager_record_debug_event("sensors_init_failed", detail);
                ESP_LOGE(TAG, "Fallo al inicializar sensores: %s", esp_err_to_name(sret));
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }

            captive_manager_record_debug_event("sensors_init_ok", "starting sensor_task");
            xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK, NULL, 5, &s_sensor_task_handle);
            s_sensors_started = true;
            captive_manager_set_sensors_started(true);
            ESP_LOGI(TAG, "Sensores y promedio iniciados");
            vTaskDelete(NULL);
            return;
        }

        ESP_LOGI(TAG, "Esperando WiFi para iniciar sensores... estado=%s time_valid=%s",
                 captive_manager_state_str(st),
                 captive_manager_time_is_valid() ? "true" : "false");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;
    if (running && esp_ota_get_state_partition(running, &ota_state) == ESP_OK && ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG, "Imagen OTA en verificacion; marcando app valida tras arranque minimo");
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_mark_app_valid_cancel_rollback());
    }

    enable_board_peripherals_power();

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
    captive_manager_set_scd40_action_callback(sensors_scd40_debug_action);
    ESP_ERROR_CHECK(captive_manager_start());

    esp_err_t sd_ret = sd_store_init();
    if (sd_ret != ESP_OK) {
        char detail[96];
        snprintf(detail, sizeof(detail), "sd_init=%s", esp_err_to_name(sd_ret));
        captive_manager_record_debug_event("sd_init_failed", detail);
        ESP_LOGW(TAG, "SD no disponible; lecturas solo en memoria hasta resolver SD: %s", esp_err_to_name(sd_ret));
    } else {
        captive_manager_record_debug_event("sd_init_ok", "sd ready");
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
