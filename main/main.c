#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "captive_manager.h"
#include "gps.h"
#include "sd_store.h"
#include "sensors.h"

static const char *TAG = "EcoSensor";

static const char *MDNS_HOSTNAME = "ecosensor03";
static const char *AP_SSID = "EcoSensor03";
static const char *AP_PASS = "LCTECO03";

// Offset EcoSensor01: SCD40 = 7.70 SEN55 = -3.02
// Offset EcoSensor02: SCD40 = 7.70 SEN55 = -3.02
// Offset EcoSensor03: SCD40 = 13.07 SEN55 = -3.02
const float ECO_SCD40_TEMP_OFFSET_C = 13.07f;
const float ECO_SEN55_TEMP_OFFSET_C = -3.02f;

#define LOG_EACH_SAMPLE          0
#define SENSOR_TASK_STACK        8192
#define SENSOR_START_TASK_STACK  4096
#define SAMPLE_DELAY_MS          30000
#define SAMPLES_PER_AVG_WINDOW   10
#define SEN51_VOC_NOX_WARMUP_MS  120000
#define BOARD_POWERON_PIN        GPIO_NUM_12

#define MEASUREMENT_PUSH_ENDPOINT      "http://ecosensor.local:8765/api/measurements/push"
#define MEASUREMENT_PUSH_PORT          8765
#define MEASUREMENT_PUSH_PATH          "/api/measurements/push"
#define MEASUREMENT_PUSH_TIMEOUT_MS    3000
#define MEASUREMENT_PUSH_TASK_STACK    7168
#define MEASUREMENT_PUSH_QUEUE_LENGTH   6


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
static TaskHandle_t s_push_task_handle = NULL;
static QueueHandle_t s_measurement_push_queue = NULL;
static bool s_sensors_started = false;

static void gps_time_available(time_t epoch_utc) {
    esp_err_t err = captive_manager_offer_time_epoch(epoch_utc, "gps");
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Hora GPS valida no aceptada: %s", esp_err_to_name(err));
    }
}

static esp_err_t post_measurement_payload(const captive_manager_readings_t *reading) {
    if (!reading || !captive_manager_can_push_measurements()) {
        return ESP_ERR_INVALID_STATE;
    }

    char payload[1024];
    int len = snprintf(payload,
                       sizeof(payload),
                       "{\"device_id\":\"%s\",\"measurement_id\":%lu,"
                       "\"boot_id\":%lu,\"uptime_s\":%lu,\"time_valid\":%s,"
                       "\"time_source\":\"%s\",\"timestamp\":\"%s\","
                       "\"pm1p0\":%.2f,\"pm2p5\":%.2f,\"pm4p0\":%.2f,\"pm10p0\":%.2f,"
                       "\"voc\":%.2f,\"nox\":%.2f,\"co2\":%u,"
                       "\"temp\":%.2f,\"hum\":%.2f,"
                       "\"scd_temp\":%.2f,\"scd_hum\":%.2f,"
                       "\"sen_temp\":%.2f,\"sen_hum\":%.2f,"
                       "\"gps_valid\":%s,\"gps_lat\":%.6f,\"gps_lon\":%.6f,"
                       "\"gps_satellites\":%u,\"gps_hdop\":%.2f,\"gps_age_ms\":%lu,"
                       "\"window_s\":%lu}",
                       MDNS_HOSTNAME,
                       (unsigned long)reading->measurement_id,
                       (unsigned long)reading->boot_id,
                       (unsigned long)reading->uptime_s,
                       reading->time_valid ? "true" : "false",
                       reading->time_source[0] ? reading->time_source : "none",
                       reading->time_valid ? reading->timestamp : "",
                       reading->pm1p0,
                       reading->pm2p5,
                       reading->pm4p0,
                       reading->pm10p0,
                       reading->voc,
                       reading->nox,
                       reading->co2,
                       reading->temp,
                       reading->hum,
                       reading->scd_temp,
                       reading->scd_hum,
                       reading->sen_temp,
                       reading->sen_hum,
                       reading->gps_valid ? "true" : "false",
                       reading->gps_valid ? reading->gps_lat : 0.0,
                       reading->gps_valid ? reading->gps_lon : 0.0,
                       reading->gps_satellites,
                       reading->gps_hdop,
                       (unsigned long)reading->gps_age_ms,
                       (unsigned long)reading->window_s);
    if (len <= 0 || len >= (int)sizeof(payload)) {
        ESP_LOGW(TAG, "Push medicion: payload demasiado grande");
        return ESP_ERR_NO_MEM;
    }

    char dynamic_push_url[160];
    const char *push_host = captive_manager_push_host();
    const char *push_url = MEASUREMENT_PUSH_ENDPOINT;
    if (push_host && push_host[0]) {
        snprintf(dynamic_push_url,
                 sizeof(dynamic_push_url),
                 "http://%s:%d%s",
                 push_host,
                 MEASUREMENT_PUSH_PORT,
                 MEASUREMENT_PUSH_PATH);
        push_url = dynamic_push_url;
    }

    esp_http_client_config_t config = {
        .url = push_url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = MEASUREMENT_PUSH_TIMEOUT_MS,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, payload, len);
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        return err;
    }
    if (status < 200 || status >= 300) {
        ESP_LOGW(TAG, "Push medicion: HTTP status=%d", status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void measurement_push_task(void *pv) {
    (void)pv;

    captive_manager_readings_t reading = {0};
    const uint32_t retry_delay_ms[] = {0, 3000, 10000};
    const size_t attempts = sizeof(retry_delay_ms) / sizeof(retry_delay_ms[0]);

    while (1) {
        if (xQueueReceive(s_measurement_push_queue, &reading, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        bool sent = false;
        for (size_t i = 0; i < attempts; i++) {
            if (retry_delay_ms[i] > 0) {
                vTaskDelay(pdMS_TO_TICKS(retry_delay_ms[i]));
            }
            esp_err_t err = post_measurement_payload(&reading);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Medicion enviada al servidor id=%lu", (unsigned long)reading.measurement_id);
                sent = true;
                break;
            }
            ESP_LOGW(TAG,
                     "Push medicion intento %u/%u fallo: %s",
                     (unsigned)(i + 1),
                     (unsigned)attempts,
                     esp_err_to_name(err));
        }

        if (!sent) {
            ESP_LOGW(TAG,
                     "Medicion id=%lu no enviada tras %u intentos; queda recuperable por sincronizacion SD",
                     (unsigned long)reading.measurement_id,
                     (unsigned)attempts);
        }
    }
}

static void schedule_measurement_push(const captive_manager_readings_t *reading) {
    if (!reading) {
        return;
    }
    if (!s_measurement_push_queue) {
        ESP_LOGW(TAG, "Push medicion: cola no inicializada");
        return;
    }
    if (!captive_manager_can_push_measurements()) {
        return;
    }

    if (xQueueSend(s_measurement_push_queue, reading, 0) != pdTRUE) {
        ESP_LOGW(TAG,
                 "Push medicion: cola llena; id=%lu queda recuperable por sincronizacion SD",
                 (unsigned long)reading->measurement_id);
    }
}

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
    snapshot.scd_temp = avg->scd_temp;
    snapshot.scd_hum = avg->scd_hum;
    snapshot.sen_temp = avg->sen_temp;
    snapshot.sen_hum = avg->sen_hum;

    gps_fix_t gps_fix = {0};
    esp_err_t gps_ret = gps_get_fix(&gps_fix, 10 * 60 * 1000, 90 * 1000);
    if (gps_ret == ESP_OK && gps_fix.valid) {
        snapshot.gps_valid = true;
        snapshot.gps_lat = gps_fix.lat;
        snapshot.gps_lon = gps_fix.lon;
        snapshot.gps_satellites = gps_fix.satellites;
        snapshot.gps_hdop = gps_fix.hdop;
        snapshot.gps_age_ms = gps_fix.age_ms;
    } else {
        ESP_LOGE(TAG, "No se obtuvo fix GPS fresco para la medicion: %s", esp_err_to_name(gps_ret));
        return;
    }

    snapshot.time_valid = captive_manager_time_is_valid();
    if (snapshot.time_valid) {
        snprintf(snapshot.time_source, sizeof(snapshot.time_source), "%s", captive_manager_time_source());
        time_t now = time(NULL);
        struct tm utc_tm = {0};
        gmtime_r(&now, &utc_tm);
        strftime(snapshot.timestamp, sizeof(snapshot.timestamp), "%Y-%m-%dT%H:%M:%SZ", &utc_tm);
    } else {
        snprintf(snapshot.time_source, sizeof(snapshot.time_source), "uptime");
    }

    uint32_t measurement_id = 0;
    esp_err_t sd_ret = sd_store_append_reading(&snapshot, &measurement_id);
    if (sd_ret == ESP_OK) {
        snapshot.id = measurement_id;
        snapshot.measurement_id = measurement_id;
        ESP_LOGI(TAG, "Medicion publicada y guardada en SD con id=%lu", (unsigned long)measurement_id);
    } else {
        ESP_LOGW(TAG, "Medicion publicada SIN guardar en SD: %s", esp_err_to_name(sd_ret));
    }

    captive_manager_set_last_readings(&snapshot);
    schedule_measurement_push(&snapshot);
}

static void sensor_task(void *pv) {
    const TickType_t sample_delay_ticks = pdMS_TO_TICKS(SAMPLE_DELAY_MS);
    const TickType_t voc_nox_warmup_ticks = pdMS_TO_TICKS(SEN51_VOC_NOX_WARMUP_MS);
    const TickType_t sensor_start_tick = xTaskGetTickCount();

    int sample_slot = 0;
    uint32_t sum_co2 = 0;
    int scd40_ok_count = 0;
    int scd40_temp_hum_count = 0;
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
            scd40_ok_count++;
        }
        if ((scd_ret == ESP_OK || scd_diag == SENSOR_DIAG_CO2_ZERO || scd_diag == SENSOR_DIAG_CO2_TOO_HIGH) &&
            (data.scd_temp != 0.0f || data.scd_hum != 0.0f)) {
            sum_scd_temp += data.scd_temp;
            sum_scd_hum += data.scd_hum;
            scd40_temp_hum_count++;
        }
        if (scd_ret != ESP_OK && scd_diag != SENSOR_DIAG_NOT_READY) {
            scd40_error_count++;
        }

        esp_err_t sen_ret = sensors_read_sen55(&data);
#if LOG_EACH_SAMPLE
        int sen_diag = sensors_get_last_sen55_diag();
#endif
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
                window_avg.co2 = (uint16_t)(sum_co2 / scd40_ok_count);
            }
            if (scd40_temp_hum_count > 0) {
                double denom = (double)scd40_temp_hum_count;
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
            scd40_temp_hum_count = 0;
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
        if (captive_manager_can_measure()) {
            if (!gps_has_valid_fix()) {
                ESP_LOGI(TAG,
                         "%s; esperando primer fix GPS valido antes de iniciar sensores (time_valid=%s, push=%s)",
                         captive_manager_can_push_measurements() ? "WiFi operativo" : "Modo offline AP+STA",
                         captive_manager_time_is_valid() ? "true" : "false",
                         captive_manager_can_push_measurements() ? "habilitado" : "deshabilitado");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }

            ESP_LOGI(TAG,
                     "%s; GPS valido, iniciando sensores (time_valid=%s, push=%s)",
                     captive_manager_can_push_measurements() ? "WiFi operativo" : "Modo offline AP+STA",
                     captive_manager_time_is_valid() ? "true" : "false",
                     captive_manager_can_push_measurements() ? "habilitado" : "deshabilitado");

            esp_err_t sret = sensors_init_all();
            if (sret != ESP_OK) {
                ESP_LOGE(TAG, "Fallo al inicializar sensores: %s", esp_err_to_name(sret));
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }

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
    esp_err_t gps_ret = gps_init();
    if (gps_ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar GPS; sensores quedaran esperando fix: %s", esp_err_to_name(gps_ret));
    }

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
    gps_set_time_callback(gps_time_available);
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_event_handler, NULL));
    captive_manager_set_sensors_started(false);
    ESP_ERROR_CHECK(captive_manager_start());

    s_measurement_push_queue = xQueueCreate(MEASUREMENT_PUSH_QUEUE_LENGTH, sizeof(captive_manager_readings_t));
    if (!s_measurement_push_queue) {
        ESP_LOGE(TAG, "No se pudo crear cola de push de mediciones");
    } else {
        BaseType_t push_ok = xTaskCreate(measurement_push_task,
                                         "measurement_push",
                                         MEASUREMENT_PUSH_TASK_STACK,
                                         NULL,
                                         tskIDLE_PRIORITY + 1,
                                         &s_push_task_handle);
        if (push_ok != pdPASS) {
            ESP_LOGE(TAG, "No se pudo iniciar tarea persistente de push");
            vQueueDelete(s_measurement_push_queue);
            s_measurement_push_queue = NULL;
        }
    }

    esp_err_t sd_ret = sd_store_init();
    if (sd_ret != ESP_OK) {
        ESP_LOGW(TAG, "SD no disponible; lecturas solo en memoria hasta resolver SD: %s", esp_err_to_name(sd_ret));
    } else {
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
