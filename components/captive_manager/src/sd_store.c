#include "sd_store.h"

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "sd_store";
static const char *MOUNT_POINT = "/sdcard";
static const char *CSV_PATH = "/sdcard/measurements.csv";
static const char *CSV_HEADER = "id,timestamp,co2,pm1p0,pm2p5,pm4p0,pm10p0,voc,nox,temp,hum,window_s\n";

static bool g_ready = false;
static uint32_t g_last_id = 0;
static SemaphoreHandle_t g_lock = NULL;

static bool file_exists(const char *path) {
    struct stat st = {0};
    return stat(path, &st) == 0;
}

static void ensure_header(void) {
    if (file_exists(CSV_PATH)) {
        return;
    }
    FILE *f = fopen(CSV_PATH, "w");
    if (!f) {
        ESP_LOGW(TAG, "No se pudo crear CSV en SD");
        return;
    }
    fputs(CSV_HEADER, f);
    fclose(f);
}

static uint32_t scan_last_id(void) {
    FILE *f = fopen(CSV_PATH, "r");
    if (!f) {
        return 0;
    }

    char line[256];
    uint32_t last = 0;
    while (fgets(line, sizeof(line), f)) {
        char *end = NULL;
        unsigned long id = strtoul(line, &end, 10);
        if (end && *end == ',' && id > last) {
            last = (uint32_t)id;
        }
    }
    fclose(f);
    return last;
}

esp_err_t sd_store_init(void) {
    if (g_ready) {
        return ESP_OK;
    }
    if (!g_lock) {
        g_lock = xSemaphoreCreateMutex();
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_card_t *card = NULL;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_STORE_PIN_MOSI,
        .miso_io_num = SD_STORE_PIN_MISO,
        .sclk_io_num = SD_STORE_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "No se pudo iniciar bus SPI SD: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_STORE_PIN_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo montar SD: %s", esp_err_to_name(ret));
        return ret;
    }

    ensure_header();
    g_last_id = scan_last_id();
    g_ready = true;
    ESP_LOGI(TAG, "SD lista en %s, ultimo id=%lu", MOUNT_POINT, (unsigned long)g_last_id);
    return ESP_OK;
}

bool sd_store_is_ready(void) {
    return g_ready;
}

uint32_t sd_store_last_id(void) {
    return g_last_id;
}

esp_err_t sd_store_append_reading(const captive_manager_readings_t *reading, uint32_t *out_id) {
    if (!g_ready || !reading || !reading->valid) {
        return ESP_ERR_INVALID_STATE;
    }

    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    uint32_t id = ++g_last_id;
    FILE *f = fopen(CSV_PATH, "a");
    if (!f) {
        g_last_id--;
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    fprintf(f,
            "%lu,%s,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu\n",
            (unsigned long)id,
            reading->timestamp,
            reading->co2,
            reading->pm1p0,
            reading->pm2p5,
            reading->pm4p0,
            reading->pm10p0,
            reading->voc,
            reading->nox,
            reading->temp,
            reading->hum,
            (unsigned long)reading->window_s);
    fclose(f);
    if (g_lock) xSemaphoreGive(g_lock);

    if (out_id) {
        *out_id = id;
    }
    return ESP_OK;
}

static bool parse_csv_line(const char *line, uint32_t *id, char *timestamp, size_t timestamp_size,
                           uint32_t *co2, float *pm1p0, float *pm2p5, float *pm4p0, float *pm10p0,
                           float *voc, float *nox, float *temp, float *hum, uint32_t *window_s) {
    if (!line || !id || !timestamp || timestamp_size == 0) {
        return false;
    }

    char ts[32] = {0};
    unsigned long parsed_id = 0;
    unsigned long parsed_co2 = 0;
    unsigned long parsed_window_s = 0;
    int matched = sscanf(line,
                         "%lu,%31[^,],%lu,%f,%f,%f,%f,%f,%f,%f,%f,%lu",
                         &parsed_id,
                         ts,
                         &parsed_co2,
                         pm1p0,
                         pm2p5,
                         pm4p0,
                         pm10p0,
                         voc,
                         nox,
                         temp,
                         hum,
                         &parsed_window_s);
    if (matched != 12) {
        return false;
    }
    *id = (uint32_t)parsed_id;
    *co2 = (uint32_t)parsed_co2;
    *window_s = (uint32_t)parsed_window_s;
    snprintf(timestamp, timestamp_size, "%s", ts);
    return true;
}

esp_err_t sd_store_add_readings_since(cJSON *array, uint32_t after_id, uint32_t limit, uint32_t *added) {
    if (!g_ready || !array) {
        return ESP_ERR_INVALID_STATE;
    }
    if (limit == 0 || limit > 1000) {
        limit = 500;
    }

    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    FILE *f = fopen(CSV_PATH, "r");
    if (!f) {
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    char line[256];
    uint32_t count = 0;
    while (fgets(line, sizeof(line), f) && count < limit) {
        uint32_t id = 0;
        uint32_t co2 = 0;
        uint32_t window_s = 0;
        float pm1p0 = 0, pm2p5 = 0, pm4p0 = 0, pm10p0 = 0, voc = 0, nox = 0, temp = 0, hum = 0;
        char timestamp[32] = {0};
        if (!parse_csv_line(line, &id, timestamp, sizeof(timestamp), &co2, &pm1p0, &pm2p5, &pm4p0, &pm10p0, &voc, &nox, &temp, &hum, &window_s)) {
            continue;
        }
        if (id <= after_id) {
            continue;
        }

        cJSON *row = cJSON_CreateObject();
        cJSON_AddNumberToObject(row, "id", id);
        cJSON_AddNumberToObject(row, "measurement_id", id);
        cJSON_AddStringToObject(row, "timestamp", timestamp);
        cJSON_AddNumberToObject(row, "co2", co2);
        cJSON_AddNumberToObject(row, "pm1p0", pm1p0);
        cJSON_AddNumberToObject(row, "pm2p5", pm2p5);
        cJSON_AddNumberToObject(row, "pm4p0", pm4p0);
        cJSON_AddNumberToObject(row, "pm10p0", pm10p0);
        cJSON_AddNumberToObject(row, "voc", voc);
        cJSON_AddNumberToObject(row, "nox", nox);
        cJSON_AddNumberToObject(row, "temp", temp);
        cJSON_AddNumberToObject(row, "hum", hum);
        cJSON_AddNumberToObject(row, "window_s", window_s);
        cJSON_AddItemToArray(array, row);
        count++;
    }
    fclose(f);
    if (g_lock) xSemaphoreGive(g_lock);

    if (added) {
        *added = count;
    }
    return ESP_OK;
}
