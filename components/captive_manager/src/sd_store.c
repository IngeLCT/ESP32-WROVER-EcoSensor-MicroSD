#include "sd_store.h"

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "sd_store";
static const char *MOUNT_POINT = "/sdcard";
static const char *CSV_PATH = "/sdcard/data.csv";
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
        ESP_LOGI(TAG, "CSV existente encontrado: %s", CSV_PATH);
        return;
    }
    ESP_LOGI(TAG, "CSV no existe; creando archivo nuevo: %s", CSV_PATH);
    FILE *f = fopen(CSV_PATH, "w");
    if (!f) {
        ESP_LOGE(TAG, "No se pudo crear CSV en SD: %s errno=%d (%s)", CSV_PATH, errno, strerror(errno));
        return;
    }
    int written = fputs(CSV_HEADER, f);
    if (written == EOF) {
        ESP_LOGE(TAG, "Error escribiendo encabezado CSV: errno=%d (%s)", errno, strerror(errno));
    } else {
        ESP_LOGI(TAG, "Encabezado CSV creado correctamente");
    }
    if (fclose(f) != 0) {
        ESP_LOGE(TAG, "Error cerrando CSV tras encabezado: errno=%d (%s)", errno, strerror(errno));
    }
}

static uint32_t scan_last_id(void) {
    FILE *f = fopen(CSV_PATH, "r");
    if (!f) {
        ESP_LOGW(TAG, "No se pudo abrir CSV para escanear ultimo ID: %s errno=%d (%s)", CSV_PATH, errno, strerror(errno));
        return 0;
    }

    char line[256];
    uint32_t last = 0;
    uint32_t rows = 0;
    while (fgets(line, sizeof(line), f)) {
        char *end = NULL;
        unsigned long id = strtoul(line, &end, 10);
        if (end && *end == ',' && id > last) {
            last = (uint32_t)id;
            rows++;
        }
    }
    fclose(f);
    ESP_LOGI(TAG, "CSV escaneado: filas=%lu ultimo_id=%lu", (unsigned long)rows, (unsigned long)last);
    return last;
}

esp_err_t sd_store_init(void) {
    if (g_ready) {
        return ESP_OK;
    }
    if (!g_lock) {
        g_lock = xSemaphoreCreateMutex();
    }

    ESP_LOGI(TAG, "Inicializando SD SPI: SCK=%d MISO=%d MOSI=%d CS=%d mount=%s csv=%s",
             SD_STORE_PIN_SCK, SD_STORE_PIN_MISO, SD_STORE_PIN_MOSI, SD_STORE_PIN_CS,
             MOUNT_POINT, CSV_PATH);

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
        ESP_LOGE(TAG, "No se pudo iniciar bus SPI SD: %s", esp_err_to_name(ret));
        return ret;
    }
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Bus SPI SD ya estaba inicializado; continuo con el montaje");
    } else {
        ESP_LOGI(TAG, "Bus SPI SD inicializado correctamente");
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_STORE_PIN_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo montar SD: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SD montada correctamente");
    sdmmc_card_print_info(stdout, card);

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
    if (!g_ready) {
        ESP_LOGW(TAG, "No se guarda medicion: SD no esta lista");
        return ESP_ERR_INVALID_STATE;
    }
    if (!reading) {
        ESP_LOGW(TAG, "No se guarda medicion: reading=NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!reading->valid) {
        ESP_LOGW(TAG, "No se guarda medicion: reading->valid=false");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    uint32_t id = ++g_last_id;
    ESP_LOGI(TAG,
             "Guardando medicion SD id=%lu timestamp=%s co2=%u pm2.5=%.2f voc=%.2f nox=%.2f temp=%.2f hum=%.2f window_s=%lu",
             (unsigned long)id,
             reading->timestamp[0] ? reading->timestamp : "SIN_HORA",
             reading->co2,
             reading->pm2p5,
             reading->voc,
             reading->nox,
             reading->temp,
             reading->hum,
             (unsigned long)reading->window_s);

    FILE *f = fopen(CSV_PATH, "a");
    if (!f) {
        ESP_LOGE(TAG, "No se pudo abrir CSV para append: %s errno=%d (%s)", CSV_PATH, errno, strerror(errno));
        g_last_id--;
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    int written = fprintf(f,
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
    if (written < 0) {
        ESP_LOGE(TAG, "fprintf fallo al escribir medicion id=%lu: errno=%d (%s)", (unsigned long)id, errno, strerror(errno));
        fclose(f);
        g_last_id--;
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    if (fflush(f) != 0) {
        ESP_LOGE(TAG, "fflush fallo para medicion id=%lu: errno=%d (%s)", (unsigned long)id, errno, strerror(errno));
        fclose(f);
        g_last_id--;
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    if (fclose(f) != 0) {
        ESP_LOGE(TAG, "fclose fallo para medicion id=%lu: errno=%d (%s)", (unsigned long)id, errno, strerror(errno));
        g_last_id--;
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    if (g_lock) xSemaphoreGive(g_lock);

    if (out_id) {
        *out_id = id;
    }
    ESP_LOGI(TAG, "Medicion SD guardada OK id=%lu bytes=%d ultimo_id=%lu", (unsigned long)id, written, (unsigned long)g_last_id);
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
