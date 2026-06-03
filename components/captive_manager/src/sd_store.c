#include "sd_store.h"

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "sd_store";
static const char *MOUNT_POINT = "/sdcard";
static const char *CSV_PATH = "/sdcard/data.csv";
static const char *CSV_HEADER = "id,boot_id,uptime_s,time_valid,timestamp,co2,pm1p0,pm2p5,pm4p0,pm10p0,voc,nox,temp,hum,scd_temp,scd_hum,sen_temp,sen_hum,window_s\n";
#define STREAM_EXPORT_BATCH_ROWS 64

static bool g_ready = false;
static uint32_t g_last_id = 0;
static SemaphoreHandle_t g_lock = NULL;
static long *g_id_offsets = NULL;
static uint32_t g_id_offsets_capacity = 0;

static bool ensure_offset_capacity(uint32_t id) {
    if (id < g_id_offsets_capacity) {
        return true;
    }
    uint32_t new_capacity = g_id_offsets_capacity ? g_id_offsets_capacity : 256;
    while (id >= new_capacity) {
        new_capacity *= 2;
    }
    long *new_offsets = realloc(g_id_offsets, sizeof(long) * new_capacity);
    if (!new_offsets) {
        ESP_LOGW(TAG, "No hay memoria para indice SD id=%lu", (unsigned long)id);
        return false;
    }
    for (uint32_t i = g_id_offsets_capacity; i < new_capacity; ++i) {
        new_offsets[i] = -1;
    }
    g_id_offsets = new_offsets;
    g_id_offsets_capacity = new_capacity;
    return true;
}

static void set_id_offset(uint32_t id, long offset) {
    if (id == 0 || offset < 0) {
        return;
    }
    if (ensure_offset_capacity(id)) {
        g_id_offsets[id] = offset;
    }
}

static long get_id_offset(uint32_t id) {
    if (id == 0 || id >= g_id_offsets_capacity || !g_id_offsets) {
        return -1;
    }
    return g_id_offsets[id];
}

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

    char line[320];
    uint32_t last = 0;
    uint32_t rows = 0;
    while (1) {
        long offset = ftell(f);
        if (!fgets(line, sizeof(line), f)) {
            break;
        }
        char *end = NULL;
        unsigned long id = strtoul(line, &end, 10);
        if (end && *end == ',' && id > 0) {
            set_id_offset((uint32_t)id, offset);
            if (id > last) {
                last = (uint32_t)id;
            }
            rows++;
        }
    }
    fclose(f);
    ESP_LOGI(TAG, "CSV escaneado/indexado: filas=%lu ultimo_id=%lu indice_cap=%lu",
             (unsigned long)rows, (unsigned long)last, (unsigned long)g_id_offsets_capacity);
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
        .max_files = 8,
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

esp_err_t sd_store_clear(void) {
    if (!g_ready) {
        ESP_LOGW(TAG, "No se puede borrar CSV: SD no esta lista");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    FILE *f = fopen(CSV_PATH, "w");
    if (!f) {
        ESP_LOGE(TAG, "No se pudo truncar CSV: %s errno=%d (%s)", CSV_PATH, errno, strerror(errno));
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }
    int written = fputs(CSV_HEADER, f);
    if (written == EOF) {
        ESP_LOGE(TAG, "No se pudo reescribir encabezado CSV: errno=%d (%s)", errno, strerror(errno));
        fclose(f);
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }
    if (fclose(f) != 0) {
        ESP_LOGE(TAG, "Error cerrando CSV tras borrado: errno=%d (%s)", errno, strerror(errno));
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }
    g_last_id = 0;
    if (g_id_offsets) {
        for (uint32_t i = 0; i < g_id_offsets_capacity; ++i) {
            g_id_offsets[i] = -1;
        }
    }
    if (g_lock) xSemaphoreGive(g_lock);
    ESP_LOGI(TAG, "Historial SD borrado correctamente; ultimo_id=0");
    return ESP_OK;
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
             "Guardando medicion SD id=%lu boot_id=%lu uptime_s=%lu time_valid=%s timestamp=%s co2=%u pm2.5=%.2f voc=%.2f nox=%.2f temp=%.2f hum=%.2f scd=%.2f/%.2f sen=%.2f/%.2f window_s=%lu",
             (unsigned long)id,
             (unsigned long)reading->boot_id,
             (unsigned long)reading->uptime_s,
             reading->time_valid ? "true" : "false",
             reading->timestamp[0] ? reading->timestamp : "SIN_HORA",
             reading->co2,
             reading->pm2p5,
             reading->voc,
             reading->nox,
             reading->temp,
             reading->hum,
             reading->scd_temp,
             reading->scd_hum,
             reading->sen_temp,
             reading->sen_hum,
             (unsigned long)reading->window_s);

    FILE *f = fopen(CSV_PATH, "a");
    if (!f) {
        ESP_LOGE(TAG, "No se pudo abrir CSV para append: %s errno=%d (%s)", CSV_PATH, errno, strerror(errno));
        g_last_id--;
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    long row_offset = ftell(f);
    int written = fprintf(f,
                          "%lu,%lu,%lu,%u,%s,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu\n",
                          (unsigned long)id,
                          (unsigned long)reading->boot_id,
                          (unsigned long)reading->uptime_s,
                          reading->time_valid ? 1U : 0U,
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
                          reading->scd_temp,
                          reading->scd_hum,
                          reading->sen_temp,
                          reading->sen_hum,
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

    set_id_offset(id, row_offset);

    if (g_lock) xSemaphoreGive(g_lock);

    if (out_id) {
        *out_id = id;
    }
    ESP_LOGI(TAG, "Medicion SD guardada OK id=%lu bytes=%d ultimo_id=%lu", (unsigned long)id, written, (unsigned long)g_last_id);
    return ESP_OK;
}

typedef struct {
    uint32_t id;
    uint32_t boot_id;
    uint32_t uptime_s;
    bool time_valid;
    char timestamp[32];
    uint32_t co2;
    float pm1p0;
    float pm2p5;
    float pm4p0;
    float pm10p0;
    float voc;
    float nox;
    float temp;
    float hum;
    float scd_temp;
    float scd_hum;
    float sen_temp;
    float sen_hum;
    uint32_t window_s;
} sd_store_row_t;

static bool parse_u32_field(const char *text, uint32_t *out) {
    if (!text || !out || text[0] == '\0') {
        return false;
    }
    char *end = NULL;
    unsigned long value = strtoul(text, &end, 10);
    if (!end || *end != '\0') {
        return false;
    }
    *out = (uint32_t)value;
    return true;
}

static bool parse_float_field(const char *text, float *out) {
    if (!text || !out || text[0] == '\0') {
        return false;
    }
    char *end = NULL;
    float value = strtof(text, &end);
    if (!end || *end != '\0') {
        return false;
    }
    *out = value;
    return true;
}

static int split_csv_simple(char *line, char **fields, int max_fields) {
    int count = 0;
    char *start = line;
    for (char *p = line; *p && count < max_fields; ++p) {
        if (*p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        }
        if (*p == ',') {
            *p = '\0';
            fields[count++] = start;
            start = p + 1;
        }
    }
    if (count < max_fields) {
        fields[count++] = start;
    }
    return count;
}

static bool parse_csv_line(const char *line, sd_store_row_t *row) {
    if (!line || !row) {
        return false;
    }

    char copy[320];
    snprintf(copy, sizeof(copy), "%s", line);

    char *fields[20] = {0};
    int n = split_csv_simple(copy, fields, 20);
    memset(row, 0, sizeof(*row));

    // Formato nuevo extendido:
    // id,boot_id,uptime_s,time_valid,timestamp,co2,pm1p0,pm2p5,pm4p0,pm10p0,voc,nox,temp,hum,scd_temp,scd_hum,sen_temp,sen_hum,window_s
    if (n >= 19 && parse_u32_field(fields[0], &row->id) && parse_u32_field(fields[1], &row->boot_id)) {
        uint32_t time_valid = 0;
        return parse_u32_field(fields[2], &row->uptime_s) &&
               parse_u32_field(fields[3], &time_valid) &&
               (row->time_valid = time_valid != 0, true) &&
               (snprintf(row->timestamp, sizeof(row->timestamp), "%s", fields[4] ? fields[4] : ""), true) &&
               parse_u32_field(fields[5], &row->co2) &&
               parse_float_field(fields[6], &row->pm1p0) &&
               parse_float_field(fields[7], &row->pm2p5) &&
               parse_float_field(fields[8], &row->pm4p0) &&
               parse_float_field(fields[9], &row->pm10p0) &&
               parse_float_field(fields[10], &row->voc) &&
               parse_float_field(fields[11], &row->nox) &&
               parse_float_field(fields[12], &row->temp) &&
               parse_float_field(fields[13], &row->hum) &&
               parse_float_field(fields[14], &row->scd_temp) &&
               parse_float_field(fields[15], &row->scd_hum) &&
               parse_float_field(fields[16], &row->sen_temp) &&
               parse_float_field(fields[17], &row->sen_hum) &&
               parse_u32_field(fields[18], &row->window_s);
    }

    // Formato nuevo anterior:
    // id,boot_id,uptime_s,time_valid,timestamp,co2,pm1p0,pm2p5,pm4p0,pm10p0,voc,nox,temp,hum,window_s
    if (n >= 15 && parse_u32_field(fields[0], &row->id) && parse_u32_field(fields[1], &row->boot_id)) {
        uint32_t time_valid = 0;
        bool ok = parse_u32_field(fields[2], &row->uptime_s) &&
               parse_u32_field(fields[3], &time_valid) &&
               (row->time_valid = time_valid != 0, true) &&
               (snprintf(row->timestamp, sizeof(row->timestamp), "%s", fields[4] ? fields[4] : ""), true) &&
               parse_u32_field(fields[5], &row->co2) &&
               parse_float_field(fields[6], &row->pm1p0) &&
               parse_float_field(fields[7], &row->pm2p5) &&
               parse_float_field(fields[8], &row->pm4p0) &&
               parse_float_field(fields[9], &row->pm10p0) &&
               parse_float_field(fields[10], &row->voc) &&
               parse_float_field(fields[11], &row->nox) &&
               parse_float_field(fields[12], &row->temp) &&
               parse_float_field(fields[13], &row->hum) &&
               parse_u32_field(fields[14], &row->window_s);
        if (ok) {
            row->scd_temp = row->temp;
            row->scd_hum = row->hum;
            row->sen_temp = row->temp;
            row->sen_hum = row->hum;
        }
        return ok;
    }

    // Compatibilidad con formato anterior:
    // id,timestamp,co2,pm1p0,pm2p5,pm4p0,pm10p0,voc,nox,temp,hum,window_s
    if (n >= 12 && parse_u32_field(fields[0], &row->id)) {
        row->time_valid = fields[1] && fields[1][0] != '\0';
        snprintf(row->timestamp, sizeof(row->timestamp), "%s", fields[1] ? fields[1] : "");
        bool ok = parse_u32_field(fields[2], &row->co2) &&
               parse_float_field(fields[3], &row->pm1p0) &&
               parse_float_field(fields[4], &row->pm2p5) &&
               parse_float_field(fields[5], &row->pm4p0) &&
               parse_float_field(fields[6], &row->pm10p0) &&
               parse_float_field(fields[7], &row->voc) &&
               parse_float_field(fields[8], &row->nox) &&
               parse_float_field(fields[9], &row->temp) &&
               parse_float_field(fields[10], &row->hum) &&
               parse_u32_field(fields[11], &row->window_s);
        if (ok) {
            row->scd_temp = row->temp;
            row->scd_hum = row->hum;
            row->sen_temp = row->temp;
            row->sen_hum = row->hum;
        }
        return ok;
    }

    return false;
}

static void add_row_fields(cJSON *row, const sd_store_row_t *parsed) {
    cJSON_AddNumberToObject(row, "id", parsed->id);
    cJSON_AddNumberToObject(row, "measurement_id", parsed->id);
    cJSON_AddNumberToObject(row, "boot_id", parsed->boot_id);
    cJSON_AddNumberToObject(row, "uptime_s", parsed->uptime_s);
    cJSON_AddBoolToObject(row, "time_valid", parsed->time_valid);
    cJSON_AddStringToObject(row, "time_source", parsed->time_valid ? "esp" : "pending_estimate");
    if (parsed->timestamp[0]) {
        cJSON_AddStringToObject(row, "timestamp", parsed->timestamp);
    } else {
        cJSON_AddNullToObject(row, "timestamp");
    }
    cJSON_AddNumberToObject(row, "co2", parsed->co2);
    cJSON_AddNumberToObject(row, "pm1p0", parsed->pm1p0);
    cJSON_AddNumberToObject(row, "pm2p5", parsed->pm2p5);
    cJSON_AddNumberToObject(row, "pm4p0", parsed->pm4p0);
    cJSON_AddNumberToObject(row, "pm10p0", parsed->pm10p0);
    cJSON_AddNumberToObject(row, "voc", parsed->voc);
    cJSON_AddNumberToObject(row, "nox", parsed->nox);
    cJSON_AddNumberToObject(row, "temp", parsed->temp);
    cJSON_AddNumberToObject(row, "hum", parsed->hum);
    cJSON_AddNumberToObject(row, "scd_temp", parsed->scd_temp);
    cJSON_AddNumberToObject(row, "scd_hum", parsed->scd_hum);
    cJSON_AddNumberToObject(row, "sen_temp", parsed->sen_temp);
    cJSON_AddNumberToObject(row, "sen_hum", parsed->sen_hum);
    cJSON_AddNumberToObject(row, "window_s", parsed->window_s);
}

static void add_row_json(cJSON *array, const sd_store_row_t *parsed) {
    cJSON *row = cJSON_CreateObject();
    add_row_fields(row, parsed);
    cJSON_AddItemToArray(array, row);
}

static esp_err_t write_row_ndjson(const sd_store_row_t *parsed, sd_store_ndjson_writer_t writer, void *ctx) {
    if (!parsed || !writer) {
        return ESP_ERR_INVALID_ARG;
    }

    char timestamp_field[96];
    if (parsed->timestamp[0]) {
        snprintf(timestamp_field, sizeof(timestamp_field), "\"%s\"", parsed->timestamp);
    } else {
        snprintf(timestamp_field, sizeof(timestamp_field), "null");
    }

    char line[640];
    int len = snprintf(
        line,
        sizeof(line),
        "{\"id\":%lu,\"measurement_id\":%lu,\"boot_id\":%lu,\"uptime_s\":%lu,"
        "\"time_valid\":%s,\"time_source\":\"%s\",\"timestamp\":%s,"
        "\"co2\":%lu,\"pm1p0\":%.2f,\"pm2p5\":%.2f,\"pm4p0\":%.2f,\"pm10p0\":%.2f,"
        "\"voc\":%.2f,\"nox\":%.2f,\"temp\":%.2f,\"hum\":%.2f,"
        "\"scd_temp\":%.2f,\"scd_hum\":%.2f,\"sen_temp\":%.2f,\"sen_hum\":%.2f,\"window_s\":%lu}\n",
        (unsigned long)parsed->id,
        (unsigned long)parsed->id,
        (unsigned long)parsed->boot_id,
        (unsigned long)parsed->uptime_s,
        parsed->time_valid ? "true" : "false",
        parsed->time_valid ? "esp" : "pending_estimate",
        timestamp_field,
        (unsigned long)parsed->co2,
        parsed->pm1p0,
        parsed->pm2p5,
        parsed->pm4p0,
        parsed->pm10p0,
        parsed->voc,
        parsed->nox,
        parsed->temp,
        parsed->hum,
        parsed->scd_temp,
        parsed->scd_hum,
        parsed->sen_temp,
        parsed->sen_hum,
        (unsigned long)parsed->window_s
    );
    if (len <= 0 || len >= (int)sizeof(line)) {
        return ESP_ERR_NO_MEM;
    }
    return writer(line, ctx);
}

static uint32_t normalize_limit(uint32_t limit) {
    if (limit == 0) {
        return 25;
    }
    if (limit > 100) {
        return 100;
    }
    return limit;
}

static uint32_t normalize_timeout_ms(uint32_t timeout_ms) {
    if (timeout_ms == 0) {
        return 1200;
    }
    if (timeout_ms > 30000) {
        return 30000;
    }
    return timeout_ms;
}

static bool scan_timed_out(int64_t started_us, uint32_t timeout_ms) {
    return (esp_timer_get_time() - started_us) > ((int64_t)timeout_ms * 1000LL);
}

esp_err_t sd_store_add_latest_reading(cJSON *object, uint32_t timeout_ms, bool *found) {
    if (found) {
        *found = false;
    }
    if (!g_ready || !object) {
        return ESP_ERR_INVALID_STATE;
    }
    timeout_ms = normalize_timeout_ms(timeout_ms);

    uint32_t target_id = g_last_id;
    if (target_id == 0) {
        return ESP_OK;
    }

    if (g_lock && xSemaphoreTake(g_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    FILE *f = fopen(CSV_PATH, "r");
    if (!f) {
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    long start_offset = get_id_offset(target_id);
    if (start_offset >= 0 && fseek(f, start_offset, SEEK_SET) != 0) {
        ESP_LOGW(TAG, "No se pudo posicionar CSV para latest id=%lu offset=%ld", (unsigned long)target_id, start_offset);
        rewind(f);
    } else if (start_offset < 0) {
        rewind(f);
    }

    char line[320];
    sd_store_row_t latest = {0};
    bool have_latest = false;
    int64_t started_us = esp_timer_get_time();
    while (fgets(line, sizeof(line), f)) {
        if (scan_timed_out(started_us, timeout_ms)) {
            fclose(f);
            if (g_lock) xSemaphoreGive(g_lock);
            return ESP_ERR_TIMEOUT;
        }
        sd_store_row_t parsed = {0};
        if (!parse_csv_line(line, &parsed)) {
            continue;
        }
        if (parsed.id > target_id) {
            break;
        }
        latest = parsed;
        have_latest = true;
        if (parsed.id == target_id) {
            break;
        }
    }
    fclose(f);
    if (g_lock) xSemaphoreGive(g_lock);

    if (have_latest) {
        add_row_fields(object, &latest);
        if (found) {
            *found = true;
        }
    }
    return ESP_OK;
}

esp_err_t sd_store_add_readings_since(cJSON *array, uint32_t after_id, uint32_t limit, uint32_t timeout_ms, uint32_t *added, uint32_t *scanned) {
    if (!g_ready || !array) {
        return ESP_ERR_INVALID_STATE;
    }
    limit = normalize_limit(limit);
    timeout_ms = normalize_timeout_ms(timeout_ms);

    if (g_lock && xSemaphoreTake(g_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    FILE *f = fopen(CSV_PATH, "r");
    if (!f) {
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    char line[320];
    uint32_t count = 0;
    uint32_t scanned_rows = 0;
    esp_err_t result = ESP_OK;
    int64_t started_us = esp_timer_get_time();
    while (fgets(line, sizeof(line), f) && count < limit) {
        if ((scanned_rows & 0x0F) == 0 && scan_timed_out(started_us, timeout_ms)) {
            result = ESP_ERR_TIMEOUT;
            break;
        }
        sd_store_row_t parsed = {0};
        if (!parse_csv_line(line, &parsed)) {
            continue;
        }
        scanned_rows++;
        if (parsed.id <= after_id) {
            continue;
        }

        add_row_json(array, &parsed);
        count++;
    }
    fclose(f);
    if (g_lock) xSemaphoreGive(g_lock);

    if (added) {
        *added = count;
    }
    if (scanned) {
        *scanned = scanned_rows;
    }
    return result;
}

esp_err_t sd_store_add_readings_range(cJSON *array, uint32_t from_id, uint32_t to_id, uint32_t limit, uint32_t timeout_ms, uint32_t *added, uint32_t *scanned) {
    if (!g_ready || !array) {
        return ESP_ERR_INVALID_STATE;
    }
    if (from_id == 0 || to_id == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t start_id = from_id < to_id ? from_id : to_id;
    uint32_t end_id = from_id > to_id ? from_id : to_id;
    limit = normalize_limit(limit);
    timeout_ms = normalize_timeout_ms(timeout_ms);

    if (g_lock && xSemaphoreTake(g_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    FILE *f = fopen(CSV_PATH, "r");
    if (!f) {
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    long start_offset = get_id_offset(start_id);
    if (start_offset >= 0 && fseek(f, start_offset, SEEK_SET) != 0) {
        ESP_LOGW(TAG, "No se pudo posicionar CSV en id=%lu offset=%ld", (unsigned long)start_id, start_offset);
        rewind(f);
    }

    char line[320];
    uint32_t count = 0;
    uint32_t scanned_rows = 0;
    esp_err_t result = ESP_OK;
    int64_t started_us = esp_timer_get_time();
    while (fgets(line, sizeof(line), f) && count < limit) {
        if ((scanned_rows & 0x0F) == 0 && scan_timed_out(started_us, timeout_ms)) {
            result = ESP_ERR_TIMEOUT;
            break;
        }
        sd_store_row_t parsed = {0};
        if (!parse_csv_line(line, &parsed)) {
            continue;
        }
        scanned_rows++;
        if (parsed.id < start_id) {
            continue;
        }
        if (parsed.id > end_id) {
            break;
        }

        add_row_json(array, &parsed);
        count++;
    }
    fclose(f);
    if (g_lock) xSemaphoreGive(g_lock);

    if (added) {
        *added = count;
    }
    if (scanned) {
        *scanned = scanned_rows;
    }
    return result;
}

esp_err_t sd_store_stream_readings_range_ndjson(uint32_t from_id, uint32_t to_id, uint32_t timeout_ms, sd_store_ndjson_writer_t writer, void *ctx, uint32_t *added, uint32_t *scanned) {
    if (!g_ready || !writer) {
        return ESP_ERR_INVALID_STATE;
    }
    if (from_id == 0 || to_id == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t start_id = from_id < to_id ? from_id : to_id;
    uint32_t end_id = from_id > to_id ? from_id : to_id;
    if (timeout_ms == 0) {
        timeout_ms = 120000;
    }
    if (timeout_ms < 30000) {
        timeout_ms = 30000;
    }
    if (timeout_ms > 120000) {
        timeout_ms = 120000;
    }

    sd_store_row_t *batch = calloc(STREAM_EXPORT_BATCH_ROWS, sizeof(sd_store_row_t));
    if (!batch) {
        return ESP_ERR_NO_MEM;
    }

    uint32_t next_id = start_id;
    uint32_t count = 0;
    uint32_t scanned_rows = 0;
    esp_err_t result = ESP_OK;
    int64_t started_us = esp_timer_get_time();

    while (next_id <= end_id) {
        if (scan_timed_out(started_us, timeout_ms)) {
            result = ESP_ERR_TIMEOUT;
            break;
        }

        uint32_t batch_count = 0;
        bool reached_end = false;

        if (g_lock && xSemaphoreTake(g_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
            result = ESP_ERR_TIMEOUT;
            break;
        }

        FILE *f = fopen(CSV_PATH, "r");
        if (!f) {
            result = ESP_FAIL;
            if (g_lock) xSemaphoreGive(g_lock);
            break;
        }

        long start_offset = get_id_offset(next_id);
        if (start_offset >= 0 && fseek(f, start_offset, SEEK_SET) != 0) {
            ESP_LOGW(TAG, "No se pudo posicionar CSV export en id=%lu offset=%ld", (unsigned long)next_id, start_offset);
            rewind(f);
        }

        char line[320];
        while (batch_count < STREAM_EXPORT_BATCH_ROWS && fgets(line, sizeof(line), f)) {
            if ((scanned_rows & 0x0F) == 0 && scan_timed_out(started_us, timeout_ms)) {
                result = ESP_ERR_TIMEOUT;
                reached_end = true;
                break;
            }
            sd_store_row_t parsed = {0};
            if (!parse_csv_line(line, &parsed)) {
                continue;
            }
            scanned_rows++;
            if (parsed.id < next_id) {
                continue;
            }
            if (parsed.id > end_id) {
                reached_end = true;
                break;
            }
            batch[batch_count++] = parsed;
            next_id = parsed.id + 1;
        }

        if (fclose(f) != 0 && result == ESP_OK) {
            result = ESP_FAIL;
        }
        if (g_lock) xSemaphoreGive(g_lock);

        for (uint32_t i = 0; i < batch_count && result == ESP_OK; ++i) {
            result = write_row_ndjson(&batch[i], writer, ctx);
            if (result == ESP_OK) {
                count++;
            }
        }

        if (result != ESP_OK || reached_end || batch_count == 0) {
            break;
        }
        vTaskDelay(1);
    }

    free(batch);
    if (added) {
        *added = count;
    }
    if (scanned) {
        *scanned = scanned_rows;
    }
    return result;
}

esp_err_t sd_store_add_recent_readings(cJSON *array, uint32_t after_id, uint32_t before_id, uint32_t limit, uint32_t timeout_ms, uint32_t *added, uint32_t *scanned) {
    if (!g_ready || !array) {
        return ESP_ERR_INVALID_STATE;
    }
    limit = normalize_limit(limit);
    timeout_ms = normalize_timeout_ms(timeout_ms);

    sd_store_row_t *ring = calloc(limit, sizeof(sd_store_row_t));
    if (!ring) {
        return ESP_ERR_NO_MEM;
    }

    if (g_lock && xSemaphoreTake(g_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        free(ring);
        return ESP_ERR_TIMEOUT;
    }
    FILE *f = fopen(CSV_PATH, "r");
    if (!f) {
        if (g_lock) xSemaphoreGive(g_lock);
        free(ring);
        return ESP_FAIL;
    }

    char line[320];
    uint32_t matched = 0;
    uint32_t scanned_rows = 0;
    esp_err_t result = ESP_OK;
    int64_t started_us = esp_timer_get_time();
    while (fgets(line, sizeof(line), f)) {
        if ((scanned_rows & 0x0F) == 0 && scan_timed_out(started_us, timeout_ms)) {
            result = ESP_ERR_TIMEOUT;
            break;
        }
        sd_store_row_t parsed = {0};
        if (!parse_csv_line(line, &parsed)) {
            continue;
        }
        scanned_rows++;
        if (parsed.id <= after_id) {
            continue;
        }
        if (before_id > 0 && parsed.id >= before_id) {
            continue;
        }
        ring[matched % limit] = parsed;
        matched++;
    }
    fclose(f);
    if (g_lock) xSemaphoreGive(g_lock);

    uint32_t count = matched < limit ? matched : limit;
    uint32_t start = matched > limit ? (matched % limit) : 0;
    for (uint32_t i = 0; i < count; ++i) {
        add_row_json(array, &ring[(start + i) % limit]);
    }
    free(ring);

    if (added) {
        *added = count;
    }
    if (scanned) {
        *scanned = scanned_rows;
    }
    return result;
}
