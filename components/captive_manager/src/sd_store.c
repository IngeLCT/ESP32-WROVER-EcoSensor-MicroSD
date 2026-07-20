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
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

static const char *TAG = "sd_store";
static const char *MOUNT_POINT = "/sdcard";
static const char *CSV_PATH = "/sdcard/data.csv";
static const char *CSV_TMP_PATH = "/sdcard/data.tmp";
static const char *CSV_BAK_PATH = "/sdcard/data.bak";
static const char *STATE_A_PATH = "/sdcard/data_state_a.bin";
static const char *STATE_B_PATH = "/sdcard/data_state_b.bin";
static const char *INDEX_PATH = "/sdcard/data.idx";
static const char *INDEX_TMP_PATH = "/sdcard/data.idx.tmp";
static const char *INDEX_BAK_PATH = "/sdcard/data.idx.bak";
static const char *CSV_HEADER = "id,boot_id,uptime_s,time_valid,time_source,timestamp,co2,pm1p0,pm2p5,pm4p0,pm10p0,voc,nox,temp,hum,scd_temp,scd_hum,sen_temp,sen_hum,gps_valid,gps_lat,gps_lon,gps_satellites,gps_hdop,gps_age_ms,window_s\n";
#define STREAM_EXPORT_BATCH_ROWS 64
#define CSV_FORMAT_VERSION 3U
#define CHECKPOINT_MAGIC 0x45534350UL /* ESCP */
#define CHECKPOINT_VERSION 1U
#define INDEX_MAGIC 0x45534958UL /* ESIX */
#define INDEX_VERSION 1U
#define INDEX_INTERVAL 64U
#define REBUILD_CHUNK_ROWS 128U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t checkpoint_version;
    uint16_t csv_format_version;
    uint64_t generation;
    uint32_t last_id;
    uint64_t row_count;
    uint64_t csv_size;
    uint64_t last_row_offset;
    uint64_t confirmed_end_offset;
    uint32_t last_row_crc;
    uint32_t index_points;
    uint32_t flags;
    uint8_t reserved[12];
    uint32_t structure_crc;
} sd_checkpoint_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t index_version;
    uint16_t csv_format_version;
    uint16_t interval;
    uint16_t entry_size;
    uint32_t entry_count;
    uint64_t csv_size;
    uint32_t entries_crc;
    uint32_t header_crc;
} sd_index_header_t;

typedef struct __attribute__((packed)) {
    uint32_t measurement_id;
    uint64_t csv_offset;
    uint32_t entry_crc;
} sd_index_entry_t;

static bool g_ready = false;
static uint32_t g_last_id = 0;
static SemaphoreHandle_t g_lock = NULL;
static sd_checkpoint_t g_checkpoint = {0};
static bool g_checkpoint_valid = false;
static bool g_index_ready = false;
static bool g_index_rebuilding = false;
static uint32_t g_rebuild_token = 0;
static sd_index_entry_t *g_index_entries = NULL;
static uint32_t g_index_count = 0;
static uint32_t g_index_capacity = 0;

static uint32_t crc32_update(uint32_t crc, const void *data, size_t len) {
    const uint8_t *bytes = (const uint8_t *)data;
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)-(int32_t)(crc & 1U));
        }
    }
    return ~crc;
}

static bool sync_file(FILE *f) {
    return f && fflush(f) == 0 && fsync(fileno(f)) == 0;
}

static bool ensure_index_capacity(uint32_t count) {
    if (count <= g_index_capacity) return true;
    uint32_t capacity = g_index_capacity ? g_index_capacity : 32U;
    while (capacity < count) capacity *= 2U;
    sd_index_entry_t *entries = realloc(g_index_entries, capacity * sizeof(*entries));
    if (!entries) return false;
    g_index_entries = entries;
    g_index_capacity = capacity;
    return true;
}

static bool sparse_point_id(uint32_t id) {
    return id == 1U || (id % INDEX_INTERVAL) == 0U;
}

static bool add_index_entry_ram(uint32_t id, uint64_t offset) {
    if (!sparse_point_id(id)) return true;
    if (g_index_count > 0 && g_index_entries[g_index_count - 1].measurement_id == id) return true;
    if (!ensure_index_capacity(g_index_count + 1U)) return false;
    sd_index_entry_t *entry = &g_index_entries[g_index_count++];
    memset(entry, 0, sizeof(*entry));
    entry->measurement_id = id;
    entry->csv_offset = offset;
    entry->entry_crc = crc32_update(0, entry, offsetof(sd_index_entry_t, entry_crc));
    return true;
}

static long get_id_offset(uint32_t id) {
    if (id == 0 || g_index_count == 0) return -1;
    uint32_t low = 0, high = g_index_count;
    while (low < high) {
        uint32_t mid = low + (high - low) / 2U;
        if (g_index_entries[mid].measurement_id <= id) low = mid + 1U;
        else high = mid;
    }
    if (low == 0) return -1;
    uint64_t offset = g_index_entries[low - 1U].csv_offset;
    return offset <= LONG_MAX ? (long)offset : -1;
}

static bool file_exists(const char *path) {
    struct stat st = {0};
    return stat(path, &st) == 0;
}

static void ensure_header(void) {
    /* Recupera el CSV original si un reinicio interrumpio una migracion justo
       despues de mover data.csv a data.bak. */
    if (!file_exists(CSV_PATH) && file_exists(CSV_BAK_PATH)) {
        if (rename(CSV_BAK_PATH, CSV_PATH) == 0) {
            ESP_LOGW(TAG, "CSV original restaurado desde respaldo de migracion");
        } else {
            ESP_LOGE(TAG, "No se pudo restaurar respaldo CSV: errno=%d (%s)", errno, strerror(errno));
            return;
        }
    }

    /* Si solo sobrevivio el temporal, se conserva como CSV activo. Si ya hay
       un CSV (original o restaurado), el temporal incompleto se descarta. */
    if (!file_exists(CSV_PATH) && !file_exists(CSV_BAK_PATH) && file_exists(CSV_TMP_PATH)) {
        if (rename(CSV_TMP_PATH, CSV_PATH) == 0) {
            ESP_LOGW(TAG, "CSV migrado recuperado desde archivo temporal");
        } else {
            ESP_LOGE(TAG, "No se pudo recuperar CSV temporal: errno=%d (%s)", errno, strerror(errno));
            return;
        }
    }
    if (file_exists(CSV_PATH)) {
        unlink(CSV_TMP_PATH);
    }

    if (file_exists(CSV_PATH)) {
        FILE *existing = fopen(CSV_PATH, "r");
        if (!existing) {
            ESP_LOGW(TAG, "CSV existente no se pudo abrir para validar encabezado: %s errno=%d (%s)", CSV_PATH, errno, strerror(errno));
            return;
        }

        char first_line[512] = {0};
        bool has_header = fgets(first_line, sizeof(first_line), existing) != NULL;
        bool first_line_is_header = has_header && strncmp(first_line, "id,", 3) == 0;
        bool header_is_current = has_header && strstr(first_line, "gps_lat") != NULL &&
                                 strstr(first_line, "gps_lon") != NULL && strstr(first_line, "time_source") != NULL;
        if (header_is_current) {
            fclose(existing);
            if (unlink(CSV_BAK_PATH) != 0 && errno != ENOENT) {
                ESP_LOGW(TAG, "CSV valido, pero no se pudo borrar respaldo antiguo: errno=%d (%s)", errno, strerror(errno));
            }
            ESP_LOGI(TAG, "CSV existente con encabezado GPS encontrado: %s", CSV_PATH);
            return;
        }

        ESP_LOGW(TAG, "CSV existente con encabezado anterior; actualizando sin borrar mediciones");
        FILE *tmp = fopen(CSV_TMP_PATH, "w");
        if (!tmp) {
            fclose(existing);
            ESP_LOGE(TAG, "No se pudo crear CSV temporal para migrar encabezado: %s errno=%d (%s)", CSV_TMP_PATH, errno, strerror(errno));
            return;
        }
        if (fputs(CSV_HEADER, tmp) == EOF) {
            ESP_LOGE(TAG, "No se pudo escribir encabezado GPS temporal: errno=%d (%s)", errno, strerror(errno));
            fclose(existing);
            fclose(tmp);
            unlink(CSV_TMP_PATH);
            return;
        }
        if (has_header && !first_line_is_header && fputs(first_line, tmp) == EOF) {
            ESP_LOGE(TAG, "No se pudo conservar primera fila durante migracion CSV: errno=%d (%s)", errno, strerror(errno));
            fclose(existing);
            fclose(tmp);
            unlink(CSV_TMP_PATH);
            return;
        }

        char line[512];
        while (fgets(line, sizeof(line), existing)) {
            if (fputs(line, tmp) == EOF) {
                ESP_LOGE(TAG, "Error copiando fila existente durante migracion CSV: errno=%d (%s)", errno, strerror(errno));
                fclose(existing);
                fclose(tmp);
                unlink(CSV_TMP_PATH);
                return;
            }
        }
        fclose(existing);
        if (fclose(tmp) != 0) {
            ESP_LOGE(TAG, "Error cerrando CSV temporal: errno=%d (%s)", errno, strerror(errno));
            unlink(CSV_TMP_PATH);
            return;
        }
        if (file_exists(CSV_BAK_PATH) && unlink(CSV_BAK_PATH) != 0) {
            ESP_LOGE(TAG, "No se pudo limpiar respaldo CSV anterior: errno=%d (%s)", errno, strerror(errno));
            unlink(CSV_TMP_PATH);
            return;
        }
        if (rename(CSV_PATH, CSV_BAK_PATH) != 0) {
            ESP_LOGE(TAG, "No se pudo respaldar CSV antes de migrar encabezado: errno=%d (%s)", errno, strerror(errno));
            unlink(CSV_TMP_PATH);
            return;
        }
        if (rename(CSV_TMP_PATH, CSV_PATH) != 0) {
            int replace_errno = errno;
            ESP_LOGE(TAG, "No se pudo instalar CSV migrado: errno=%d (%s)", replace_errno, strerror(replace_errno));
            if (rename(CSV_BAK_PATH, CSV_PATH) == 0) {
                ESP_LOGW(TAG, "CSV original restaurado despues del fallo de migracion");
                unlink(CSV_TMP_PATH);
            } else {
                ESP_LOGE(TAG,
                         "No se pudo restaurar CSV original; se conserva en %s: errno=%d (%s)",
                         CSV_BAK_PATH,
                         errno,
                         strerror(errno));
            }
            return;
        }
        if (unlink(CSV_BAK_PATH) != 0 && errno != ENOENT) {
            ESP_LOGW(TAG, "CSV migrado, pero no se pudo borrar respaldo: errno=%d (%s)", errno, strerror(errno));
        }
        ESP_LOGI(TAG, "Encabezado CSV migrado a formato GPS correctamente");
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

static bool parse_line_id(const char *line, uint32_t *id) {
    if (!line || !id || line[0] == '\0' || strncmp(line, "id,", 3) == 0) return false;
    char *end = NULL;
    unsigned long value = strtoul(line, &end, 10);
    if (!end || *end != ',' || value == 0 || value > UINT32_MAX) return false;
    *id = (uint32_t)value;
    return true;
}

static bool csv_size(uint64_t *size) {
    struct stat st = {0};
    if (!size || stat(CSV_PATH, &st) != 0 || st.st_size < 0) return false;
    *size = (uint64_t)st.st_size;
    return true;
}

static bool read_row_at(uint64_t offset, char *line, size_t line_size, uint64_t expected_end) {
    if (!line || line_size < 2 || offset > LONG_MAX) return false;
    FILE *f = fopen(CSV_PATH, "rb");
    if (!f) return false;
    bool ok = fseek(f, (long)offset, SEEK_SET) == 0 && fgets(line, line_size, f) != NULL;
    long end = ok ? ftell(f) : -1;
    fclose(f);
    return ok && end >= 0 && (expected_end == 0 || (uint64_t)end == expected_end) && strchr(line, '\n') != NULL;
}

static uint32_t checkpoint_crc(const sd_checkpoint_t *state) {
    return crc32_update(0, state, offsetof(sd_checkpoint_t, structure_crc));
}

static bool read_checkpoint(const char *path, sd_checkpoint_t *state) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    sd_checkpoint_t candidate = {0};
    bool ok = fread(&candidate, 1, sizeof(candidate), f) == sizeof(candidate) && fgetc(f) == EOF;
    fclose(f);
    if (!ok || candidate.magic != CHECKPOINT_MAGIC || candidate.checkpoint_version != CHECKPOINT_VERSION ||
        candidate.csv_format_version != CSV_FORMAT_VERSION || candidate.confirmed_end_offset != candidate.csv_size ||
        candidate.structure_crc != checkpoint_crc(&candidate)) return false;
    *state = candidate;
    return true;
}

static bool write_checkpoint_locked(const sd_checkpoint_t *source) {
    sd_checkpoint_t next = *source;
    next.magic = CHECKPOINT_MAGIC;
    next.checkpoint_version = CHECKPOINT_VERSION;
    next.csv_format_version = CSV_FORMAT_VERSION;
    next.generation = g_checkpoint_valid ? g_checkpoint.generation + 1U : 1U;
    next.index_points = g_index_count;
    next.structure_crc = checkpoint_crc(&next);
    const char *path = (next.generation & 1U) ? STATE_A_PATH : STATE_B_PATH;
    FILE *f = fopen(path, "wb");
    bool ok = f && fwrite(&next, 1, sizeof(next), f) == sizeof(next) && sync_file(f);
    if (f && fclose(f) != 0) ok = false;
    if (!ok) {
        ESP_LOGE(TAG, "No se pudo confirmar checkpoint %s: errno=%d (%s)", path, errno, strerror(errno));
        return false;
    }
    g_checkpoint = next;
    g_checkpoint_valid = true;
    return true;
}

static bool validate_checkpoint_row(const sd_checkpoint_t *state, uint64_t actual_size) {
    if (!state || state->csv_size > actual_size) return false;
    if (state->last_id == 0) return state->last_row_crc == 0 && state->last_row_offset == 0;
    char line[512] = {0};
    uint32_t id = 0;
    return read_row_at(state->last_row_offset, line, sizeof(line), state->confirmed_end_offset) &&
           parse_line_id(line, &id) && id == state->last_id &&
           crc32_update(0, line, strlen(line)) == state->last_row_crc;
}

static bool truncate_csv(uint64_t size) {
    FILE *f = fopen(CSV_PATH, "r+b");
    if (!f) return false;
    bool ok = fflush(f) == 0 && ftruncate(fileno(f), (off_t)size) == 0 && fsync(fileno(f)) == 0;
    fclose(f);
    return ok;
}

static bool recover_last_complete_row(sd_checkpoint_t *state) {
    uint64_t size = 0;
    if (!state || !csv_size(&size)) return false;
    memset(state, 0, sizeof(*state));
    state->csv_size = size;
    state->confirmed_end_offset = size;
    if (size == 0) return true;

    FILE *f = fopen(CSV_PATH, "rb");
    if (!f) return false;
    uint64_t end = size;
    if (fseek(f, (long)(size - 1U), SEEK_SET) != 0) { fclose(f); return false; }
    int last_char = fgetc(f);
    if (last_char != '\n') {
        while (end > 0) {
            end--;
            if (fseek(f, (long)end, SEEK_SET) != 0) break;
            if (fgetc(f) == '\n') { end++; break; }
        }
        if (end < size) ESP_LOGW(TAG, "Cola CSV incompleta detectada; se truncan %llu bytes", (unsigned long long)(size - end));
    }
    if (end == 0) { fclose(f); return false; }

    uint64_t row_offset = end - 1U;
    while (row_offset > 0) {
        row_offset--;
        if (fseek(f, (long)row_offset, SEEK_SET) != 0) { fclose(f); return false; }
        if (fgetc(f) == '\n') { row_offset++; break; }
    }
    char line[512] = {0};
    bool got = fseek(f, (long)row_offset, SEEK_SET) == 0 && fgets(line, sizeof(line), f) != NULL;
    fclose(f);
    if (!got || !strchr(line, '\n')) return false;
    if (end < size && !truncate_csv(end)) return false;

    uint32_t id = 0;
    state->csv_size = end;
    state->confirmed_end_offset = end;
    if (!parse_line_id(line, &id)) return strncmp(line, "id,", 3) == 0;
    state->last_id = id;
    state->row_count = id; /* Los IDs del formato actual son monotónicos desde 1. */
    state->last_row_offset = row_offset;
    state->last_row_crc = crc32_update(0, line, strlen(line));
    return true;
}

static bool recover_checkpoint_tail(sd_checkpoint_t *state, uint64_t actual_size) {
    if (!state || state->confirmed_end_offset > actual_size || state->confirmed_end_offset > LONG_MAX) return false;
    if (state->confirmed_end_offset == actual_size) return true;
    FILE *f = fopen(CSV_PATH, "rb");
    if (!f || fseek(f, (long)state->confirmed_end_offset, SEEK_SET) != 0) { if (f) fclose(f); return false; }
    char line[512];
    uint64_t confirmed_end = state->confirmed_end_offset;
    while (1) {
        long row_offset = ftell(f);
        if (!fgets(line, sizeof(line), f)) break;
        long row_end = ftell(f);
        if (!strchr(line, '\n')) break;
        uint32_t id = 0;
        if (!parse_line_id(line, &id) || id <= state->last_id) break;
        state->last_id = id;
        state->row_count++;
        state->last_row_offset = (uint64_t)row_offset;
        state->last_row_crc = crc32_update(0, line, strlen(line));
        confirmed_end = (uint64_t)row_end;
    }
    fclose(f);
    if (confirmed_end < actual_size && !truncate_csv(confirmed_end)) return false;
    state->csv_size = confirmed_end;
    state->confirmed_end_offset = confirmed_end;
    return true;
}

static uint32_t index_header_crc(const sd_index_header_t *header) {
    return crc32_update(0, header, offsetof(sd_index_header_t, header_crc));
}

static bool persist_index_locked(void) {
    uint64_t index_csv_size = 0;
    csv_size(&index_csv_size);
    sd_index_header_t header = {
        .magic = INDEX_MAGIC, .index_version = INDEX_VERSION, .csv_format_version = CSV_FORMAT_VERSION,
        .interval = INDEX_INTERVAL, .entry_size = sizeof(sd_index_entry_t), .entry_count = g_index_count,
        .csv_size = index_csv_size,
    };
    header.entries_crc = crc32_update(0, g_index_entries, g_index_count * sizeof(*g_index_entries));
    header.header_crc = index_header_crc(&header);
    FILE *f = fopen(INDEX_TMP_PATH, "wb");
    bool ok = f && fwrite(&header, 1, sizeof(header), f) == sizeof(header) &&
              (g_index_count == 0 || fwrite(g_index_entries, sizeof(*g_index_entries), g_index_count, f) == g_index_count) &&
              sync_file(f);
    if (f && fclose(f) != 0) ok = false;
    if (!ok) { unlink(INDEX_TMP_PATH); return false; }
    unlink(INDEX_BAK_PATH);
    bool had_index = file_exists(INDEX_PATH);
    if (had_index && rename(INDEX_PATH, INDEX_BAK_PATH) != 0) { unlink(INDEX_TMP_PATH); return false; }
    if (rename(INDEX_TMP_PATH, INDEX_PATH) != 0) {
        if (had_index) rename(INDEX_BAK_PATH, INDEX_PATH);
        unlink(INDEX_TMP_PATH);
        return false;
    }
    unlink(INDEX_BAK_PATH);
    return true;
}

static bool load_index(void) {
    if (!file_exists(INDEX_PATH) && file_exists(INDEX_BAK_PATH)) rename(INDEX_BAK_PATH, INDEX_PATH);
    if (file_exists(INDEX_PATH)) {
        unlink(INDEX_TMP_PATH);
        unlink(INDEX_BAK_PATH);
    }
    FILE *f = fopen(INDEX_PATH, "rb");
    if (!f) return false;
    struct stat st = {0};
    bool file_size_ok = stat(INDEX_PATH, &st) == 0 && st.st_size >= (off_t)sizeof(sd_index_header_t);
    uint32_t expected_points = g_checkpoint.last_id == 0 ? 0U : 1U + (g_checkpoint.last_id / INDEX_INTERVAL);
    sd_index_header_t header = {0};
    bool ok = file_size_ok && fread(&header, 1, sizeof(header), f) == sizeof(header) && header.magic == INDEX_MAGIC &&
              header.index_version == INDEX_VERSION && header.csv_format_version == CSV_FORMAT_VERSION &&
              header.interval == INDEX_INTERVAL && header.entry_size == sizeof(sd_index_entry_t) &&
              header.header_crc == index_header_crc(&header) &&
              header.entry_count == expected_points &&
              (uint64_t)st.st_size == sizeof(header) + (uint64_t)header.entry_count * sizeof(sd_index_entry_t) &&
              header.csv_size <= g_checkpoint.csv_size && ensure_index_capacity(header.entry_count);
    if (ok && header.entry_count > 0) ok = fread(g_index_entries, sizeof(*g_index_entries), header.entry_count, f) == header.entry_count;
    fclose(f);
    if (!ok || header.entries_crc != crc32_update(0, g_index_entries, header.entry_count * sizeof(*g_index_entries))) {
        g_index_count = 0;
        return false;
    }
    for (uint32_t i = 0; i < header.entry_count; ++i) {
        sd_index_entry_t *entry = &g_index_entries[i];
        if (!sparse_point_id(entry->measurement_id) || entry->csv_offset >= g_checkpoint.csv_size ||
            entry->entry_crc != crc32_update(0, entry, offsetof(sd_index_entry_t, entry_crc)) ||
            (i > 0 && entry->measurement_id <= g_index_entries[i - 1].measurement_id)) {
            g_index_count = 0;
            return false;
        }
    }
    g_index_count = header.entry_count;
    return true;
}

static void index_rebuild_task(void *arg) {
    uint32_t token = (uint32_t)(uintptr_t)arg;
    sd_index_entry_t *rebuilt = NULL;
    uint32_t count = 0, capacity = 0;
    uint64_t snapshot_end = g_checkpoint.csv_size;
    uint32_t snapshot_last_id = g_checkpoint.last_id;
    bool rebuild_complete = false;
    FILE *f = fopen(CSV_PATH, "rb");
    if (!f) goto done;
    char line[512];
    while (token == g_rebuild_token) {
        uint32_t chunk = 0;
        if (g_lock && xSemaphoreTake(g_lock, pdMS_TO_TICKS(1000)) != pdTRUE) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        while (chunk++ < REBUILD_CHUNK_ROWS && (uint64_t)ftell(f) < snapshot_end && fgets(line, sizeof(line), f)) {
            long offset = ftell(f) - (long)strlen(line);
            uint32_t id = 0;
            if (!parse_line_id(line, &id) || !sparse_point_id(id)) continue;
            if (count == capacity) {
                uint32_t next = capacity ? capacity * 2U : 32U;
                sd_index_entry_t *grown = realloc(rebuilt, next * sizeof(*grown));
                if (!grown) { if (g_lock) xSemaphoreGive(g_lock); goto close_file; }
                rebuilt = grown; capacity = next;
            }
            sd_index_entry_t *entry = &rebuilt[count++];
            memset(entry, 0, sizeof(*entry)); entry->measurement_id = id; entry->csv_offset = (uint64_t)offset;
            entry->entry_crc = crc32_update(0, entry, offsetof(sd_index_entry_t, entry_crc));
        }
        bool finished = feof(f) || (uint64_t)ftell(f) >= snapshot_end;
        if (g_lock) xSemaphoreGive(g_lock);
        if (finished) { rebuild_complete = true; break; }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
close_file:
    fclose(f);
    if (token != g_rebuild_token || !rebuild_complete) goto done;
    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    if (token != g_rebuild_token) { if (g_lock) xSemaphoreGive(g_lock); goto done; }
    bool merge_complete = true;
    for (uint32_t i = 0; i < g_index_count; ++i) {
        if (g_index_entries[i].measurement_id <= snapshot_last_id) continue;
        if (count == capacity) {
            uint32_t next = capacity ? capacity * 2U : 32U;
            sd_index_entry_t *grown = realloc(rebuilt, next * sizeof(*grown));
            if (!grown) { merge_complete = false; break; }
            rebuilt = grown; capacity = next;
        }
        rebuilt[count++] = g_index_entries[i];
    }
    if (!merge_complete) { if (g_lock) xSemaphoreGive(g_lock); goto done; }
    free(g_index_entries);
    g_index_entries = rebuilt; rebuilt = NULL;
    g_index_count = count; g_index_capacity = capacity;
    g_index_ready = persist_index_locked() && load_index();
    g_index_rebuilding = false;
    if (g_checkpoint_valid) write_checkpoint_locked(&g_checkpoint);
    if (g_lock) xSemaphoreGive(g_lock);
    ESP_LOGI(TAG, "Indice disperso reconstruido: puntos=%lu intervalo=%u", (unsigned long)g_index_count, INDEX_INTERVAL);
done:
    free(rebuilt);
    if (token == g_rebuild_token) g_index_rebuilding = false;
    vTaskDelete(NULL);
}

static void start_index_rebuild(void) {
    if (g_index_rebuilding) return;
    g_index_rebuilding = true;
    uint32_t token = ++g_rebuild_token;
    if (xTaskCreate(index_rebuild_task, "sd_index_rebuild", 4096, (void *)(uintptr_t)token, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        g_index_rebuilding = false;
        ESP_LOGW(TAG, "No hay memoria para iniciar reconstruccion del indice");
    }
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
    uint64_t actual_size = 0;
    if (!csv_size(&actual_size)) return ESP_FAIL;
    sd_checkpoint_t state_a = {0}, state_b = {0};
    bool valid_a = read_checkpoint(STATE_A_PATH, &state_a) && validate_checkpoint_row(&state_a, actual_size);
    bool valid_b = read_checkpoint(STATE_B_PATH, &state_b) && validate_checkpoint_row(&state_b, actual_size);
    sd_checkpoint_t recovered = {0};
    bool recovered_ok = false;
    bool checkpoint_needs_write = !(valid_a || valid_b);
    if (valid_a || valid_b) {
        recovered = (!valid_b || (valid_a && state_a.generation >= state_b.generation)) ? state_a : state_b;
        checkpoint_needs_write = recovered.csv_size != actual_size;
        recovered_ok = recover_checkpoint_tail(&recovered, actual_size);
        ESP_LOGI(TAG, "Checkpoint seleccionado: generacion=%llu ultimo_id=%lu cola=%llu bytes",
                 (unsigned long long)recovered.generation, (unsigned long)recovered.last_id,
                 (unsigned long long)(actual_size - recovered.confirmed_end_offset));
    } else {
        ESP_LOGW(TAG, "Sin checkpoint utilizable; recuperando solo la ultima fila y reconstruyendo indice en segundo plano");
        recovered_ok = recover_last_complete_row(&recovered);
    }
    if (!recovered_ok) {
        ESP_LOGE(TAG, "No se pudo determinar un ultimo ID seguro; SD no se habilita para evitar duplicados");
        return ESP_ERR_INVALID_STATE;
    }
    g_checkpoint = recovered;
    g_checkpoint_valid = valid_a || valid_b;
    g_last_id = recovered.last_id;
    if (checkpoint_needs_write) write_checkpoint_locked(&recovered);
    g_index_ready = load_index();
    g_ready = true;
    if (!g_index_ready) start_index_rebuild();
    ESP_LOGI(TAG, "SD lista en %s, ultimo id=%lu checkpoint=%s indice=%s; sensores no esperan reconstruccion",
             MOUNT_POINT, (unsigned long)g_last_id, g_checkpoint_valid ? "valido" : "pendiente",
             g_index_ready ? "listo" : "reconstruyendo");
    return ESP_OK;
}

bool sd_store_is_ready(void) {
    return g_ready;
}

uint32_t sd_store_last_id(void) {
    return g_last_id;
}

bool sd_store_checkpoint_valid(void) { return g_checkpoint_valid; }
uint64_t sd_store_checkpoint_generation(void) { return g_checkpoint_valid ? g_checkpoint.generation : 0; }
bool sd_store_history_index_ready(void) { return g_index_ready; }
bool sd_store_history_index_rebuilding(void) { return g_index_rebuilding; }
uint32_t sd_store_history_index_points(void) { return g_index_count; }
uint32_t sd_store_format_version(void) { return CSV_FORMAT_VERSION; }

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
    g_rebuild_token++;
    g_index_rebuilding = false;
    g_last_id = 0;
    g_index_count = 0;
    g_index_ready = false;
    unlink(STATE_A_PATH);
    unlink(STATE_B_PATH);
    unlink(INDEX_PATH);
    unlink(INDEX_TMP_PATH);
    unlink(INDEX_BAK_PATH);
    sd_checkpoint_t empty = {0};
    uint64_t empty_csv_size = 0;
    csv_size(&empty_csv_size);
    empty.csv_size = empty_csv_size;
    empty.confirmed_end_offset = empty_csv_size;
    g_checkpoint_valid = false;
    if (!write_checkpoint_locked(&empty) || !persist_index_locked()) {
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }
    g_index_ready = true;
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
    uint32_t id = g_last_id + 1U;
    ESP_LOGI(TAG,
             "Guardando medicion SD id=%lu boot_id=%lu uptime_s=%lu time_valid=%s timestamp=%s co2=%u pm2.5=%.2f voc=%.2f nox=%.2f temp=%.2f hum=%.2f scd=%.2f/%.2f sen=%.2f/%.2f gps=%s %.6f/%.6f window_s=%lu",
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
             reading->gps_valid ? "true" : "false",
             reading->gps_valid ? reading->gps_lat : 0.0,
             reading->gps_valid ? reading->gps_lon : 0.0,
             (unsigned long)reading->window_s);

    FILE *f = fopen(CSV_PATH, "a");
    if (!f) {
        ESP_LOGE(TAG, "No se pudo abrir CSV para append: %s errno=%d (%s)", CSV_PATH, errno, strerror(errno));
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }
    long row_offset = ftell(f);
    char written_row[512];
    int written = snprintf(written_row, sizeof(written_row),
                           "%lu,%lu,%lu,%u,%s,%s,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%.6f,%.6f,%u,%.2f,%lu,%lu\n",
                          (unsigned long)id,
                          (unsigned long)reading->boot_id,
                          (unsigned long)reading->uptime_s,
                          reading->time_valid ? 1U : 0U,
                          reading->time_source[0] ? reading->time_source : "none",
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
                          reading->gps_valid ? 1U : 0U,
                          reading->gps_valid ? reading->gps_lat : 0.0,
                          reading->gps_valid ? reading->gps_lon : 0.0,
                          reading->gps_satellites,
                          reading->gps_hdop,
                          (unsigned long)reading->gps_age_ms,
                           (unsigned long)reading->window_s);
    if (written <= 0 || written >= (int)sizeof(written_row) || fwrite(written_row, 1, (size_t)written, f) != (size_t)written) {
        ESP_LOGE(TAG, "Fallo al serializar/escribir medicion id=%lu: errno=%d (%s)", (unsigned long)id, errno, strerror(errno));
        fclose(f);
        if (!truncate_csv((uint64_t)row_offset)) g_ready = false;
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    if (!sync_file(f)) {
        ESP_LOGE(TAG, "No se pudo sincronizar medicion id=%lu: errno=%d (%s)", (unsigned long)id, errno, strerror(errno));
        fclose(f);
        if (!truncate_csv((uint64_t)row_offset)) g_ready = false;
        if (g_lock) xSemaphoreGive(g_lock);
        return ESP_FAIL;
    }

    if (fclose(f) != 0) {
        ESP_LOGW(TAG, "fclose reporto error despues de fsync para id=%lu: errno=%d (%s)", (unsigned long)id, errno, strerror(errno));
    }

    uint64_t row_end = (uint64_t)row_offset + (uint64_t)written;
    g_last_id = id;
    if (sparse_point_id(id) && (!add_index_entry_ram(id, (uint64_t)row_offset) || !persist_index_locked())) {
        g_index_ready = false;
        ESP_LOGW(TAG, "Fila confirmada, pero indice pendiente de reconstruccion para id=%lu", (unsigned long)id);
    }
    sd_checkpoint_t next = g_checkpoint;
    next.last_id = id;
    next.row_count = g_checkpoint.row_count + 1U;
    next.csv_size = row_end;
    next.last_row_offset = (uint64_t)row_offset;
    next.confirmed_end_offset = row_end;
    next.last_row_crc = crc32_update(0, written_row, strlen(written_row));
    if (!write_checkpoint_locked(&next)) {
        g_checkpoint = next;
        ESP_LOGW(TAG, "Fila id=%lu confirmada en CSV; checkpoint quedo atrasado y se recuperara al reiniciar", (unsigned long)id);
    }

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
    char time_source[16];
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
    bool gps_valid;
    double gps_lat;
    double gps_lon;
    uint32_t gps_satellites;
    float gps_hdop;
    uint32_t gps_age_ms;
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

    char copy[512];
    snprintf(copy, sizeof(copy), "%s", line);

    char *fields[30] = {0};
    int n = split_csv_simple(copy, fields, 30);
    memset(row, 0, sizeof(*row));

    // Formato UTC con fuente horaria y GPS.
    if (n >= 26 && parse_u32_field(fields[0], &row->id) && parse_u32_field(fields[1], &row->boot_id)) {
        uint32_t time_valid = 0;
        uint32_t gps_valid = 0;
        bool ok = parse_u32_field(fields[2], &row->uptime_s) &&
               parse_u32_field(fields[3], &time_valid) &&
               (row->time_valid = time_valid != 0, true) &&
               (snprintf(row->time_source, sizeof(row->time_source), "%s", fields[4] ? fields[4] : "none"), true) &&
               (snprintf(row->timestamp, sizeof(row->timestamp), "%s", fields[5] ? fields[5] : ""), true) &&
               parse_u32_field(fields[6], &row->co2) &&
               parse_float_field(fields[7], &row->pm1p0) && parse_float_field(fields[8], &row->pm2p5) &&
               parse_float_field(fields[9], &row->pm4p0) && parse_float_field(fields[10], &row->pm10p0) &&
               parse_float_field(fields[11], &row->voc) && parse_float_field(fields[12], &row->nox) &&
               parse_float_field(fields[13], &row->temp) && parse_float_field(fields[14], &row->hum) &&
               parse_float_field(fields[15], &row->scd_temp) && parse_float_field(fields[16], &row->scd_hum) &&
               parse_float_field(fields[17], &row->sen_temp) && parse_float_field(fields[18], &row->sen_hum) &&
               parse_u32_field(fields[19], &gps_valid) &&
               (row->gps_lat = strtod(fields[20] ? fields[20] : "0", NULL), true) &&
               (row->gps_lon = strtod(fields[21] ? fields[21] : "0", NULL), true) &&
               parse_u32_field(fields[22], &row->gps_satellites) && parse_float_field(fields[23], &row->gps_hdop) &&
               parse_u32_field(fields[24], &row->gps_age_ms) && parse_u32_field(fields[25], &row->window_s);
        row->gps_valid = gps_valid != 0;
        return ok;
    }

    // Formato nuevo con GPS:
    // id,boot_id,uptime_s,time_valid,timestamp,co2,pm1p0,pm2p5,pm4p0,pm10p0,voc,nox,temp,hum,scd_temp,scd_hum,sen_temp,sen_hum,gps_valid,gps_lat,gps_lon,gps_satellites,gps_hdop,gps_age_ms,window_s
    if (n >= 25 && parse_u32_field(fields[0], &row->id) && parse_u32_field(fields[1], &row->boot_id)) {
        uint32_t time_valid = 0;
        uint32_t gps_valid = 0;
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
               parse_float_field(fields[14], &row->scd_temp) &&
               parse_float_field(fields[15], &row->scd_hum) &&
               parse_float_field(fields[16], &row->sen_temp) &&
               parse_float_field(fields[17], &row->sen_hum) &&
               parse_u32_field(fields[18], &gps_valid) &&
               (row->gps_lat = strtod(fields[19] ? fields[19] : "0", NULL), true) &&
               (row->gps_lon = strtod(fields[20] ? fields[20] : "0", NULL), true) &&
               parse_u32_field(fields[21], &row->gps_satellites) &&
               parse_float_field(fields[22], &row->gps_hdop) &&
               parse_u32_field(fields[23], &row->gps_age_ms) &&
               parse_u32_field(fields[24], &row->window_s);
        row->gps_valid = gps_valid != 0;
        snprintf(row->time_source, sizeof(row->time_source), "%s", row->time_valid ? "esp" : "uptime");
        return ok;
    }

    // Formato nuevo extendido sin GPS:
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
    cJSON_AddStringToObject(row, "time_source",
                            parsed->time_source[0] ? parsed->time_source : (parsed->time_valid ? "esp" : "uptime"));
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
    cJSON_AddBoolToObject(row, "gps_valid", parsed->gps_valid);
    if (parsed->gps_valid) {
        cJSON_AddNumberToObject(row, "gps_lat", parsed->gps_lat);
        cJSON_AddNumberToObject(row, "gps_lon", parsed->gps_lon);
        cJSON_AddNumberToObject(row, "gps_satellites", parsed->gps_satellites);
        cJSON_AddNumberToObject(row, "gps_hdop", parsed->gps_hdop);
        cJSON_AddNumberToObject(row, "gps_age_ms", parsed->gps_age_ms);
    } else {
        cJSON_AddNullToObject(row, "gps_lat");
        cJSON_AddNullToObject(row, "gps_lon");
        cJSON_AddNumberToObject(row, "gps_satellites", 0);
        cJSON_AddNumberToObject(row, "gps_hdop", 0);
        cJSON_AddNumberToObject(row, "gps_age_ms", 0);
    }
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

    char gps_lat_field[32];
    char gps_lon_field[32];
    if (parsed->gps_valid) {
        snprintf(gps_lat_field, sizeof(gps_lat_field), "%.6f", parsed->gps_lat);
        snprintf(gps_lon_field, sizeof(gps_lon_field), "%.6f", parsed->gps_lon);
    } else {
        snprintf(gps_lat_field, sizeof(gps_lat_field), "null");
        snprintf(gps_lon_field, sizeof(gps_lon_field), "null");
    }

    char line[768];
    int len = snprintf(
        line,
        sizeof(line),
        "{\"id\":%lu,\"measurement_id\":%lu,\"boot_id\":%lu,\"uptime_s\":%lu,"
        "\"time_valid\":%s,\"time_source\":\"%s\",\"timestamp\":%s,"
        "\"co2\":%lu,\"pm1p0\":%.2f,\"pm2p5\":%.2f,\"pm4p0\":%.2f,\"pm10p0\":%.2f,"
        "\"voc\":%.2f,\"nox\":%.2f,\"temp\":%.2f,\"hum\":%.2f,"
        "\"scd_temp\":%.2f,\"scd_hum\":%.2f,\"sen_temp\":%.2f,\"sen_hum\":%.2f,"
        "\"gps_valid\":%s,\"gps_lat\":%s,\"gps_lon\":%s,"
        "\"gps_satellites\":%lu,\"gps_hdop\":%.2f,\"gps_age_ms\":%lu,\"window_s\":%lu}\n",
        (unsigned long)parsed->id,
        (unsigned long)parsed->id,
        (unsigned long)parsed->boot_id,
        (unsigned long)parsed->uptime_s,
        parsed->time_valid ? "true" : "false",
        parsed->time_source[0] ? parsed->time_source : (parsed->time_valid ? "esp" : "uptime"),
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
        parsed->gps_valid ? "true" : "false",
        gps_lat_field,
        gps_lon_field,
        (unsigned long)parsed->gps_satellites,
        parsed->gps_hdop,
        (unsigned long)parsed->gps_age_ms,
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

    char line[512];
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

    long start_offset = get_id_offset(after_id + 1U);
    if (start_offset >= 0 && fseek(f, start_offset, SEEK_SET) != 0) rewind(f);

    char line[512];
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

    char line[512];
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

        char line[512];
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

    uint32_t recent_end = before_id > 0 ? before_id - 1U : g_last_id;
    uint32_t recent_start = recent_end > (limit + INDEX_INTERVAL) ? recent_end - (limit + INDEX_INTERVAL) : 1U;
    long recent_offset = get_id_offset(recent_start);
    if (recent_offset >= 0 && fseek(f, recent_offset, SEEK_SET) != 0) rewind(f);

    char line[512];
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
