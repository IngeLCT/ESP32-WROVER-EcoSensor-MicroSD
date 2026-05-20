#include "ota_manager.h"

#include "esp_app_desc.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ota_manager";

#define OTA_TASK_STACK 8192
#define OTA_TASK_PRIO 5
#define OTA_READ_BUF_SIZE 4096

typedef struct {
    ota_manager_state_t state;
    char target_version[OTA_MANAGER_VERSION_MAX_LEN];
    char firmware_url[OTA_MANAGER_URL_MAX_LEN];
    char sha256[OTA_MANAGER_SHA256_MAX_LEN];
    char last_error[OTA_MANAGER_ERROR_MAX_LEN];
    int bytes_received;
    int total_bytes;
} ota_status_t;

static ota_status_t g_status = {
    .state = OTA_MANAGER_STATE_IDLE,
    .total_bytes = -1,
};
static SemaphoreHandle_t g_lock = NULL;

static void set_error_locked(const char *error) {
    g_status.state = OTA_MANAGER_STATE_ERROR;
    snprintf(g_status.last_error, sizeof(g_status.last_error), "%s", error ? error : "unknown_error");
}

static void set_state(ota_manager_state_t state) {
    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    g_status.state = state;
    if (g_lock) xSemaphoreGive(g_lock);
}

static void set_error(const char *error) {
    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    set_error_locked(error);
    if (g_lock) xSemaphoreGive(g_lock);
    ESP_LOGE(TAG, "OTA error: %s", error ? error : "unknown_error");
}

static void set_progress(int bytes_received, int total_bytes) {
    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    g_status.bytes_received = bytes_received;
    if (total_bytes >= 0) {
        g_status.total_bytes = total_bytes;
    }
    if (g_lock) xSemaphoreGive(g_lock);
}

const char *ota_manager_current_version(void) {
    const esp_app_desc_t *desc = esp_app_get_description();
    return desc ? desc->version : "unknown";
}

const char *ota_manager_state_name(ota_manager_state_t state) {
    switch (state) {
        case OTA_MANAGER_STATE_IDLE: return "idle";
        case OTA_MANAGER_STATE_QUEUED: return "queued";
        case OTA_MANAGER_STATE_DOWNLOADING: return "downloading";
        case OTA_MANAGER_STATE_WRITING: return "writing";
        case OTA_MANAGER_STATE_SUCCESS: return "success";
        case OTA_MANAGER_STATE_ERROR: return "error";
        case OTA_MANAGER_STATE_REBOOTING: return "rebooting";
        default: return "unknown";
    }
}

bool ota_manager_busy(void) {
    bool busy = false;
    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    busy = g_status.state == OTA_MANAGER_STATE_QUEUED ||
           g_status.state == OTA_MANAGER_STATE_DOWNLOADING ||
           g_status.state == OTA_MANAGER_STATE_WRITING ||
           g_status.state == OTA_MANAGER_STATE_REBOOTING;
    if (g_lock) xSemaphoreGive(g_lock);
    return busy;
}

static void ota_task(void *arg) {
    char url[OTA_MANAGER_URL_MAX_LEN];
    char target_version[OTA_MANAGER_VERSION_MAX_LEN];
    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    snprintf(url, sizeof(url), "%s", g_status.firmware_url);
    snprintf(target_version, sizeof(target_version), "%s", g_status.target_version);
    g_status.bytes_received = 0;
    g_status.total_bytes = -1;
    g_status.last_error[0] = '\0';
    if (g_lock) xSemaphoreGive(g_lock);

    ESP_LOGI(TAG, "OTA iniciada: version=%s url=%s", target_version, url);

    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 10000,
        .buffer_size = OTA_READ_BUF_SIZE,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        set_error("http_client_init_failed");
        vTaskDelete(NULL);
        return;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        char msg[OTA_MANAGER_ERROR_MAX_LEN];
        snprintf(msg, sizeof(msg), "http_open_failed:%s", esp_err_to_name(err));
        set_error(msg);
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    int total = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status < 200 || status >= 300) {
        char msg[OTA_MANAGER_ERROR_MAX_LEN];
        snprintf(msg, sizeof(msg), "http_status_%d", status);
        set_error(msg);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }
    set_progress(0, total);
    ESP_LOGI(TAG, "OTA descarga aceptada: http=%d total=%d", status, total);

    const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    if (!partition) {
        set_error("no_update_partition");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    esp_ota_handle_t ota_handle = 0;
    err = esp_ota_begin(partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        char msg[OTA_MANAGER_ERROR_MAX_LEN];
        snprintf(msg, sizeof(msg), "ota_begin_failed:%s", esp_err_to_name(err));
        set_error(msg);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    char *buffer = malloc(OTA_READ_BUF_SIZE);
    if (!buffer) {
        set_error("no_memory");
        esp_ota_abort(ota_handle);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    int received = 0;
    set_state(OTA_MANAGER_STATE_DOWNLOADING);
    while (true) {
        int read = esp_http_client_read(client, buffer, OTA_READ_BUF_SIZE);
        if (read < 0) {
            set_error("http_read_failed");
            esp_ota_abort(ota_handle);
            free(buffer);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            vTaskDelete(NULL);
            return;
        }
        if (read == 0) {
            break;
        }
        set_state(OTA_MANAGER_STATE_WRITING);
        err = esp_ota_write(ota_handle, buffer, read);
        if (err != ESP_OK) {
            char msg[OTA_MANAGER_ERROR_MAX_LEN];
            snprintf(msg, sizeof(msg), "ota_write_failed:%s", esp_err_to_name(err));
            set_error(msg);
            esp_ota_abort(ota_handle);
            free(buffer);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            vTaskDelete(NULL);
            return;
        }
        received += read;
        set_progress(received, total);
        if (total > 0 && (received == total || received % (64 * 1024) < OTA_READ_BUF_SIZE)) {
            ESP_LOGI(TAG, "OTA progreso: %d/%d bytes", received, total);
        }
        set_state(OTA_MANAGER_STATE_DOWNLOADING);
    }

    free(buffer);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        char msg[OTA_MANAGER_ERROR_MAX_LEN];
        snprintf(msg, sizeof(msg), "ota_end_failed:%s", esp_err_to_name(err));
        set_error(msg);
        vTaskDelete(NULL);
        return;
    }

    err = esp_ota_set_boot_partition(partition);
    if (err != ESP_OK) {
        char msg[OTA_MANAGER_ERROR_MAX_LEN];
        snprintf(msg, sizeof(msg), "set_boot_partition_failed:%s", esp_err_to_name(err));
        set_error(msg);
        vTaskDelete(NULL);
        return;
    }

    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    g_status.state = OTA_MANAGER_STATE_SUCCESS;
    g_status.bytes_received = received;
    g_status.total_bytes = total;
    g_status.last_error[0] = '\0';
    if (g_lock) xSemaphoreGive(g_lock);
    ESP_LOGI(TAG, "OTA finalizada OK: version=%s bytes=%d. Reiniciando...", target_version, received);

    vTaskDelay(pdMS_TO_TICKS(800));
    set_state(OTA_MANAGER_STATE_REBOOTING);
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

esp_err_t ota_manager_init(void) {
    if (!g_lock) {
        g_lock = xSemaphoreCreateMutex();
        if (!g_lock) {
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

esp_err_t ota_manager_start(const char *device_id, const char *target_version, const char *firmware_url, const char *sha256) {
    (void)device_id;
    if (!target_version || !target_version[0] || !firmware_url || !firmware_url[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strncmp(firmware_url, "http://", 7) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(target_version) >= OTA_MANAGER_VERSION_MAX_LEN || strlen(firmware_url) >= OTA_MANAGER_URL_MAX_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (sha256 && strlen(sha256) >= OTA_MANAGER_SHA256_MAX_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (ota_manager_busy()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    g_status.state = OTA_MANAGER_STATE_QUEUED;
    snprintf(g_status.target_version, sizeof(g_status.target_version), "%s", target_version);
    snprintf(g_status.firmware_url, sizeof(g_status.firmware_url), "%s", firmware_url);
    snprintf(g_status.sha256, sizeof(g_status.sha256), "%s", sha256 ? sha256 : "");
    g_status.last_error[0] = '\0';
    g_status.bytes_received = 0;
    g_status.total_bytes = -1;
    if (g_lock) xSemaphoreGive(g_lock);

    BaseType_t created = xTaskCreate(ota_task, "ota_task", OTA_TASK_STACK, NULL, OTA_TASK_PRIO, NULL);
    if (created != pdPASS) {
        set_error("task_create_failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void ota_manager_add_status_json(cJSON *root) {
    if (!root) {
        return;
    }
    if (g_lock) xSemaphoreTake(g_lock, portMAX_DELAY);
    ota_manager_state_t state = g_status.state;
    int bytes = g_status.bytes_received;
    int total = g_status.total_bytes;
    cJSON_AddBoolToObject(root, "ok", state != OTA_MANAGER_STATE_ERROR);
    cJSON_AddStringToObject(root, "state", ota_manager_state_name(state));
    cJSON_AddStringToObject(root, "current_version", ota_manager_current_version());
    if (g_status.target_version[0]) {
        cJSON_AddStringToObject(root, "target_version", g_status.target_version);
    } else {
        cJSON_AddNullToObject(root, "target_version");
    }
    cJSON_AddNumberToObject(root, "bytes_received", bytes);
    cJSON_AddNumberToObject(root, "total_bytes", total);
    if (total > 0) {
        cJSON_AddNumberToObject(root, "progress_pct", (bytes * 100.0) / total);
    } else {
        cJSON_AddNullToObject(root, "progress_pct");
    }
    if (g_status.last_error[0]) {
        cJSON_AddStringToObject(root, "last_error", g_status.last_error);
    } else {
        cJSON_AddNullToObject(root, "last_error");
    }
    if (g_status.sha256[0]) {
        cJSON_AddStringToObject(root, "sha256", g_status.sha256);
    }
    if (g_lock) xSemaphoreGive(g_lock);
}
