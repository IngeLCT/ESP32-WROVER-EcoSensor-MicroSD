#include "captive_manager.h"
#include "wifi_store.h"
#include "sd_store.h"
#include "ota_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mdns.h"
#include "lwip/ip4_addr.h"
#include "esp_sntp.h"

#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <ctype.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <errno.h>
#include <dirent.h>

static const char *TAG = "wifi_manager";

static captive_manager_cfg_t g_cfg;
static captive_state_t g_state = CAP_STATE_IDLE;
static httpd_handle_t g_server = NULL;
static esp_netif_t *g_ap_netif = NULL;
static esp_netif_t *g_sta_netif = NULL;

static bool g_sta_have_ip = false;
static bool g_using_saved = false;
static bool g_saved_apsta_mode = false;
static bool g_sensors_started = false;
static TaskHandle_t g_saved_reconnect_task = NULL;
static bool g_time_valid = false;
static uint32_t g_boot_id = 0;
static char g_config_date[11] = {0};
static char g_config_time[9] = {0};
static char g_last_sync_source[16] = "none";
static char g_push_host[64] = {0};
static int  g_connect_attempts = 0;
static int64_t g_boot_time_ms = 0;
static captive_manager_readings_t g_last_readings = {0};
static bool g_sntp_started = false;

#define MIN_VALID_EPOCH 1704067200LL /* 2024-01-01T00:00:00Z */
#define TIME_ADJUST_THRESHOLD_SECONDS 10

static void restart_later_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

static void set_state(captive_state_t st);
static esp_err_t start_ap(void);
static esp_err_t start_apsta_with_saved_credentials(void);
static esp_err_t start_http(void);
static void scan_start(void);
static void connect_sta(const char *ssid, const char *pass, bool from_saved);
static void shutdown_ap_http(void);
static void start_mdns_service(void);
static bool in_boot_grace_window(void);
static void start_with_saved_or_manager(void);
static void ensure_saved_reconnect_task(void);
static esp_err_t parse_ipv4_or_fail(const char *text, esp_ip4_addr_t *out);
static esp_err_t apply_saved_sta_ip_config(void);
static bool validate_date_time(const char *date, const char *time_text);
static esp_err_t apply_device_time(const char *date, const char *time_text);
static void load_saved_device_time(void);
static void get_active_ip_string(char *buf, size_t buf_size);
static void get_current_datetime_string(char *buf, size_t buf_size);
static void start_sntp_if_needed(void);
static bool parse_utc_timestamp(const char *timestamp, time_t *epoch_out);

static time_t utc_fields_to_epoch(int year, int month, int day, int hour, int minute, int second) {
    int adjusted_year = year - (month <= 2 ? 1 : 0);
    int era = (adjusted_year >= 0 ? adjusted_year : adjusted_year - 399) / 400;
    unsigned year_of_era = (unsigned)(adjusted_year - era * 400);
    unsigned month_prime = (unsigned)(month + (month > 2 ? -3 : 9));
    unsigned day_of_year = (153U * month_prime + 2U) / 5U + (unsigned)day - 1U;
    unsigned day_of_era = year_of_era * 365U + year_of_era / 4U - year_of_era / 100U + day_of_year;
    int64_t days = (int64_t)era * 146097LL + (int64_t)day_of_era - 719468LL;
    return (time_t)(days * 86400LL + hour * 3600LL + minute * 60LL + second);
}

const char* captive_manager_state_str(captive_state_t st) {
    switch(st){
        case CAP_STATE_IDLE: return "IDLE";
        case CAP_STATE_PREP: return "PREP";
        case CAP_STATE_SCAN: return "SCAN";
        case CAP_STATE_CONNECTING: return "CONNECTING";
        case CAP_STATE_WAIT_LOGIN: return "WAIT_LOGIN";
        case CAP_STATE_VERIFY: return "VERIFY";
        case CAP_STATE_OPERATIONAL: return "OPERATIONAL";
        case CAP_STATE_RECAPTIVE: return "RECAPTIVE";
        case CAP_STATE_APSTA_OFFLINE: return "APSTA_OFFLINE";
        default: return "UNKNOWN";
    }
}

captive_state_t captive_manager_get_state(void){
    return g_state;
}

static void set_state(captive_state_t st) {
    if (g_state != st) {
        ESP_LOGI(TAG, "STATE: %s -> %s", captive_manager_state_str(g_state), captive_manager_state_str(st));
        g_state = st;
    }
}

bool captive_manager_using_saved(void) {
    return g_using_saved;
}

void captive_manager_set_sensors_started(bool started) {
    g_sensors_started = started;
}

void captive_manager_set_last_readings(const captive_manager_readings_t *readings) {
    if (!readings) {
        memset(&g_last_readings, 0, sizeof(g_last_readings));
        return;
    }
    g_last_readings = *readings;
}


bool captive_manager_time_is_valid(void) {
    return g_time_valid;
}

const char *captive_manager_time_source(void) {
    return g_time_valid ? g_last_sync_source : "none";
}

time_t captive_manager_current_epoch(void) {
    return g_time_valid ? time(NULL) : 0;
}

static int time_source_priority(const char *source) {
    if (!source) return 0;
    if (strcmp(source, "gps") == 0) return 3;
    if (strcmp(source, "ntp") == 0) return 2;
    if (strcmp(source, "server") == 0) return 1;
    return 0;
}

esp_err_t captive_manager_offer_time_epoch(time_t epoch_utc, const char *source) {
    if (epoch_utc < (time_t)MIN_VALID_EPOCH || time_source_priority(source) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int incoming_priority = time_source_priority(source);
    int current_priority = time_source_priority(g_last_sync_source);
    time_t current = time(NULL);
    long long drift = g_time_valid ? llabs((long long)epoch_utc - (long long)current) : 0;

    /* El servidor es solo respaldo: nunca reemplaza GPS/NTP validos. Una
       fuente de igual o mayor calidad solo ajusta si el desfase es material. */
    if (g_time_valid && incoming_priority < current_priority) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!g_time_valid || drift > TIME_ADJUST_THRESHOLD_SECONDS) {
        struct timeval tv = {.tv_sec = epoch_utc, .tv_usec = 0};
        if (settimeofday(&tv, NULL) != 0) {
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Reloj UTC ajustado por %s; epoch=%lld desfase=%lld s",
                 source, (long long)epoch_utc, drift);
    }

    g_time_valid = true;
    snprintf(g_last_sync_source, sizeof(g_last_sync_source), "%s", source);
    struct tm utc_tm = {0};
    gmtime_r(&epoch_utc, &utc_tm);
    strftime(g_config_date, sizeof(g_config_date), "%d-%m-%Y", &utc_tm);
    strftime(g_config_time, sizeof(g_config_time), "%H:%M:%S", &utc_tm);
    return ESP_OK;
}

static void sntp_time_sync_cb(struct timeval *tv) {
    time_t epoch = tv ? tv->tv_sec : time(NULL);
    esp_err_t err = captive_manager_offer_time_epoch(epoch, "ntp");
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Sincronizacion NTP recibida pero no aceptada: %s", esp_err_to_name(err));
    }
}

static void start_sntp_if_needed(void) {
    if (g_sntp_started) return;
    g_sntp_started = true;
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(sntp_time_sync_cb);
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP iniciado en segundo plano");
}

bool captive_manager_can_measure(void) {
    return g_sta_have_ip || g_saved_apsta_mode;
}

bool captive_manager_can_push_measurements(void) {
    return g_sta_have_ip && g_state == CAP_STATE_OPERATIONAL;
}

const char *captive_manager_push_host(void) {
    return g_push_host;
}

uint32_t captive_manager_boot_id(void) {
    return g_boot_id;
}

static bool validate_date_time(const char *date, const char *time_text) {
    if (!date || !time_text || strlen(date) != 10 || strlen(time_text) != 8) return false;
    if (date[2] != '-' || date[5] != '-' || time_text[2] != ':' || time_text[5] != ':') return false;
    for (int i = 0; i < 10; i++) {
        if (i == 2 || i == 5) continue;
        if (!isdigit((unsigned char)date[i])) return false;
    }
    for (int i = 0; i < 8; i++) {
        if (i == 2 || i == 5) continue;
        if (!isdigit((unsigned char)time_text[i])) return false;
    }

    int day = (date[0] - '0') * 10 + (date[1] - '0');
    int month = (date[3] - '0') * 10 + (date[4] - '0');
    int year = (date[6] - '0') * 1000 + (date[7] - '0') * 100 + (date[8] - '0') * 10 + (date[9] - '0');
    int hour = (time_text[0] - '0') * 10 + (time_text[1] - '0');
    int minute = (time_text[3] - '0') * 10 + (time_text[4] - '0');
    int second = (time_text[6] - '0') * 10 + (time_text[7] - '0');

    if (year < 2024 || month < 1 || month > 12 || day < 1 || day > 31) return false;
    if (hour > 23 || minute > 59 || second > 59) return false;

    static const int days_in_month[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
    int max_day = days_in_month[month];
    bool leap = ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
    if (month == 2 && leap) max_day = 29;
    return day <= max_day;
}

static esp_err_t apply_device_time(const char *date, const char *time_text) {
    if (!validate_date_time(date, time_text)) return ESP_ERR_INVALID_ARG;

    struct tm tm_value = {0};
    tm_value.tm_mday = (date[0] - '0') * 10 + (date[1] - '0');
    tm_value.tm_mon = ((date[3] - '0') * 10 + (date[4] - '0')) - 1;
    tm_value.tm_year = ((date[6] - '0') * 1000 + (date[7] - '0') * 100 + (date[8] - '0') * 10 + (date[9] - '0')) - 1900;
    tm_value.tm_hour = (time_text[0] - '0') * 10 + (time_text[1] - '0');
    tm_value.tm_min = (time_text[3] - '0') * 10 + (time_text[4] - '0');
    tm_value.tm_sec = (time_text[6] - '0') * 10 + (time_text[7] - '0');
    tm_value.tm_isdst = 0;

    time_t epoch = utc_fields_to_epoch(tm_value.tm_year + 1900, tm_value.tm_mon + 1, tm_value.tm_mday,
                                       tm_value.tm_hour, tm_value.tm_min, tm_value.tm_sec);
    if (epoch < 0) return ESP_ERR_INVALID_ARG;

    esp_err_t offer_err = captive_manager_offer_time_epoch(epoch, "server");
    if (offer_err != ESP_OK) return offer_err;

    snprintf(g_config_date, sizeof(g_config_date), "%s", date);
    snprintf(g_config_time, sizeof(g_config_time), "%s", time_text);
    device_time_cfg_t cfg = {0};
    cfg.valid = true;
    snprintf(cfg.date, sizeof(cfg.date), "%s", date);
    snprintf(cfg.time, sizeof(cfg.time), "%s", time_text);
    return wifi_store_save_device_time(&cfg);
}

static void load_saved_device_time(void) {
    device_time_cfg_t cfg = {0};
    esp_err_t err = wifi_store_load_device_time(&cfg);
    if (err == ESP_OK && cfg.valid && validate_date_time(cfg.date, cfg.time)) {
        snprintf(g_config_date, sizeof(g_config_date), "%s", cfg.date);
        snprintf(g_config_time, sizeof(g_config_time), "%s", cfg.time);
        ESP_LOGI(TAG, "Ultima fecha/hora guardada en NVS: %s %s (no se considera valida tras reinicio)", cfg.date, cfg.time);
    } else {
        g_config_date[0] = 0;
        g_config_time[0] = 0;
    }

    g_time_valid = false;
    snprintf(g_last_sync_source, sizeof(g_last_sync_source), "none");
    ESP_LOGI(TAG, "Fecha/hora requiere sincronizacion del servidor; mediciones offline usaran uptime hasta sincronizar");
}

esp_err_t captive_manager_init(const captive_manager_cfg_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    g_cfg = *cfg;
    setenv("TZ", "CST6", 1);
    tzset();
    g_boot_id = esp_random();
    if (g_boot_id == 0) {
        g_boot_id = 1;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    load_saved_device_time();
    ESP_ERROR_CHECK(ota_manager_init());

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    g_ap_netif  = esp_netif_create_default_wifi_ap();
    g_sta_netif = esp_netif_create_default_wifi_sta();
    (void)g_ap_netif;
    (void)g_sta_netif;

    wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wic));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());

    g_boot_time_ms = esp_timer_get_time() / 1000;
    set_state(CAP_STATE_IDLE);
    return ESP_OK;
}

static bool in_boot_grace_window(void) {
    if (g_cfg.boot_grace_ms <= 0) return false;
    int64_t now_ms = esp_timer_get_time() / 1000;
    return (now_ms - g_boot_time_ms) < g_cfg.boot_grace_ms;
}

static void saved_reconnect_task(void *arg) {
    (void)arg;
    const TickType_t interval = pdMS_TO_TICKS(15 * 60 * 1000);
    while (1) {
        vTaskDelay(interval);
        if (g_saved_apsta_mode && !g_sta_have_ip && wifi_store_has_credentials()) {
            ESP_LOGI(TAG, "AP+STA offline: reintentando red guardada (intervalo 15 min)");
            g_connect_attempts = 0;
            set_state(CAP_STATE_CONNECTING);
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
        }
    }
}

static void ensure_saved_reconnect_task(void) {
    if (!g_saved_reconnect_task) {
        xTaskCreate(saved_reconnect_task, "saved_reconnect", 3072, NULL, 4, &g_saved_reconnect_task);
    }
}

static void start_with_saved_or_manager(void) {
    if (wifi_store_has_credentials()) {
        char ssid[33], pass[65];
        if (wifi_store_load(ssid, sizeof(ssid), pass, sizeof(pass)) == ESP_OK) {
            ESP_LOGI(TAG, "Credenciales guardadas encontradas: SSID=%s", ssid);
            connect_sta(ssid, pass, true);
            return;
        }
    }

    set_state(CAP_STATE_PREP);
    ESP_ERROR_CHECK(start_ap());
    ESP_ERROR_CHECK(start_http());
    set_state(CAP_STATE_SCAN);
    scan_start();
}

esp_err_t captive_manager_start(void) {
    if (g_state != CAP_STATE_IDLE) return ESP_ERR_INVALID_STATE;
    start_with_saved_or_manager();
    return ESP_OK;
}

static esp_err_t start_ap(void) {
    wifi_config_t ap_cfg = {0};
    snprintf((char*)ap_cfg.ap.ssid, sizeof(ap_cfg.ap.ssid), "%s", g_cfg.ap_ssid);
    snprintf((char*)ap_cfg.ap.password, sizeof(ap_cfg.ap.password), "%s", g_cfg.ap_pass);
    ap_cfg.ap.ssid_len = strlen((char*)ap_cfg.ap.ssid);
    ap_cfg.ap.channel = 1;
    ap_cfg.ap.authmode = strlen(g_cfg.ap_pass) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.beacon_interval = 100;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_LOGI(TAG, "AP started SSID=%s", ap_cfg.ap.ssid);
    start_mdns_service();
    return ESP_OK;
}

static esp_err_t start_apsta_with_saved_credentials(void) {
    char ssid[33] = {0};
    char pass[65] = {0};
    esp_err_t err = wifi_store_load(ssid, sizeof(ssid), pass, sizeof(pass));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No hay credenciales guardadas para reconectar en AP+STA: %s", esp_err_to_name(err));
        return err;
    }

    wifi_config_t ap_cfg = {0};
    snprintf((char*)ap_cfg.ap.ssid, sizeof(ap_cfg.ap.ssid), "%s", g_cfg.ap_ssid);
    snprintf((char*)ap_cfg.ap.password, sizeof(ap_cfg.ap.password), "%s", g_cfg.ap_pass);
    ap_cfg.ap.ssid_len = strlen((char*)ap_cfg.ap.ssid);
    ap_cfg.ap.channel = 1;
    ap_cfg.ap.authmode = strlen(g_cfg.ap_pass) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.beacon_interval = 100;

    wifi_config_t sta_cfg = {0};
    strlcpy((char*)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid));
    strlcpy((char*)sta_cfg.sta.password, pass, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(apply_saved_sta_ip_config());
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));

    g_sta_have_ip = false;
    g_using_saved = true;
    g_saved_apsta_mode = true;
    ensure_saved_reconnect_task();
    set_state(CAP_STATE_CONNECTING);
    ESP_LOGI(TAG, "AP+STA activo: AP=%s, reintentando red guardada SSID=%s", ap_cfg.ap.ssid, sta_cfg.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_connect());
    start_mdns_service();
    return ESP_OK;
}

static esp_err_t parse_ipv4_or_fail(const char *text, esp_ip4_addr_t *out) {
    if (!text || !out || !ip4addr_aton(text, (ip4_addr_t *)out)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t apply_saved_sta_ip_config(void) {
    if (!g_sta_netif) return ESP_ERR_INVALID_STATE;

    wifi_static_ip_cfg_t cfg = {0};
    esp_err_t err = wifi_store_load_static_ip_cfg(&cfg);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No se pudo leer config IP estática: %s", esp_err_to_name(err));
        return err;
    }

    if (!cfg.enabled) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcpc_stop(g_sta_netif));
        err = esp_netif_dhcpc_start(g_sta_netif);
        if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
            ESP_LOGW(TAG, "No se pudo iniciar DHCP cliente: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "STA usando DHCP");
        return ESP_OK;
    }

    esp_netif_ip_info_t ip_info = {0};
    ip4addr_aton("255.255.255.0", (ip4_addr_t *)&ip_info.netmask);
    if (parse_ipv4_or_fail(cfg.ip, &ip_info.ip) != ESP_OK ||
        parse_ipv4_or_fail(cfg.gateway, &ip_info.gw) != ESP_OK) {
        ESP_LOGE(TAG, "Configuración IP estática inválida en NVS");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcpc_stop(g_sta_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(g_sta_netif, &ip_info));

    ESP_LOGI(TAG, "STA usando IP estática: ip=%s mask=255.255.255.0 gw=%s",
             cfg.ip, cfg.gateway);
    return ESP_OK;
}

static void connect_sta(const char *ssid, const char *pass, bool from_saved) {
    wifi_config_t sta_cfg = {0};
    strlcpy((char*)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid));
    strlcpy((char*)sta_cfg.sta.password, pass ? pass : "", sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(apply_saved_sta_ip_config());
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    set_state(CAP_STATE_CONNECTING);
    g_sta_have_ip = false;
    g_using_saved = from_saved;
    g_saved_apsta_mode = false;
    g_connect_attempts = 0;
    ESP_LOGI(TAG, "(STA) Connecting to SSID=%s saved=%d", sta_cfg.sta.ssid, from_saved);
    ESP_ERROR_CHECK(esp_wifi_connect());
    start_mdns_service();
}

void captive_manager_notify_sta_got_ip(void) {
    g_sta_have_ip = true;
    g_saved_apsta_mode = false;
    g_connect_attempts = 0;
    shutdown_ap_http();
    esp_err_t http_err = start_http();
    if (http_err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar HTTP en STA: %s", esp_err_to_name(http_err));
    }
    set_state(CAP_STATE_OPERATIONAL);
    start_sntp_if_needed();
    ESP_LOGI(TAG, "STA connected and operational; HTTP endpoints available on port 80");
}

void captive_manager_notify_sta_disconnected(int reason_code) {
    ESP_LOGW(TAG, "STA disconnected (reason=%d) state=%s saved=%d attempts=%d",
             reason_code, captive_manager_state_str(g_state), g_using_saved, g_connect_attempts);
    g_sta_have_ip = false;

    if (g_state == CAP_STATE_CONNECTING || g_state == CAP_STATE_OPERATIONAL || g_state == CAP_STATE_APSTA_OFFLINE) {
        g_connect_attempts++;
        bool in_grace = in_boot_grace_window();
        bool under_limit = (g_connect_attempts < g_cfg.conn_max_attempts);

        if (under_limit || in_grace) {
            vTaskDelay(pdMS_TO_TICKS(g_cfg.conn_retry_delay_ms));
            esp_wifi_connect();
            return;
        }

        if (g_saved_apsta_mode) {
            ESP_LOGW(TAG, "Red guardada no disponible. AP+STA offline activo: midiendo en SD y reintentando cada 15 min");
            set_state(CAP_STATE_APSTA_OFFLINE);
            return;
        }

        ESP_LOGW(TAG, "Sin conexión tras reintentos. Activando AP y manteniendo reconexión STA con credenciales guardadas");
        captive_manager_enter_recaptive();
    }
}

esp_err_t captive_manager_enter_recaptive(void) {
    ESP_LOGW(TAG, "Entering Wi-Fi manager mode");
    shutdown_ap_http();
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_disconnect());
    g_sta_have_ip = false;
    g_connect_attempts = 0;

    if (wifi_store_has_credentials() && start_apsta_with_saved_credentials() == ESP_OK) {
    } else {
        g_using_saved = false;
        g_saved_apsta_mode = false;
        set_state(CAP_STATE_RECAPTIVE);
        ESP_ERROR_CHECK(start_ap());
        set_state(CAP_STATE_SCAN);
        scan_start();
    }

    if (!g_server) {
        ESP_ERROR_CHECK(start_http());
    }
    return ESP_OK;
}

static void scan_start(void) {
    wifi_scan_config_t sc = {
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE
    };
    esp_wifi_scan_start(&sc, false);
}

static esp_err_t favicon_get(httpd_req_t *r)
{
    httpd_resp_set_status(r, "204 No Content");
    httpd_resp_set_type(r, "image/x-icon");
    return httpd_resp_send(r, NULL, 0);
}

static esp_err_t root_get(httpd_req_t *r)
{
    httpd_resp_set_type(r, "text/html; charset=utf-8");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store, no-cache, must-revalidate, proxy-revalidate");
    httpd_resp_set_hdr(r, "Pragma", "no-cache");
    httpd_resp_set_hdr(r, "Expires", "0");
    httpd_resp_set_hdr(r, "Connection", "close");

    const char *page =
        "<!doctype html><html lang=\"es\"><head>"
        "<meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>EcoSensor Wi-Fi Manager</title>"
        "<style>"
        "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Helvetica,Arial,sans-serif;margin:0;background:#f6f7fb;color:#111}"
        ".wrap{max-width:720px;margin:24px auto;padding:16px}"
        ".card{background:#fff;border-radius:16px;box-shadow:0 6px 18px rgba(0,0,0,.08);padding:20px}"
        "h1{font-size:20px;margin:0 0 12px}"
        "label{display:block;font-weight:600;margin:16px 0 6px}"
        "select,input{width:100%;box-sizing:border-box;padding:10px 12px;border:1px solid #dcdfe6;border-radius:10px;font-size:14px}"
        ".row-check{display:flex;align-items:center;gap:10px;margin-top:18px}"
        ".row-check input{width:auto;margin:0}"
        ".row-check label{margin:0;font-weight:600}"
        ".ip-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}"
        "button,input[type=submit]{margin-top:18px;border:0;border-radius:12px;padding:12px 16px;cursor:pointer;font-weight:600}"
        "input[type=submit]{background:#111;color:#fff}"
        "</style></head><body><div class=\"wrap\"><div class=\"card\">"
        "<h1>EcoSensor</h1>"
        "<p>Para conectar EcoSensor a su red local seleccione el SSID de la red wifi, si su red no aparece refresque la pagina web para volver a escanear</p>"
        "<form id=\"wifiForm\">"
        "<label for=\"ssid\">SSID de la red local</label>"
        "<select id=\"ssid\" name=\"ssid\" required><option value=\"\" disabled selected>Cargando redes...</option></select>"
        "<label for=\"pass\">Capture su Contraseña</label>"
        "<input type=\"text\" id=\"pass\" name=\"pass\">"
        "<div class=\"row-check\"><input type=\"checkbox\" id=\"use_static_ip\" name=\"use_static_ip\"><label for=\"use_static_ip\">Configurar IP manual</label></div>"
        "<div class=\"ip-grid\">"
        "<div><label for=\"static_ip\">IP</label><input type=\"text\" id=\"static_ip\" name=\"static_ip\" placeholder=\"192.168.1.201\" disabled></div>"
        "<div><label for=\"gateway\">Gateway</label><input type=\"text\" id=\"gateway\" name=\"gateway\" placeholder=\"192.168.1.254\" disabled></div>"
        "</div>"
        "<input type=\"submit\" value=\"Guardar y reiniciar\">"
        "</form>"
        "</div></div>"
        "<script>\"use strict\";"
        "async function loadNetworks(){"
        "try{"
        "const r=await fetch('/scan');"
        "const j=await r.json();"
        "const aps=Array.isArray(j)?j:(Array.isArray(j.networks)?j.networks:[]);"
        "const sel=document.getElementById('ssid');"
        "sel.innerHTML='';"
        "if(aps.length){"
        "for(const ap of aps){"
        "const o=document.createElement('option');"
        "const isOpen=!!ap.open;"
        "o.value=ap.ssid||'';"
        "o.setAttribute('data-open',isOpen?'1':'0');"
        "o.textContent=(ap.ssid||'(oculta)')+(isOpen?' (abierta)':'');"
        "sel.appendChild(o);"
        "}"
        "sel.selectedIndex=0;applyPassDisable();"
        "}else{sel.innerHTML='<option value=\\\"\\\">(No se encontraron redes)</option>';document.getElementById('pass').disabled=false;}"
        "}catch(e){console.error('Error cargando redes:',e);}"
        "}"
        "function applyPassDisable(){"
        "const sel=document.getElementById('ssid');"
        "const pass=document.getElementById('pass');"
        "const opt=sel&&sel.options[sel.selectedIndex];"
        "const isOpen=!!(opt&&opt.getAttribute('data-open')==='1');"
        "pass.disabled=isOpen;"
        "if(isOpen){pass.value='';pass.placeholder='Esta red no requiere contraseña';}else{pass.placeholder=' ';}"
        "}"
        "function applyStaticIpToggle(){"
        "const enabled=document.getElementById('use_static_ip').checked;"
        "['static_ip','gateway'].forEach(id=>{document.getElementById(id).disabled=!enabled;});"
        "}"
        "document.getElementById('ssid').addEventListener('change',applyPassDisable);"
        "document.getElementById('use_static_ip').addEventListener('change',applyStaticIpToggle);"
        "document.getElementById('wifiForm').addEventListener('submit',async(ev)=>{"
        "ev.preventDefault();"
        "const ssid=document.getElementById('ssid').value||'';"
        "const pass=document.getElementById('pass').value||'';"
        "const use_static_ip=document.getElementById('use_static_ip').checked;"
        "const static_ip=document.getElementById('static_ip').value||'';"
        "const gateway=document.getElementById('gateway').value||'';"
        "if(use_static_ip && (!static_ip || !gateway)){alert('Para IP manual debes completar IP y gateway.');return;}"
        "try{"
        "const resp=await fetch('/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid,pass,use_static_ip,static_ip,gateway})});"
        "const txt=await resp.text();"
        "alert(txt||'Guardado. Reiniciando...');"
        "}catch(e){alert('Error guardando: '+(e&&e.message?e.message:e));}"
        "});"
        "window.addEventListener('load',()=>{loadNetworks();applyStaticIpToggle();});"
        "</script></body></html>";

    const char *device_id = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
    char display_id[64];
    if (strncmp(device_id, "ecosensor", 9) == 0 && strlen(device_id) > 9) {
        snprintf(display_id, sizeof(display_id), "EcoSensor%s", device_id + 9);
    } else {
        snprintf(display_id, sizeof(display_id), "%s", device_id);
    }

    const char *marker = "<h1>EcoSensor</h1>";
    const char *marker_pos = strstr(page, marker);
    if (!marker_pos) {
        return httpd_resp_send(r, page, HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_send_chunk(r, page, marker_pos - page);
    char heading[96];
    snprintf(heading, sizeof(heading), "<h1>%s</h1>", display_id);
    httpd_resp_send_chunk(r, heading, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(r, marker_pos + strlen(marker), HTTPD_RESP_USE_STRLEN);
    return httpd_resp_send_chunk(r, NULL, 0);
}

static esp_err_t save_post(httpd_req_t *r) {
    char buf[512];
    int len = httpd_req_recv(r, buf, sizeof(buf)-1);
    if (len <= 0) return httpd_resp_send_err(r, 400, "empty");
    buf[len] = 0;

    cJSON *root = cJSON_Parse(buf);
    if (!root) return httpd_resp_send_err(r, 400, "json");

    cJSON *js = cJSON_GetObjectItem(root, "ssid");
    cJSON *jp = cJSON_GetObjectItem(root, "pass");
    cJSON *j_use_static_ip = cJSON_GetObjectItem(root, "use_static_ip");
    cJSON *j_static_ip = cJSON_GetObjectItem(root, "static_ip");
    cJSON *j_gateway = cJSON_GetObjectItem(root, "gateway");
    if (!cJSON_IsString(js)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 400, "ssid?");
    }

    const char *ssid = js->valuestring;
    const char *pass = (jp && cJSON_IsString(jp)) ? jp->valuestring : "";
    bool use_static_ip = cJSON_IsTrue(j_use_static_ip);

    wifi_static_ip_cfg_t ip_cfg = {0};
    ip_cfg.enabled = use_static_ip;
    if (use_static_ip) {
        const char *static_ip = (j_static_ip && cJSON_IsString(j_static_ip)) ? j_static_ip->valuestring : "";
        const char *gateway = (j_gateway && cJSON_IsString(j_gateway)) ? j_gateway->valuestring : "";

        if (!static_ip[0] || !gateway[0]) {
            cJSON_Delete(root);
            return httpd_resp_send_err(r, 400, "static ip fields missing");
        }
        if (strlen(static_ip) >= sizeof(ip_cfg.ip) || strlen(gateway) >= sizeof(ip_cfg.gateway)) {
            cJSON_Delete(root);
            return httpd_resp_send_err(r, 400, "static ip field too long");
        }

        strcpy(ip_cfg.ip, static_ip);
        strcpy(ip_cfg.gateway, gateway);

        esp_ip4_addr_t tmp = {0};
        if (parse_ipv4_or_fail(ip_cfg.ip, &tmp) != ESP_OK ||
            parse_ipv4_or_fail(ip_cfg.gateway, &tmp) != ESP_OK) {
            cJSON_Delete(root);
            return httpd_resp_send_err(r, 400, "invalid static ip config");
        }
    }

    esp_err_t err = wifi_store_save(ssid, pass);
    if (err == ESP_OK) {
        err = wifi_store_save_static_ip_cfg(&ip_cfg);
    }
    if (err != ESP_OK) {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 500, "save failed");
    }

    httpd_resp_sendstr(r, "Datos guardados. El dispositivo se reiniciará para conectar a la red.");
    xTaskCreate(restart_later_task, "rst_later", 2048, NULL, 5, NULL);

    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t scan_get(httpd_req_t *r) {
    wifi_mode_t old_mode;
    esp_wifi_get_mode(&old_mode);
    if (old_mode != WIFI_MODE_APSTA) {
        esp_wifi_set_mode(WIFI_MODE_APSTA);
    }

    wifi_scan_config_t sc = { .show_hidden = false };
    esp_wifi_scan_start(&sc, true);

    uint16_t ap_num = 0;
    esp_wifi_scan_get_ap_num(&ap_num);
    if (ap_num > (uint16_t)g_cfg.max_scan_aps) ap_num = (uint16_t)g_cfg.max_scan_aps;

    wifi_ap_record_t *list = calloc(ap_num ? ap_num : 1, sizeof(wifi_ap_record_t));
    if (!list) {
        if (old_mode != WIFI_MODE_APSTA) esp_wifi_set_mode(old_mode);
        return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    }

    if (ap_num > 0) {
        esp_wifi_scan_get_ap_records(&ap_num, list);
    }

    httpd_resp_set_type(r, "application/json");
    httpd_resp_send_chunk(r, "{\"networks\":[", -1);
    for (int i = 0; i < ap_num; i++) {
        char buf[128];
        int open = (list[i].authmode == WIFI_AUTH_OPEN) ? 1 : 0;
        snprintf(buf, sizeof(buf), "{\"ssid\":\"%s\",\"open\":%d}", (char*)list[i].ssid, open);
        httpd_resp_send_chunk(r, buf, -1);
        if (i < ap_num - 1) httpd_resp_send_chunk(r, ",", -1);
    }
    httpd_resp_send_chunk(r, "]}", -1);
    httpd_resp_send_chunk(r, NULL, 0);

    free(list);
    if (old_mode != WIFI_MODE_APSTA) esp_wifi_set_mode(old_mode);
    return ESP_OK;
}


static esp_err_t status_get(httpd_req_t *r) {
    const char *device_id = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
    char mdns_name[64];
    char ip_buf[32];
    snprintf(mdns_name, sizeof(mdns_name), "%s.local", device_id);
    get_active_ip_string(ip_buf, sizeof(ip_buf));

    const char *wifi = "ap_mode";
    if (g_sta_have_ip) {
        wifi = "connected";
    } else if (g_saved_apsta_mode || g_state == CAP_STATE_APSTA_OFFLINE) {
        wifi = "apsta_offline";
    } else if (g_state == CAP_STATE_CONNECTING || g_state == CAP_STATE_VERIFY || g_state == CAP_STATE_WAIT_LOGIN) {
        wifi = "connecting";
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddStringToObject(root, "firmware_version", ota_manager_current_version());
    cJSON_AddStringToObject(root, "wifi", wifi);
    cJSON_AddStringToObject(root, "ip", ip_buf);
    cJSON_AddStringToObject(root, "mdns", mdns_name);
    cJSON_AddBoolToObject(root, "sd_ready", sd_store_is_ready());
    cJSON_AddNumberToObject(root, "sd_last_id", sd_store_last_id());
    cJSON_AddBoolToObject(root, "checkpoint_valid", sd_store_checkpoint_valid());
    cJSON_AddNumberToObject(root, "checkpoint_generation", (double)sd_store_checkpoint_generation());
    cJSON_AddBoolToObject(root, "history_index_ready", sd_store_history_index_ready());
    cJSON_AddBoolToObject(root, "history_index_rebuilding", sd_store_history_index_rebuilding());
    cJSON_AddNumberToObject(root, "history_index_points", sd_store_history_index_points());
    cJSON_AddNumberToObject(root, "sd_format_version", sd_store_format_version());
    cJSON_AddNumberToObject(root, "boot_id", g_boot_id);
    cJSON_AddNumberToObject(root, "current_uptime_s", (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddBoolToObject(root, "time_valid", g_time_valid);
    cJSON_AddBoolToObject(root, "needs_time_sync", !g_time_valid);
    cJSON_AddStringToObject(root, "time_source", g_time_valid ? g_last_sync_source : "none");
    cJSON_AddStringToObject(root, "last_sync_source", g_last_sync_source);
    if (g_time_valid) {
        cJSON_AddNumberToObject(root, "current_epoch", (double)time(NULL));
    } else {
        cJSON_AddNullToObject(root, "current_epoch");
    }
    if (g_push_host[0]) {
        cJSON_AddStringToObject(root, "push_host", g_push_host);
    } else {
        cJSON_AddNullToObject(root, "push_host");
    }
    if (g_time_valid) {
        char current_datetime[32];
        get_current_datetime_string(current_datetime, sizeof(current_datetime));
        cJSON_AddStringToObject(root, "date", g_config_date);
        cJSON_AddStringToObject(root, "time", g_config_time);
        cJSON_AddStringToObject(root, "current_datetime", current_datetime);
    } else {
        cJSON_AddNullToObject(root, "current_datetime");
        if (g_config_date[0] && g_config_time[0]) {
            cJSON_AddStringToObject(root, "last_saved_date", g_config_date);
            cJSON_AddStringToObject(root, "last_saved_time", g_config_time);
        }
    }
    cJSON_AddStringToObject(root, "sensors", g_sensors_started ? "running" : "waiting");
    uint32_t last_measurement_id = g_last_readings.measurement_id;
    uint32_t sd_last_id = sd_store_last_id();
    if (sd_last_id > last_measurement_id) {
        last_measurement_id = sd_last_id;
    }
    cJSON_AddNumberToObject(root, "last_measurement_id", last_measurement_id);
    cJSON_AddNumberToObject(root, "last_measurement_uptime_s", g_last_readings.uptime_s);
    if (g_last_readings.timestamp[0]) {
        cJSON_AddStringToObject(root, "last_measurement_timestamp", g_last_readings.timestamp);
    } else {
        cJSON_AddNullToObject(root, "last_measurement_timestamp");
    }
    cJSON_AddBoolToObject(root, "last_measurement_time_valid", g_last_readings.time_valid);
    cJSON_AddStringToObject(root, "last_measurement_time_source",
                            g_last_readings.time_source[0] ? g_last_readings.time_source : "none");
    time_t last_measurement_epoch = 0;
    if (g_last_readings.time_valid && parse_utc_timestamp(g_last_readings.timestamp, &last_measurement_epoch)) {
        cJSON_AddNumberToObject(root, "last_measurement_epoch", (double)last_measurement_epoch);
    } else {
        cJSON_AddNullToObject(root, "last_measurement_epoch");
    }
    cJSON_AddBoolToObject(root, "gps_valid", g_last_readings.gps_valid);
    if (g_last_readings.gps_valid) {
        cJSON_AddNumberToObject(root, "gps_lat", g_last_readings.gps_lat);
        cJSON_AddNumberToObject(root, "gps_lon", g_last_readings.gps_lon);
        cJSON_AddNumberToObject(root, "gps_satellites", g_last_readings.gps_satellites);
        cJSON_AddNumberToObject(root, "gps_hdop", g_last_readings.gps_hdop);
        cJSON_AddNumberToObject(root, "gps_age_ms", g_last_readings.gps_age_ms);
    } else {
        cJSON_AddNullToObject(root, "gps_lat");
        cJSON_AddNullToObject(root, "gps_lon");
        cJSON_AddNumberToObject(root, "gps_satellites", 0);
        cJSON_AddNumberToObject(root, "gps_hdop", 0);
        cJSON_AddNumberToObject(root, "gps_age_ms", 0);
    }
    cJSON_AddStringToObject(root, "state", captive_manager_state_str(g_state));
    cJSON_AddBoolToObject(root, "using_saved", g_using_saved);
    cJSON_AddBoolToObject(root, "saved_apsta_mode", g_saved_apsta_mode);
    cJSON_AddBoolToObject(root, "can_measure", captive_manager_can_measure());
    cJSON_AddBoolToObject(root, "can_push", captive_manager_can_push_measurements());
    cJSON_AddNumberToObject(root, "conn_attempts", g_connect_attempts);
    char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, out);
    free(out);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t lecturas_get(httpd_req_t *r) {
    const char *device_id = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
    uint32_t sd_last_id = sd_store_last_id();
    bool use_sd_latest = sd_store_is_ready() && sd_last_id > 0 && (!g_last_readings.valid || sd_last_id > g_last_readings.measurement_id);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddNumberToObject(root, "boot_id", g_boot_id);
    cJSON_AddNumberToObject(root, "current_uptime_s", (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddBoolToObject(root, "sd_ready", sd_store_is_ready());
    cJSON_AddNumberToObject(root, "sd_last_id", sd_last_id);

    if (use_sd_latest) {
        bool found = false;
        esp_err_t err = sd_store_add_latest_reading(root, 1500, &found);
        if (err == ESP_OK && found) {
            cJSON_AddBoolToObject(root, "valid", true);
            cJSON_AddStringToObject(root, "latest_source", "sd");
        } else {
            ESP_LOGW(TAG, "No se pudo leer latest desde SD id=%lu err=%s found=%d", (unsigned long)sd_last_id, esp_err_to_name(err), found);
            use_sd_latest = false;
        }
    }

    if (!use_sd_latest) {
        cJSON_AddBoolToObject(root, "valid", g_last_readings.valid);
        cJSON_AddNumberToObject(root, "window_s", g_last_readings.window_s);

        if (!g_last_readings.valid) {
            cJSON_AddBoolToObject(root, "time_valid", g_time_valid);
            cJSON_AddStringToObject(root, "message", "Sin lecturas promediadas disponibles");
        } else {
            cJSON_AddStringToObject(root, "latest_source", "ram");
            cJSON_AddNumberToObject(root, "id", g_last_readings.measurement_id);
            cJSON_AddNumberToObject(root, "measurement_id", g_last_readings.measurement_id);
            cJSON_AddNumberToObject(root, "boot_id", g_last_readings.boot_id);
            cJSON_AddNumberToObject(root, "uptime_s", g_last_readings.uptime_s);
            cJSON_AddBoolToObject(root, "time_valid", g_last_readings.time_valid);
            cJSON_AddStringToObject(root, "time_source",
                                    g_last_readings.time_source[0] ? g_last_readings.time_source : "none");
            if (g_last_readings.timestamp[0]) {
                cJSON_AddStringToObject(root, "timestamp", g_last_readings.timestamp);
            } else {
                cJSON_AddNullToObject(root, "timestamp");
            }
            cJSON_AddNumberToObject(root, "co2", g_last_readings.co2);
            cJSON_AddNumberToObject(root, "pm1p0", g_last_readings.pm1p0);
            cJSON_AddNumberToObject(root, "pm2p5", g_last_readings.pm2p5);
            cJSON_AddNumberToObject(root, "pm4p0", g_last_readings.pm4p0);
            cJSON_AddNumberToObject(root, "pm10p0", g_last_readings.pm10p0);
            cJSON_AddNumberToObject(root, "voc", g_last_readings.voc);
            cJSON_AddNumberToObject(root, "nox", g_last_readings.nox);
            cJSON_AddNumberToObject(root, "temp", g_last_readings.temp);
            cJSON_AddNumberToObject(root, "hum", g_last_readings.hum);
            cJSON_AddNumberToObject(root, "scd_temp", g_last_readings.scd_temp);
            cJSON_AddNumberToObject(root, "scd_hum", g_last_readings.scd_hum);
            cJSON_AddNumberToObject(root, "sen_temp", g_last_readings.sen_temp);
            cJSON_AddNumberToObject(root, "sen_hum", g_last_readings.sen_hum);
            cJSON_AddBoolToObject(root, "gps_valid", g_last_readings.gps_valid);
            if (g_last_readings.gps_valid) {
                cJSON_AddNumberToObject(root, "gps_lat", g_last_readings.gps_lat);
                cJSON_AddNumberToObject(root, "gps_lon", g_last_readings.gps_lon);
                cJSON_AddNumberToObject(root, "gps_satellites", g_last_readings.gps_satellites);
                cJSON_AddNumberToObject(root, "gps_hdop", g_last_readings.gps_hdop);
                cJSON_AddNumberToObject(root, "gps_age_ms", g_last_readings.gps_age_ms);
            } else {
                cJSON_AddNullToObject(root, "gps_lat");
                cJSON_AddNullToObject(root, "gps_lon");
                cJSON_AddNumberToObject(root, "gps_satellites", 0);
                cJSON_AddNumberToObject(root, "gps_hdop", 0);
                cJSON_AddNumberToObject(root, "gps_age_ms", 0);
            }
        }
    }

    char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, out);
    free(out);
    cJSON_Delete(root);
    return ESP_OK;
}



static bool safe_web_asset_name(const char *name) {
    if (!name || !name[0] || strlen(name) > 48) return false;
    if (strstr(name, "..") || strchr(name, '/') || strchr(name, '\\')) return false;
    for (const char *p = name; *p; ++p) {
        if (!(isalnum((unsigned char)*p) || *p == '.' || *p == '_' || *p == '-')) return false;
    }
    return true;
}

static const char *web_asset_content_type(const char *name) {
    const char *ext = strrchr(name ? name : "", '.');
    if (!ext) return "application/octet-stream";
    if (strcmp(ext, ".html") == 0 || strcmp(ext, ".htm") == 0) return "text/html; charset=utf-8";
    if (strcmp(ext, ".css") == 0) return "text/css; charset=utf-8";
    if (strcmp(ext, ".js") == 0) return "application/javascript; charset=utf-8";
    if (strcmp(ext, ".png") == 0) return "image/png";
    return "application/octet-stream";
}

static bool serve_sd_web_file(httpd_req_t *r, const char *name) {
    if (!sd_store_is_ready() || !safe_web_asset_name(name)) return false;

    char path[96];
    snprintf(path, sizeof(path), "/sdcard/web/%s", name);
    FILE *f = fopen(path, "rb");
    if (!f) return false;

    httpd_resp_set_type(r, web_asset_content_type(name));
    httpd_resp_set_hdr(r, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(r, "Pragma", "no-cache");
    char chunk[1024];
    size_t n = 0;
    while ((n = fread(chunk, 1, sizeof(chunk), f)) > 0) {
        if (httpd_resp_send_chunk(r, chunk, n) != ESP_OK) {
            fclose(f);
            httpd_resp_send_chunk(r, NULL, 0);
            return true;
        }
    }
    fclose(f);
    httpd_resp_send_chunk(r, NULL, 0);
    return true;
}

static esp_err_t sd_web_asset_get(httpd_req_t *r, const char *name) {
    if (serve_sd_web_file(r, name)) return ESP_OK;
    return httpd_resp_send_err(r, 404, "web asset not found");
}

static esp_err_t sd_style_get(httpd_req_t *r) { return sd_web_asset_get(r, "st.css"); }
static esp_err_t sd_script_get(httpd_req_t *r) { return sd_web_asset_get(r, "sc.js"); }
static esp_err_t sd_logo_get(httpd_req_t *r) { return sd_web_asset_get(r, "lct.png"); }

static void clear_sd_web_dir(void) {
    if (!sd_store_is_ready()) return;

    DIR *dir = opendir("/sdcard/web");
    if (!dir) {
        return;
    }

    struct dirent *entry = NULL;
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        char path[320];
        snprintf(path, sizeof(path), "/sdcard/web/%s", entry->d_name);
        if (remove(path) != 0) {
            ESP_LOGW(TAG, "No se pudo borrar asset residual %s errno=%d (%s)", path, errno, strerror(errno));
        }
    }
    closedir(dir);
    if (rmdir("/sdcard/web") != 0 && errno != ENOENT) {
        ESP_LOGW(TAG, "No se pudo borrar directorio /sdcard/web errno=%d (%s)", errno, strerror(errno));
    }
}

static esp_err_t ensure_sd_web_dir(void) {
    if (!sd_store_is_ready()) return ESP_ERR_INVALID_STATE;
    if (mkdir("/sdcard/web", 0775) != 0 && errno != EEXIST) {
        ESP_LOGE(TAG, "No se pudo crear /sdcard/web errno=%d (%s)", errno, strerror(errno));
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t download_web_asset_to_sd(const char *url, const char *name, size_t *out_bytes, char *err_buf, size_t err_buf_size) {
    if (err_buf && err_buf_size) err_buf[0] = '\0';
    if (!url || !url[0] || !safe_web_asset_name(name)) {
        if (err_buf && err_buf_size) snprintf(err_buf, err_buf_size, "invalid_arg");
        return ESP_ERR_INVALID_ARG;
    }
    if (ensure_sd_web_dir() != ESP_OK) {
        if (err_buf && err_buf_size) snprintf(err_buf, err_buf_size, "sd_web_dir_failed");
        return ESP_ERR_INVALID_STATE;
    }

    char final_path[96];
    snprintf(final_path, sizeof(final_path), "/sdcard/web/%s", name);

    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 15000,
        .buffer_size = 1024,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        if (err_buf && err_buf_size) snprintf(err_buf, err_buf_size, "http_client_init_failed");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        if (err_buf && err_buf_size) snprintf(err_buf, err_buf_size, "http_open:%s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    (void)esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status && (status < 200 || status >= 300)) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        ESP_LOGW(TAG, "HTTP asset %s status=%d url=%s", name, status, url);
        if (err_buf && err_buf_size) snprintf(err_buf, err_buf_size, "http_status:%d", status);
        return ESP_FAIL;
    }

    remove(final_path);
    FILE *f = fopen(final_path, "wb");
    if (!f) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        ESP_LOGE(TAG, "No se pudo abrir %s errno=%d (%s)", final_path, errno, strerror(errno));
        if (err_buf && err_buf_size) snprintf(err_buf, err_buf_size, "fopen:%d:%s", errno, strerror(errno));
        return ESP_FAIL;
    }

    size_t total = 0;
    char buffer[1024];
    while (1) {
        int read_len = esp_http_client_read(client, buffer, sizeof(buffer));
        if (read_len < 0) {
            err = ESP_FAIL;
            if (err_buf && err_buf_size) snprintf(err_buf, err_buf_size, "http_read_failed");
            break;
        }
        if (read_len == 0) {
            break;
        }
        if (fwrite(buffer, 1, read_len, f) != (size_t)read_len) {
            err = ESP_FAIL;
            if (err_buf && err_buf_size) snprintf(err_buf, err_buf_size, "fwrite:%d:%s", errno, strerror(errno));
            break;
        }
        total += (size_t)read_len;
    }

    if (fflush(f) != 0) {
        err = ESP_FAIL;
        if (err_buf && err_buf_size) snprintf(err_buf, err_buf_size, "fflush:%d:%s", errno, strerror(errno));
    }
    fclose(f);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        remove(final_path);
        if (out_bytes) *out_bytes = total;
        return err;
    }
    if (out_bytes) *out_bytes = total;
    ESP_LOGI(TAG, "Asset web guardado en SD: %s (%u bytes)", final_path, (unsigned)total);
    return ESP_OK;
}


static esp_err_t tabla_get(httpd_req_t *r) {
    /* La vista compilada es la fuente autoritativa. Un in.htm antiguo en la SD
       no debe volver a cortar timestamps UTC sin convertirlos. */

    const char *device_id = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
    char display_id[64];
    if (strncmp(device_id, "ecosensor", 9) == 0) {
        snprintf(display_id, sizeof(display_id), "EcoSensor%s", device_id + 9);
    } else {
        snprintf(display_id, sizeof(display_id), "%s", device_id);
    }

    httpd_resp_set_type(r, "text/html; charset=utf-8");
    httpd_resp_send_chunk(r,
        "<!DOCTYPE html><html lang=\"es\"><head>"
        "<title>EcoSensor</title>"
        "<meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
        "<meta http-equiv=\"refresh\" content=\"20\">"
        "<style>"
        "html{font-family:Arial Narrow,Arial,sans-serif;text-align:center}"
        "body{background-color:#cce5dc;padding:10%;margin:0}"
        ".brand{display:flex;align-items:center;justify-content:center;gap:12px;margin:0 0 14px}"
        ".brand img{width:80px;height:80px;object-fit:contain}"
        ".brand-title{color:rgb(4,87,9);font-size:25px;font-weight:700}"
        ".brand-title sup{font-size:11px;margin-left:1px}"
        "h2{color:rgb(4,4,52);text-decoration:underline;margin:10px 0}"
        "p{font-size:18px;color:rgb(4,4,52)}"
        "table{width:100%;border-spacing:0;border-collapse:separate;margin-top:20px}"
        "th,td{font-size:25px;text-align:center;border:1px solid black;border-radius:10px;padding:8px;background:#ffffff66}"
        "th{background-color:#80ffd4}"
        ".estado{font-size:18px;margin-top:18px}.aviso{font-size:22px;background:#fff3b0;border:1px solid #000;border-radius:10px;padding:12px}"
        "@media(max-width:640px){body{padding:6%}th,td{font-size:18px;padding:7px}h1{font-size:23px}h2{font-size:20px}p{font-size:16px}}"
        "</style></head><body>", -1);

    httpd_resp_send_chunk(r, "<div class=\"brand\"><img src=\"lct.png\" alt=\"LCT Didacticos\"><div class=\"brand-title\">EcoSensor<sup>&reg;</sup></div></div>", -1);
    char buf[256];
    snprintf(buf, sizeof(buf), "<h2>ID: %s</h2>", display_id);
    httpd_resp_send_chunk(r, buf, -1);

    if (!g_last_readings.valid) {
        httpd_resp_send_chunk(r,
            "<p class=\"aviso\">Sin medicion promediada disponible.<br>"
            "Espera a que termine la primera ventana de muestreo del EcoSensor.</p>", -1);
    } else {
        httpd_resp_send_chunk(r,
            "<table><tr><th>Mediciones</th><th>Valor</th><th>Unidad</th></tr>", -1);
        snprintf(buf, sizeof(buf), "<tr><td>CO2</td><td>%u</td><td>ppm</td></tr>", g_last_readings.co2); httpd_resp_send_chunk(r, buf, -1);
        snprintf(buf, sizeof(buf), "<tr><td>PM 1.0</td><td>%.2f</td><td>ug/m3</td></tr>", g_last_readings.pm1p0); httpd_resp_send_chunk(r, buf, -1);
        snprintf(buf, sizeof(buf), "<tr><td>PM 2.5</td><td>%.2f</td><td>ug/m3</td></tr>", g_last_readings.pm2p5); httpd_resp_send_chunk(r, buf, -1);
        snprintf(buf, sizeof(buf), "<tr><td>PM 4.0</td><td>%.2f</td><td>ug/m3</td></tr>", g_last_readings.pm4p0); httpd_resp_send_chunk(r, buf, -1);
        snprintf(buf, sizeof(buf), "<tr><td>PM 10</td><td>%.2f</td><td>ug/m3</td></tr>", g_last_readings.pm10p0); httpd_resp_send_chunk(r, buf, -1);
        snprintf(buf, sizeof(buf), "<tr><td>VOC</td><td>%.2f</td><td>indice</td></tr>", g_last_readings.voc); httpd_resp_send_chunk(r, buf, -1);
        snprintf(buf, sizeof(buf), "<tr><td>NOx</td><td>%.2f</td><td>indice</td></tr>", g_last_readings.nox); httpd_resp_send_chunk(r, buf, -1);
        snprintf(buf, sizeof(buf), "<tr><td>Temperatura</td><td>%.2f</td><td>&deg;C</td></tr>", g_last_readings.temp); httpd_resp_send_chunk(r, buf, -1);
        snprintf(buf, sizeof(buf), "<tr><td>Humedad</td><td>%.2f</td><td>%%</td></tr>", g_last_readings.hum); httpd_resp_send_chunk(r, buf, -1);
        httpd_resp_send_chunk(r, "</table>", -1);

        time_t measurement_epoch = 0;
        if (g_last_readings.time_valid && parse_utc_timestamp(g_last_readings.timestamp, &measurement_epoch)) {
            char date_text[11];
            char time_text[9];
            struct tm local_tm = {0};
            localtime_r(&measurement_epoch, &local_tm);
            strftime(date_text, sizeof(date_text), "%d-%m-%Y", &local_tm);
            strftime(time_text, sizeof(time_text), "%H:%M:%S", &local_tm);
            snprintf(buf, sizeof(buf), "<p class=\"estado\">Fecha de última medición: <span>%s</span></p>", date_text);
            httpd_resp_send_chunk(r, buf, -1);
            snprintf(buf, sizeof(buf), "<p class=\"estado\">Hora última medición: <span>%s</span></p>", time_text);
            httpd_resp_send_chunk(r, buf, -1);
        } else {
            httpd_resp_send_chunk(r, "<p class=\"estado\">Fecha de última medición: Pendiente</p>", -1);
            httpd_resp_send_chunk(r, "<p class=\"estado\">Hora última medición: Pendiente</p>", -1);
        }
    }

    httpd_resp_send_chunk(r, "</body></html>", -1);
    httpd_resp_send_chunk(r, NULL, 0);
    return ESP_OK;
}

static esp_err_t lecturas_since_get(httpd_req_t *r) {
    char query[96] = {0};
    uint32_t after_id = 0;
    uint32_t limit = 25;
    uint32_t timeout_ms = 1200;

    if (httpd_req_get_url_query_str(r, query, sizeof(query)) == ESP_OK) {
        char value[24] = {0};
        if (httpd_query_key_value(query, "after", value, sizeof(value)) == ESP_OK ||
            httpd_query_key_value(query, "after_id", value, sizeof(value)) == ESP_OK ||
            httpd_query_key_value(query, "since", value, sizeof(value)) == ESP_OK) {
            after_id = (uint32_t)strtoul(value, NULL, 10);
        }
        if (httpd_query_key_value(query, "limit", value, sizeof(value)) == ESP_OK) {
            limit = (uint32_t)strtoul(value, NULL, 10);
        }
        if (httpd_query_key_value(query, "timeout_ms", value, sizeof(value)) == ESP_OK) {
            timeout_ms = (uint32_t)strtoul(value, NULL, 10);
        }
    }

    const char *device_id = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
    cJSON *root = cJSON_CreateObject();
    cJSON *rows = cJSON_CreateArray();
    uint32_t added = 0;
    uint32_t scanned = 0;
    esp_err_t err = sd_store_add_readings_since(rows, after_id, limit, timeout_ms, &added, &scanned);

    cJSON_AddBoolToObject(root, "ok", err == ESP_OK);
    if (err == ESP_ERR_TIMEOUT) {
        cJSON_AddStringToObject(root, "error", "sd_scan_timeout");
    } else if (err != ESP_OK) {
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
    }
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddBoolToObject(root, "sd_ready", sd_store_is_ready());
    cJSON_AddNumberToObject(root, "after_id", after_id);
    cJSON_AddNumberToObject(root, "last_id", sd_store_last_id());
    cJSON_AddNumberToObject(root, "boot_id", g_boot_id);
    cJSON_AddNumberToObject(root, "current_uptime_s", (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddNumberToObject(root, "requested_limit", limit);
    cJSON_AddNumberToObject(root, "timeout_ms", timeout_ms);
    cJSON_AddNumberToObject(root, "scanned", scanned);
    cJSON_AddNumberToObject(root, "count", added);
    cJSON_AddItemToObject(root, "rows", rows);

    char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, out);
    free(out);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t lecturas_range_get(httpd_req_t *r) {
    char query[128] = {0};
    uint32_t from_id = 0;
    uint32_t to_id = 0;
    uint32_t limit = 25;
    uint32_t timeout_ms = 20000;

    if (httpd_req_get_url_query_str(r, query, sizeof(query)) == ESP_OK) {
        char value[24] = {0};
        if (httpd_query_key_value(query, "from", value, sizeof(value)) == ESP_OK ||
            httpd_query_key_value(query, "from_id", value, sizeof(value)) == ESP_OK) {
            from_id = (uint32_t)strtoul(value, NULL, 10);
        }
        if (httpd_query_key_value(query, "to", value, sizeof(value)) == ESP_OK ||
            httpd_query_key_value(query, "to_id", value, sizeof(value)) == ESP_OK) {
            to_id = (uint32_t)strtoul(value, NULL, 10);
        }
        if (httpd_query_key_value(query, "limit", value, sizeof(value)) == ESP_OK) {
            limit = (uint32_t)strtoul(value, NULL, 10);
        }
        if (httpd_query_key_value(query, "timeout_ms", value, sizeof(value)) == ESP_OK) {
            timeout_ms = (uint32_t)strtoul(value, NULL, 10);
        }
    }

    const char *device_id = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
    cJSON *root = cJSON_CreateObject();
    cJSON *rows = cJSON_CreateArray();
    uint32_t added = 0;
    uint32_t scanned = 0;
    esp_err_t err = sd_store_add_readings_range(rows, from_id, to_id, limit, timeout_ms, &added, &scanned);

    cJSON_AddBoolToObject(root, "ok", err == ESP_OK);
    if (err == ESP_ERR_TIMEOUT) {
        cJSON_AddStringToObject(root, "error", "sd_scan_timeout");
    } else if (err != ESP_OK) {
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
    }
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddBoolToObject(root, "sd_ready", sd_store_is_ready());
    cJSON_AddNumberToObject(root, "from_id", from_id);
    cJSON_AddNumberToObject(root, "to_id", to_id);
    cJSON_AddNumberToObject(root, "last_id", sd_store_last_id());
    cJSON_AddNumberToObject(root, "boot_id", g_boot_id);
    cJSON_AddNumberToObject(root, "current_uptime_s", (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddNumberToObject(root, "requested_limit", limit);
    cJSON_AddNumberToObject(root, "timeout_ms", timeout_ms);
    cJSON_AddNumberToObject(root, "scanned", scanned);
    cJSON_AddNumberToObject(root, "count", added);
    cJSON_AddItemToObject(root, "rows", rows);

    char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, out);
    free(out);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t ndjson_http_writer(const char *line, void *ctx) {
    httpd_req_t *req = (httpd_req_t *)ctx;
    if (!line || !req) {
        return ESP_ERR_INVALID_ARG;
    }
    return httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t lecturas_export_get(httpd_req_t *r) {
    char query[128] = {0};
    uint32_t from_id = 0;
    uint32_t to_id = 0;
    uint32_t timeout_ms = 120000;

    if (httpd_req_get_url_query_str(r, query, sizeof(query)) == ESP_OK) {
        char value[24] = {0};
        if (httpd_query_key_value(query, "from", value, sizeof(value)) == ESP_OK ||
            httpd_query_key_value(query, "from_id", value, sizeof(value)) == ESP_OK) {
            from_id = (uint32_t)strtoul(value, NULL, 10);
        }
        if (httpd_query_key_value(query, "to", value, sizeof(value)) == ESP_OK ||
            httpd_query_key_value(query, "to_id", value, sizeof(value)) == ESP_OK) {
            to_id = (uint32_t)strtoul(value, NULL, 10);
        }
        if (httpd_query_key_value(query, "timeout_ms", value, sizeof(value)) == ESP_OK) {
            timeout_ms = (uint32_t)strtoul(value, NULL, 10);
        }
    }

    httpd_resp_set_type(r, "application/x-ndjson");
    httpd_resp_set_hdr(r, "X-EcoSensor-Format", "ndjson");
    httpd_resp_set_hdr(r, "X-EcoSensor-Last-Id", "0");

    uint32_t added = 0;
    uint32_t scanned = 0;
    esp_err_t err = sd_store_stream_readings_range_ndjson(from_id, to_id, timeout_ms, ndjson_http_writer, r, &added, &scanned);
    if (err == ESP_OK) {
        httpd_resp_send_chunk(r, NULL, 0);
    } else {
        char error_line[160];
        snprintf(error_line, sizeof(error_line), "{\"error\":\"%s\",\"count\":%lu,\"scanned\":%lu}\n",
                 esp_err_to_name(err), (unsigned long)added, (unsigned long)scanned);
        httpd_resp_send_chunk(r, error_line, HTTPD_RESP_USE_STRLEN);
        httpd_resp_send_chunk(r, NULL, 0);
    }
    return ESP_OK;
}

static esp_err_t lecturas_recent_get(httpd_req_t *r) {
    char query[128] = {0};
    uint32_t after_id = 0;
    uint32_t before_id = 0;
    uint32_t limit = 25;
    uint32_t timeout_ms = 1200;

    if (httpd_req_get_url_query_str(r, query, sizeof(query)) == ESP_OK) {
        char value[24] = {0};
        if (httpd_query_key_value(query, "after", value, sizeof(value)) == ESP_OK ||
            httpd_query_key_value(query, "after_id", value, sizeof(value)) == ESP_OK) {
            after_id = (uint32_t)strtoul(value, NULL, 10);
        }
        if (httpd_query_key_value(query, "before", value, sizeof(value)) == ESP_OK ||
            httpd_query_key_value(query, "before_id", value, sizeof(value)) == ESP_OK) {
            before_id = (uint32_t)strtoul(value, NULL, 10);
        }
        if (httpd_query_key_value(query, "limit", value, sizeof(value)) == ESP_OK) {
            limit = (uint32_t)strtoul(value, NULL, 10);
        }
        if (httpd_query_key_value(query, "timeout_ms", value, sizeof(value)) == ESP_OK) {
            timeout_ms = (uint32_t)strtoul(value, NULL, 10);
        }
    }

    const char *device_id = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
    cJSON *root = cJSON_CreateObject();
    cJSON *rows = cJSON_CreateArray();
    uint32_t added = 0;
    uint32_t scanned = 0;
    esp_err_t err = sd_store_add_recent_readings(rows, after_id, before_id, limit, timeout_ms, &added, &scanned);

    cJSON_AddBoolToObject(root, "ok", err == ESP_OK);
    if (err == ESP_ERR_TIMEOUT) {
        cJSON_AddStringToObject(root, "error", "sd_scan_timeout");
    } else if (err != ESP_OK) {
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
    }
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddBoolToObject(root, "sd_ready", sd_store_is_ready());
    cJSON_AddNumberToObject(root, "after_id", after_id);
    cJSON_AddNumberToObject(root, "before_id", before_id);
    cJSON_AddNumberToObject(root, "last_id", sd_store_last_id());
    cJSON_AddNumberToObject(root, "boot_id", g_boot_id);
    cJSON_AddNumberToObject(root, "current_uptime_s", (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddNumberToObject(root, "requested_limit", limit);
    cJSON_AddNumberToObject(root, "timeout_ms", timeout_ms);
    cJSON_AddNumberToObject(root, "scanned", scanned);
    cJSON_AddNumberToObject(root, "count", added);
    cJSON_AddItemToObject(root, "rows", rows);

    char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, out);
    free(out);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t wifi_clear_delete(httpd_req_t *r) {
    httpd_resp_set_type(r, "text/plain");
    httpd_resp_sendstr(r, "Credenciales Wi-Fi borradas. El dispositivo se reiniciará en segundos.");
    wifi_store_clear();
    xTaskCreate(restart_later_task, "rst_later", 2048, NULL, 5, NULL);
    return ESP_OK;
}

static esp_err_t wifi_clear_get(httpd_req_t *r) {
    return wifi_clear_delete(r);
}

static esp_err_t readings_clear_delete(httpd_req_t *r) {
    esp_err_t err = sd_store_clear();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", err == ESP_OK);
    cJSON_AddStringToObject(root, "device_id", (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor");
    cJSON_AddBoolToObject(root, "sd_ready", sd_store_is_ready());
    cJSON_AddNumberToObject(root, "last_id", sd_store_last_id());
    if (err != ESP_OK) {
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
    }

    char *text = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_set_status(r, err == ESP_OK ? "200 OK" : "500 Internal Server Error");
    httpd_resp_sendstr(r, text);
    free(text);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t readings_clear_get(httpd_req_t *r) {
    return readings_clear_delete(r);
}

static esp_err_t config_post(httpd_req_t *r) {
    char buf[512];
    int len = httpd_req_recv(r, buf, sizeof(buf) - 1);
    if (len <= 0) return httpd_resp_send_err(r, 400, "empty");
    buf[len] = 0;

    cJSON *root = cJSON_Parse(buf);
    if (!root) return httpd_resp_send_err(r, 400, "json");

    cJSON *j_push_host = cJSON_GetObjectItem(root, "push_host");
    if (!cJSON_IsString(j_push_host)) {
        j_push_host = cJSON_GetObjectItem(root, "server_ip");
    }
    if (cJSON_IsString(j_push_host) && j_push_host->valuestring &&
        strlen(j_push_host->valuestring) < sizeof(g_push_host)) {
        snprintf(g_push_host, sizeof(g_push_host), "%s", j_push_host->valuestring);
        ESP_LOGI(TAG, "Push host configurado por servidor: %s", g_push_host);
    }

    esp_err_t err = ESP_OK;
    cJSON *j_epoch = cJSON_GetObjectItem(root, "epoch");
    if (!cJSON_IsNumber(j_epoch)) {
        j_epoch = cJSON_GetObjectItem(root, "unix_epoch");
    }
    cJSON *j_date = cJSON_GetObjectItem(root, "date");
    cJSON *j_time = cJSON_GetObjectItem(root, "time");
    if (!cJSON_IsString(j_date)) {
        j_date = cJSON_GetObjectItem(root, "fecha");
    }
    if (!cJSON_IsString(j_time)) {
        j_time = cJSON_GetObjectItem(root, "hora");
    }

    if (cJSON_IsNumber(j_epoch)) {
        err = captive_manager_offer_time_epoch((time_t)j_epoch->valuedouble, "server");
    } else if (cJSON_IsString(j_date) && cJSON_IsString(j_time)) {
        err = apply_device_time(j_date->valuestring, j_time->valuestring);
    } else {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 400, "epoch or date/time required");
    }

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 400, "invalid time");
    }

    cJSON *out = cJSON_CreateObject();
    cJSON_AddBoolToObject(out, "ok", true);
    cJSON_AddStringToObject(out, "device_id", (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor");
    cJSON_AddBoolToObject(out, "time_valid", g_time_valid);
    cJSON_AddNumberToObject(out, "current_epoch", g_time_valid ? (double)time(NULL) : 0.0);
    cJSON_AddStringToObject(out, "time_source", g_time_valid ? g_last_sync_source : "none");
    cJSON_AddBoolToObject(out, "time_applied", err == ESP_OK);
    cJSON_AddStringToObject(out, "date", g_config_date);
    cJSON_AddStringToObject(out, "time", g_config_time);
    char *text = cJSON_PrintUnformatted(out);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, text);
    free(text);
    cJSON_Delete(out);
    cJSON_Delete(root);
    ESP_LOGI(TAG, "Configuracion del servidor procesada; fuente=%s aplicada=%s",
             g_last_sync_source, err == ESP_OK ? "si" : "no");
    return ESP_OK;
}


static esp_err_t web_update_post(httpd_req_t *r) {
    if (!g_sta_have_ip) {
        return httpd_resp_send_err(r, 409, "network not ready");
    }
    if (!sd_store_is_ready()) {
        return httpd_resp_send_err(r, 409, "sd not ready");
    }
    if (r->content_len <= 0 || r->content_len >= 4096) {
        return httpd_resp_send_err(r, 400, "invalid payload size");
    }

    char *buf = calloc(1, r->content_len + 1);
    if (!buf) return httpd_resp_send_err(r, 500, "no memory");
    int total_read = 0;
    while (total_read < r->content_len) {
        int received = httpd_req_recv(r, buf + total_read, r->content_len - total_read);
        if (received <= 0) {
            free(buf);
            return httpd_resp_send_err(r, 400, "payload read failed");
        }
        total_read += received;
    }
    buf[total_read] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) return httpd_resp_send_err(r, 400, "json");

    cJSON *j_device_id = cJSON_GetObjectItem(root, "device_id");
    const char *expected_device_id = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
    if (cJSON_IsString(j_device_id) && strcmp(j_device_id->valuestring, expected_device_id) != 0) {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 400, "device_id mismatch");
    }

    cJSON *files = cJSON_GetObjectItem(root, "files");
    if (!cJSON_IsArray(files)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 400, "files array required");
    }

    clear_sd_web_dir();
    esp_err_t dir_err = ensure_sd_web_dir();
    if (dir_err != ESP_OK) {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 500, "sd web dir failed");
    }

    int saved = 0;
    int failed = 0;
    cJSON *details = cJSON_CreateArray();
    cJSON *item = NULL;
    cJSON_ArrayForEach(item, files) {
        cJSON *j_name = cJSON_GetObjectItem(item, "name");
        cJSON *j_url = cJSON_GetObjectItem(item, "url");
        if (!cJSON_IsString(j_name) || !cJSON_IsString(j_url)) {
            failed++;
            continue;
        }
        size_t bytes = 0;
        char err_detail[96] = {0};
        esp_err_t err = download_web_asset_to_sd(j_url->valuestring, j_name->valuestring, &bytes, err_detail, sizeof(err_detail));
        cJSON *row = cJSON_CreateObject();
        cJSON_AddStringToObject(row, "name", j_name->valuestring);
        cJSON_AddBoolToObject(row, "ok", err == ESP_OK);
        cJSON_AddNumberToObject(row, "bytes", (double)bytes);
        if (err != ESP_OK) {
            cJSON_AddStringToObject(row, "error", esp_err_to_name(err));
            if (err_detail[0]) cJSON_AddStringToObject(row, "detail", err_detail);
        }
        cJSON_AddItemToArray(details, row);
        if (err == ESP_OK) saved++; else failed++;
    }

    cJSON *out = cJSON_CreateObject();
    cJSON_AddBoolToObject(out, "ok", failed == 0);
    cJSON_AddStringToObject(out, "device_id", expected_device_id);
    cJSON_AddNumberToObject(out, "saved", saved);
    cJSON_AddNumberToObject(out, "failed", failed);
    cJSON_AddNumberToObject(out, "total", cJSON_GetArraySize(files));
    cJSON_AddItemToObject(out, "files", details);
    char *text = cJSON_PrintUnformatted(out);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_set_status(r, failed == 0 ? "200 OK" : "500 Internal Server Error");
    httpd_resp_sendstr(r, text);
    free(text);
    cJSON_Delete(out);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t ota_status_get(httpd_req_t *r) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device_id", (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor");
    ota_manager_add_status_json(root);
    char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, out);
    free(out);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t ota_update_post(httpd_req_t *r) {
    if (!g_sta_have_ip) {
        return httpd_resp_send_err(r, 409, "network not ready");
    }
    if (ota_manager_busy()) {
        return httpd_resp_send_err(r, 409, "ota already in progress");
    }
    if (r->content_len <= 0 || r->content_len >= 768) {
        return httpd_resp_send_err(r, 400, "invalid payload size");
    }

    char buf[768];
    int total = 0;
    while (total < r->content_len && total < (int)sizeof(buf) - 1) {
        int received = httpd_req_recv(r, buf + total, r->content_len - total);
        if (received <= 0) {
            return httpd_resp_send_err(r, 400, "payload read failed");
        }
        total += received;
    }
    buf[total] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        return httpd_resp_send_err(r, 400, "json");
    }
    cJSON *j_device_id = cJSON_GetObjectItem(root, "device_id");
    cJSON *j_version = cJSON_GetObjectItem(root, "version");
    cJSON *j_url = cJSON_GetObjectItem(root, "firmware_url");
    cJSON *j_sha = cJSON_GetObjectItem(root, "sha256");
    if (!cJSON_IsString(j_device_id) || !cJSON_IsString(j_version) || !cJSON_IsString(j_url)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 400, "device_id/version/firmware_url required");
    }

    const char *expected_device_id = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
    if (strcmp(j_device_id->valuestring, expected_device_id) != 0) {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 400, "device_id mismatch");
    }

    esp_err_t err = ota_manager_start(
        j_device_id->valuestring,
        j_version->valuestring,
        j_url->valuestring,
        cJSON_IsString(j_sha) ? j_sha->valuestring : ""
    );
    cJSON_Delete(root);
    if (err == ESP_ERR_INVALID_STATE) {
        return httpd_resp_send_err(r, 409, "ota already in progress");
    }
    if (err != ESP_OK) {
        return httpd_resp_send_err(r, 400, esp_err_to_name(err));
    }

    cJSON *out = cJSON_CreateObject();
    cJSON_AddBoolToObject(out, "ok", true);
    cJSON_AddStringToObject(out, "state", "queued");
    cJSON_AddStringToObject(out, "device_id", expected_device_id);
    cJSON_AddStringToObject(out, "current_version", ota_manager_current_version());
    char *text = cJSON_PrintUnformatted(out);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, text);
    free(text);
    cJSON_Delete(out);
    return ESP_OK;
}

static httpd_uri_t uri_favicon          = { .uri="/favicon.ico", .method=HTTP_GET,    .handler=favicon_get };
static httpd_uri_t uri_root             = { .uri="/",            .method=HTTP_GET,    .handler=root_get };
static httpd_uri_t uri_scan             = { .uri="/scan",        .method=HTTP_GET,    .handler=scan_get };
static httpd_uri_t uri_save             = { .uri="/save",        .method=HTTP_POST,   .handler=save_post };
static httpd_uri_t uri_status           = { .uri="/status",      .method=HTTP_GET,    .handler=status_get };
static httpd_uri_t uri_lecturas         = { .uri="/lecturas",    .method=HTTP_GET,    .handler=lecturas_get };
static httpd_uri_t uri_tabla            = { .uri="/tabla",       .method=HTTP_GET,    .handler=tabla_get };
static httpd_uri_t uri_api_latest       = { .uri="/api/latest",  .method=HTTP_GET,    .handler=lecturas_get };
static httpd_uri_t uri_web_style        = { .uri="/st.css",       .method=HTTP_GET,    .handler=sd_style_get };
static httpd_uri_t uri_web_script       = { .uri="/sc.js",       .method=HTTP_GET,    .handler=sd_script_get };
static httpd_uri_t uri_web_logo         = { .uri="/lct.png",     .method=HTTP_GET,    .handler=sd_logo_get };
static httpd_uri_t uri_lecturas_since   = { .uri="/lecturas/since", .method=HTTP_GET,  .handler=lecturas_since_get };
static httpd_uri_t uri_lecturas_range   = { .uri="/lecturas/range", .method=HTTP_GET, .handler=lecturas_range_get };
static httpd_uri_t uri_lecturas_export  = { .uri="/lecturas/export", .method=HTTP_GET, .handler=lecturas_export_get };
static httpd_uri_t uri_lecturas_recent  = { .uri="/lecturas/recent", .method=HTTP_GET, .handler=lecturas_recent_get };
static httpd_uri_t uri_config           = { .uri="/config",      .method=HTTP_POST,   .handler=config_post };
static httpd_uri_t uri_time             = { .uri="/time",        .method=HTTP_POST,   .handler=config_post };
static httpd_uri_t uri_wifi_clr         = { .uri="/wifi/clear",  .method=HTTP_DELETE, .handler=wifi_clear_delete };
static httpd_uri_t uri_wifi_clr_get     = { .uri="/wifi/clear",  .method=HTTP_GET,    .handler=wifi_clear_get };
static httpd_uri_t uri_readings_clr     = { .uri="/lecturas/clear", .method=HTTP_DELETE, .handler=readings_clear_delete };
static httpd_uri_t uri_readings_clr_get = { .uri="/lecturas/clear", .method=HTTP_GET,    .handler=readings_clear_get };
static httpd_uri_t uri_ota_update       = { .uri="/ota/update", .method=HTTP_POST, .handler=ota_update_post };
static httpd_uri_t uri_ota_status       = { .uri="/ota/status", .method=HTTP_GET, .handler=ota_status_get };
static httpd_uri_t uri_web_update       = { .uri="/web/update", .method=HTTP_POST, .handler=web_update_post };

static esp_err_t start_http(void) {
    if (g_server) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets = 7;
    cfg.max_uri_handlers = 30;
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;

    if (httpd_start(&g_server, &cfg) == ESP_OK) {
        httpd_register_uri_handler(g_server, &uri_root);
        httpd_register_uri_handler(g_server, &uri_favicon);
        httpd_register_uri_handler(g_server, &uri_scan);
        httpd_register_uri_handler(g_server, &uri_save);
        httpd_register_uri_handler(g_server, &uri_status);
        httpd_register_uri_handler(g_server, &uri_lecturas);
        httpd_register_uri_handler(g_server, &uri_tabla);
        httpd_register_uri_handler(g_server, &uri_api_latest);
        httpd_register_uri_handler(g_server, &uri_web_style);
        httpd_register_uri_handler(g_server, &uri_web_script);
        httpd_register_uri_handler(g_server, &uri_web_logo);
        httpd_register_uri_handler(g_server, &uri_lecturas_since);
        httpd_register_uri_handler(g_server, &uri_lecturas_range);
        httpd_register_uri_handler(g_server, &uri_lecturas_export);
        httpd_register_uri_handler(g_server, &uri_lecturas_recent);
        httpd_register_uri_handler(g_server, &uri_config);
        httpd_register_uri_handler(g_server, &uri_time);
        httpd_register_uri_handler(g_server, &uri_wifi_clr);
        httpd_register_uri_handler(g_server, &uri_wifi_clr_get);
        httpd_register_uri_handler(g_server, &uri_readings_clr);
        httpd_register_uri_handler(g_server, &uri_readings_clr_get);
        httpd_register_uri_handler(g_server, &uri_ota_update);
        httpd_register_uri_handler(g_server, &uri_ota_status);
        httpd_register_uri_handler(g_server, &uri_web_update);
        return ESP_OK;
    }

    return ESP_FAIL;
}

static void shutdown_ap_http(void) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(WIFI_MODE_STA));
}

static void start_mdns_service(void) {
    static bool mdns_started = false;
    if (mdns_started) return;

    if (mdns_init() == ESP_OK) {
        const char *hostname = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
        mdns_hostname_set(hostname);
        mdns_instance_name_set(hostname);
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        mdns_started = true;
        ESP_LOGI(TAG, "mDNS started as %s.local", hostname);
    } else {
        ESP_LOGW(TAG, "mDNS init failed");
    }
}

static void get_current_datetime_string(char *buf, size_t buf_size) {
    if (!buf || buf_size == 0) {
        return;
    }

    time_t now = 0;
    struct tm tm_now = {0};
    time(&now);
    localtime_r(&now, &tm_now);
    strftime(buf, buf_size, "%d-%m-%Y %H:%M:%S", &tm_now);
}

static bool parse_utc_timestamp(const char *timestamp, time_t *epoch_out) {
    if (!timestamp || !epoch_out || strlen(timestamp) < 20 || timestamp[19] != 'Z') {
        return false;
    }
    int year, month, day, hour, minute, second;
    if (sscanf(timestamp, "%4d-%2d-%2dT%2d:%2d:%2dZ",
               &year, &month, &day, &hour, &minute, &second) != 6) {
        return false;
    }
    struct tm tm_utc = {
        .tm_year = year - 1900,
        .tm_mon = month - 1,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = second,
        .tm_isdst = 0,
    };
    time_t epoch = utc_fields_to_epoch(year, month, day, hour, minute, second);
    struct tm check = {0};
    gmtime_r(&epoch, &check);
    if (epoch < (time_t)MIN_VALID_EPOCH || check.tm_year != tm_utc.tm_year ||
        check.tm_mon != tm_utc.tm_mon || check.tm_mday != tm_utc.tm_mday ||
        check.tm_hour != tm_utc.tm_hour || check.tm_min != tm_utc.tm_min ||
        check.tm_sec != tm_utc.tm_sec) {
        return false;
    }
    *epoch_out = epoch;
    return true;
}

static void get_active_ip_string(char *buf, size_t buf_size) {
    if (!buf || buf_size == 0) {
        return;
    }

    buf[0] = '\0';
    esp_netif_ip_info_t ip_info = {0};
    esp_err_t err = ESP_FAIL;

    if (g_sta_have_ip && g_sta_netif) {
        err = esp_netif_get_ip_info(g_sta_netif, &ip_info);
    } else if (g_ap_netif) {
        err = esp_netif_get_ip_info(g_ap_netif, &ip_info);
    }

    if (err == ESP_OK) {
        snprintf(buf,
                 buf_size,
                 IPSTR,
                 IP2STR(&ip_info.ip));
    }
}
