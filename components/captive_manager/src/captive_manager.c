#include "captive_manager.h"
#include "wifi_store.h"
#include "sd_store.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mdns.h"
#include "lwip/ip4_addr.h"

#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <ctype.h>
#include <sys/time.h>

static const char *TAG = "wifi_manager";

static captive_manager_cfg_t g_cfg;
static captive_state_t g_state = CAP_STATE_IDLE;
static httpd_handle_t g_server = NULL;
static esp_netif_t *g_ap_netif = NULL;
static esp_netif_t *g_sta_netif = NULL;

static bool g_sta_have_ip = false;
static bool g_using_saved = false;
static bool g_sensors_started = false;
static bool g_time_valid = false;
static uint32_t g_boot_id = 0;
static char g_config_date[11] = {0};
static char g_config_time[9] = {0};
static char g_last_sync_source[16] = "none";
static int  g_connect_attempts = 0;
static int64_t g_boot_time_ms = 0;
static captive_manager_readings_t g_last_readings = {0};

static void restart_later_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

static void set_state(captive_state_t st);
static esp_err_t start_ap(void);
static esp_err_t start_http(void);
static void scan_start(void);
static void connect_sta(const char *ssid, const char *pass, bool from_saved);
static void shutdown_ap_http(void);
static void start_mdns_service(void);
static bool in_boot_grace_window(void);
static void start_with_saved_or_manager(void);
static esp_err_t parse_ipv4_or_fail(const char *text, esp_ip4_addr_t *out);
static esp_err_t apply_saved_sta_ip_config(void);
static bool validate_date_time(const char *date, const char *time_text);
static esp_err_t apply_device_time(const char *date, const char *time_text);
static void load_saved_device_time(void);
static void get_active_ip_string(char *buf, size_t buf_size);
static void get_current_datetime_string(char *buf, size_t buf_size);

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
    tm_value.tm_isdst = -1;

    time_t epoch = mktime(&tm_value);
    if (epoch < 0) return ESP_ERR_INVALID_ARG;

    struct timeval now = { .tv_sec = epoch, .tv_usec = 0 };
    if (settimeofday(&now, NULL) != 0) return ESP_FAIL;

    snprintf(g_config_date, sizeof(g_config_date), "%s", date);
    snprintf(g_config_time, sizeof(g_config_time), "%s", time_text);
    g_time_valid = true;
    snprintf(g_last_sync_source, sizeof(g_last_sync_source), "server");

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
    ESP_LOGI(TAG, "Fecha/hora requiere sincronizacion del servidor; sensores en espera hasta POST /time o POST /config");
}

esp_err_t captive_manager_init(const captive_manager_cfg_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    g_cfg = *cfg;
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
    g_connect_attempts = 0;
    ESP_LOGI(TAG, "(STA) Connecting to SSID=%s saved=%d", sta_cfg.sta.ssid, from_saved);
    ESP_ERROR_CHECK(esp_wifi_connect());
    start_mdns_service();
}

void captive_manager_notify_sta_got_ip(void) {
    g_sta_have_ip = true;
    g_connect_attempts = 0;
    shutdown_ap_http();
    esp_err_t http_err = start_http();
    if (http_err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar HTTP en STA: %s", esp_err_to_name(http_err));
    }
    set_state(CAP_STATE_OPERATIONAL);
    ESP_LOGI(TAG, "STA connected and operational; HTTP endpoints available on port 80");
}

void captive_manager_notify_sta_disconnected(int reason_code) {
    ESP_LOGW(TAG, "STA disconnected (reason=%d) state=%s saved=%d attempts=%d",
             reason_code, captive_manager_state_str(g_state), g_using_saved, g_connect_attempts);
    g_sta_have_ip = false;

    if (g_state == CAP_STATE_CONNECTING || g_state == CAP_STATE_OPERATIONAL) {
        g_connect_attempts++;
        bool in_grace = in_boot_grace_window();
        bool under_limit = (g_connect_attempts < g_cfg.conn_max_attempts);

        if (under_limit || in_grace) {
            vTaskDelay(pdMS_TO_TICKS(g_cfg.conn_retry_delay_ms));
            esp_wifi_connect();
            return;
        }

        ESP_LOGW(TAG, "Sin conexión tras reintentos. Volviendo al Wi-Fi manager");
        captive_manager_enter_recaptive();
    }
}

esp_err_t captive_manager_enter_recaptive(void) {
    ESP_LOGW(TAG, "Entering Wi-Fi manager mode");
    shutdown_ap_http();
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_disconnect());
    g_sta_have_ip = false;
    g_using_saved = false;
    g_connect_attempts = 0;

    set_state(CAP_STATE_RECAPTIVE);
    ESP_ERROR_CHECK(start_ap());
    if (!g_server) {
        ESP_ERROR_CHECK(start_http());
    }
    set_state(CAP_STATE_SCAN);
    scan_start();
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
        "<h1>Wi-Fi Manager</h1>"
        "<form id=\"wifiForm\">"
        "<label for=\"ssid\">SSID</label>"
        "<select id=\"ssid\" name=\"ssid\" required><option value=\"\" disabled selected>Cargando redes...</option></select>"
        "<label for=\"pass\">Contraseña</label>"
        "<input type=\"password\" id=\"pass\" name=\"pass\">"
        "<div class=\"row-check\"><input type=\"checkbox\" id=\"use_static_ip\" name=\"use_static_ip\"><label for=\"use_static_ip\">Configurar IP manual</label><span style=\"font-size:12px;color:#555\">Si al conectar la red, el servidor no puede establecer conexion con el pruebe configurando una IP fija.</span></div>"
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

    return httpd_resp_send(r, page, HTTPD_RESP_USE_STRLEN);
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
    } else if (g_state == CAP_STATE_CONNECTING || g_state == CAP_STATE_VERIFY || g_state == CAP_STATE_WAIT_LOGIN) {
        wifi = "connecting";
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddStringToObject(root, "wifi", wifi);
    cJSON_AddStringToObject(root, "ip", ip_buf);
    cJSON_AddStringToObject(root, "mdns", mdns_name);
    cJSON_AddBoolToObject(root, "sd_ready", sd_store_is_ready());
    cJSON_AddNumberToObject(root, "sd_last_id", sd_store_last_id());
    cJSON_AddNumberToObject(root, "boot_id", g_boot_id);
    cJSON_AddNumberToObject(root, "current_uptime_s", (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddBoolToObject(root, "time_valid", g_time_valid);
    cJSON_AddBoolToObject(root, "needs_time_sync", !g_time_valid);
    cJSON_AddStringToObject(root, "last_sync_source", g_last_sync_source);
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
    cJSON_AddStringToObject(root, "state", captive_manager_state_str(g_state));
    cJSON_AddBoolToObject(root, "using_saved", g_using_saved);
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
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddNumberToObject(root, "boot_id", g_boot_id);
    cJSON_AddNumberToObject(root, "current_uptime_s", (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddBoolToObject(root, "valid", g_last_readings.valid);
    cJSON_AddNumberToObject(root, "window_s", g_last_readings.window_s);

    if (!g_last_readings.valid) {
        cJSON_AddBoolToObject(root, "time_valid", g_time_valid);
        cJSON_AddStringToObject(root, "message", "Sin lecturas promediadas disponibles");
    } else {
        cJSON_AddNumberToObject(root, "id", g_last_readings.measurement_id);
        cJSON_AddNumberToObject(root, "measurement_id", g_last_readings.measurement_id);
        cJSON_AddNumberToObject(root, "boot_id", g_last_readings.boot_id);
        cJSON_AddNumberToObject(root, "uptime_s", g_last_readings.uptime_s);
        cJSON_AddBoolToObject(root, "time_valid", g_last_readings.time_valid);
        cJSON_AddStringToObject(root, "time_source", g_last_readings.time_valid ? "esp" : "pending_estimate");
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
    }

    char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, out);
    free(out);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t lecturas_since_get(httpd_req_t *r) {
    char query[96] = {0};
    uint32_t after_id = 0;
    uint32_t limit = 500;

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
    }

    const char *device_id = (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor";
    cJSON *root = cJSON_CreateObject();
    cJSON *rows = cJSON_CreateArray();
    uint32_t added = 0;
    esp_err_t err = sd_store_add_readings_since(rows, after_id, limit, &added);

    cJSON_AddBoolToObject(root, "ok", err == ESP_OK);
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddBoolToObject(root, "sd_ready", sd_store_is_ready());
    cJSON_AddNumberToObject(root, "after_id", after_id);
    cJSON_AddNumberToObject(root, "last_id", sd_store_last_id());
    cJSON_AddNumberToObject(root, "boot_id", g_boot_id);
    cJSON_AddNumberToObject(root, "current_uptime_s", (double)(esp_timer_get_time() / 1000000ULL));
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
    char buf[256];
    int len = httpd_req_recv(r, buf, sizeof(buf) - 1);
    if (len <= 0) return httpd_resp_send_err(r, 400, "empty");
    buf[len] = 0;

    cJSON *root = cJSON_Parse(buf);
    if (!root) return httpd_resp_send_err(r, 400, "json");

    cJSON *j_date = cJSON_GetObjectItem(root, "date");
    cJSON *j_time = cJSON_GetObjectItem(root, "time");
    if (!cJSON_IsString(j_date)) {
        j_date = cJSON_GetObjectItem(root, "fecha");
    }
    if (!cJSON_IsString(j_time)) {
        j_time = cJSON_GetObjectItem(root, "hora");
    }

    if (!cJSON_IsString(j_date) || !cJSON_IsString(j_time)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 400, "date/time required");
    }

    esp_err_t err = apply_device_time(j_date->valuestring, j_time->valuestring);
    if (err != ESP_OK) {
        cJSON_Delete(root);
        return httpd_resp_send_err(r, 400, "invalid date/time");
    }

    cJSON *out = cJSON_CreateObject();
    cJSON_AddBoolToObject(out, "ok", true);
    cJSON_AddStringToObject(out, "device_id", (g_cfg.mdns_hostname && g_cfg.mdns_hostname[0]) ? g_cfg.mdns_hostname : "ecosensor");
    cJSON_AddBoolToObject(out, "time_valid", g_time_valid);
    cJSON_AddStringToObject(out, "date", g_config_date);
    cJSON_AddStringToObject(out, "time", g_config_time);
    char *text = cJSON_PrintUnformatted(out);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, text);
    free(text);
    cJSON_Delete(out);
    cJSON_Delete(root);
    ESP_LOGI(TAG, "Fecha/hora configurada por servidor: %s %s", g_config_date, g_config_time);
    return ESP_OK;
}

static httpd_uri_t uri_favicon          = { .uri="/favicon.ico", .method=HTTP_GET,    .handler=favicon_get };
static httpd_uri_t uri_root             = { .uri="/",            .method=HTTP_GET,    .handler=root_get };
static httpd_uri_t uri_scan             = { .uri="/scan",        .method=HTTP_GET,    .handler=scan_get };
static httpd_uri_t uri_save             = { .uri="/save",        .method=HTTP_POST,   .handler=save_post };
static httpd_uri_t uri_status           = { .uri="/status",      .method=HTTP_GET,    .handler=status_get };
static httpd_uri_t uri_lecturas         = { .uri="/lecturas",    .method=HTTP_GET,    .handler=lecturas_get };
static httpd_uri_t uri_lecturas_since   = { .uri="/lecturas/since", .method=HTTP_GET,  .handler=lecturas_since_get };
static httpd_uri_t uri_config           = { .uri="/config",      .method=HTTP_POST,   .handler=config_post };
static httpd_uri_t uri_time             = { .uri="/time",        .method=HTTP_POST,   .handler=config_post };
static httpd_uri_t uri_wifi_clr         = { .uri="/wifi/clear",  .method=HTTP_DELETE, .handler=wifi_clear_delete };
static httpd_uri_t uri_wifi_clr_get     = { .uri="/wifi/clear",  .method=HTTP_GET,    .handler=wifi_clear_get };
static httpd_uri_t uri_readings_clr     = { .uri="/lecturas/clear", .method=HTTP_DELETE, .handler=readings_clear_delete };
static httpd_uri_t uri_readings_clr_get = { .uri="/lecturas/clear", .method=HTTP_GET,    .handler=readings_clear_get };

static esp_err_t start_http(void) {
    if (g_server) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets = 2;
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;

    if (httpd_start(&g_server, &cfg) == ESP_OK) {
        httpd_register_uri_handler(g_server, &uri_root);
        httpd_register_uri_handler(g_server, &uri_favicon);
        httpd_register_uri_handler(g_server, &uri_scan);
        httpd_register_uri_handler(g_server, &uri_save);
        httpd_register_uri_handler(g_server, &uri_status);
        httpd_register_uri_handler(g_server, &uri_lecturas);
        httpd_register_uri_handler(g_server, &uri_lecturas_since);
        httpd_register_uri_handler(g_server, &uri_config);
        httpd_register_uri_handler(g_server, &uri_time);
        httpd_register_uri_handler(g_server, &uri_wifi_clr);
        httpd_register_uri_handler(g_server, &uri_wifi_clr_get);
        httpd_register_uri_handler(g_server, &uri_readings_clr);
        httpd_register_uri_handler(g_server, &uri_readings_clr_get);
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
