#include "wifi_store.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

#define WIFI_STORE_NS      "wifi_cfg"
#define WIFI_KEY_SSID      "ssid"
#define WIFI_KEY_PASS      "pass"
#define WIFI_KEY_IP_EN     "ip_en"
#define WIFI_KEY_STATIC_IP "static_ip"
#define WIFI_KEY_SUBNET    "subnet"
#define WIFI_KEY_GATEWAY   "gateway"
#define WIFI_KEY_DNS1      "dns1"
#define WIFI_KEY_DNS2      "dns2"

static esp_err_t ensure_nvs_open(nvs_handle_t *h, nvs_open_mode mode) {
    return nvs_open(WIFI_STORE_NS, mode, h);
}

static esp_err_t load_str_or_empty(nvs_handle_t h, const char *key, char *out, size_t out_len) {
    if (!out || out_len == 0) return ESP_ERR_INVALID_ARG;
    out[0] = 0;
    size_t len = out_len;
    esp_err_t err = nvs_get_str(h, key, out, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        out[0] = 0;
        return ESP_OK;
    }
    return err;
}

bool wifi_store_has_credentials(void) {
    nvs_handle_t h;
    if (ensure_nvs_open(&h, NVS_READONLY) != ESP_OK) return false;
    size_t len = 0;
    esp_err_t err = nvs_get_str(h, WIFI_KEY_SSID, NULL, &len);
    nvs_close(h);
    return err == ESP_OK && len > 0;
}

esp_err_t wifi_store_load(char *ssid, size_t ssid_len, char *pass, size_t pass_len) {
    if (!ssid || !pass) return ESP_ERR_INVALID_ARG;
    ssid[0] = 0;
    pass[0] = 0;

    nvs_handle_t h;
    esp_err_t err = ensure_nvs_open(&h, NVS_READONLY);
    if (err != ESP_OK) return err;

    size_t l1 = ssid_len;
    err = nvs_get_str(h, WIFI_KEY_SSID, ssid, &l1);
    if (err != ESP_OK) {
        nvs_close(h);
        return err;
    }

    size_t l2 = pass_len;
    err = nvs_get_str(h, WIFI_KEY_PASS, pass, &l2);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        pass[0] = 0;
        err = ESP_OK;
    }

    nvs_close(h);
    return err;
}

esp_err_t wifi_store_save(const char *ssid, const char *pass) {
    if (!ssid) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = ensure_nvs_open(&h, NVS_READWRITE);
    if (err != ESP_OK) return err;

    ESP_ERROR_CHECK(nvs_set_str(h, WIFI_KEY_SSID, ssid));
    ESP_ERROR_CHECK(nvs_set_str(h, WIFI_KEY_PASS, pass ? pass : ""));
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t wifi_store_save_static_ip_cfg(const wifi_static_ip_cfg_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = ensure_nvs_open(&h, NVS_READWRITE);
    if (err != ESP_OK) return err;

    ESP_ERROR_CHECK(nvs_set_u8(h, WIFI_KEY_IP_EN, cfg->enabled ? 1 : 0));

    if (cfg->enabled) {
        ESP_ERROR_CHECK(nvs_set_str(h, WIFI_KEY_STATIC_IP, cfg->ip));
        ESP_ERROR_CHECK(nvs_set_str(h, WIFI_KEY_GATEWAY, cfg->gateway));
    } else {
        nvs_erase_key(h, WIFI_KEY_STATIC_IP);
        nvs_erase_key(h, WIFI_KEY_GATEWAY);
    }

    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t wifi_store_load_static_ip_cfg(wifi_static_ip_cfg_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    memset(cfg, 0, sizeof(*cfg));

    nvs_handle_t h;
    esp_err_t err = ensure_nvs_open(&h, NVS_READONLY);
    if (err != ESP_OK) return err;

    uint8_t enabled = 0;
    err = nvs_get_u8(h, WIFI_KEY_IP_EN, &enabled);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        cfg->enabled = false;
        nvs_close(h);
        return ESP_OK;
    }
    if (err != ESP_OK) {
        nvs_close(h);
        return err;
    }

    cfg->enabled = enabled != 0;
    if (cfg->enabled) {
        if ((err = load_str_or_empty(h, WIFI_KEY_STATIC_IP, cfg->ip, sizeof(cfg->ip))) != ESP_OK ||
            (err = load_str_or_empty(h, WIFI_KEY_GATEWAY, cfg->gateway, sizeof(cfg->gateway))) != ESP_OK) {
            nvs_close(h);
            return err;
        }
    }

    nvs_close(h);
    return ESP_OK;
}

esp_err_t wifi_store_clear(void) {
    nvs_handle_t h;
    esp_err_t err = ensure_nvs_open(&h, NVS_READWRITE);
    if (err != ESP_OK) return err;

    nvs_erase_key(h, WIFI_KEY_SSID);
    nvs_erase_key(h, WIFI_KEY_PASS);
    nvs_erase_key(h, WIFI_KEY_IP_EN);
    nvs_erase_key(h, WIFI_KEY_STATIC_IP);
    nvs_erase_key(h, WIFI_KEY_GATEWAY);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}
