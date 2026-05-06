idf_component_register(
    SRCS "src/captive_manager.c" "src/wifi_store.c"
    INCLUDE_DIRS "." "include"
    REQUIRES esp_wifi esp_event esp_http_server esp_netif nvs_flash esp_timer json mdns
)
