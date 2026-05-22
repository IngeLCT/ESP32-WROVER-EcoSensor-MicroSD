idf_component_register(
    SRCS "main.c" "sensors.c"
    INCLUDE_DIRS "."
    REQUIRES captive_manager esp_event esp_wifi driver esp_http_client
)
