idf_component_register(
  SRCS "main.c"
  INCLUDE_DIRS "."
  PRIV_REQUIRES
    esp_event
    nvs_flash
    esp_netif
    esp_wifi
    esp_http_server
    esp32-camera
)