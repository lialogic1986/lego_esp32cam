idf_component_register(
  SRCS
    "app_main.c"
    "conn_mgr.c"
    "camera_task.c"
    "rc522.c"
    "udp_tx.c"
    "rfid_task.c"
    "ws_client.c"
    "ota.c"
    "remote_console.c"
    "log_stream.c"
  INCLUDE_DIRS "."
  REQUIRES
        esp32-camera
        driver
        esp_netif
        esp_wifi
        nvs_flash
        esp_http_client
        esp_https_ota
        app_update
        esp_partition
        json
        esp_websocket_client  # добавишь, когда компонент реально подтянут
)
