#include "esp_camera.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
//#include "mdns.h"

#define TAG "CAM"

#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

#define WIFI_SSID "csse4011" //"Attila MHS" //"infrastructure"//"Aussie Broadband 2278"
#define WIFI_PASS "csse4011wifi" //"egtx7544" //"9zLWjb73-gjF"//"yubemfhase"
// Event group to signal when connected or failed
static EventGroupHandle_t wifi_event_group;

// Bit flags for event group
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int retry_count = 0;
#define MAX_RETRIES 5

void init_camera() {
    camera_config_t camera_config = {
    .pin_pwdn       = 32,
    .pin_reset      = -1,   // No reset pin
    .pin_xclk       = 0,
    .pin_sscb_sda   = 26,
    .pin_sscb_scl   = 27,

    .pin_d7         = 35,
    .pin_d6         = 34,
    .pin_d5         = 39,
    .pin_d4         = 36,
    .pin_d3         = 21,
    .pin_d2         = 19,
    .pin_d1         = 18,
    .pin_d0         = 5,
    .pin_vsync      = 25,
    .pin_href       = 23,
    .pin_pclk       = 22,

    .xclk_freq_hz   = 15000000, //max is 240000000, try lower like 10000000 for low light confitions
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,
    .pixel_format   = PIXFORMAT_JPEG, // Supported: JPEG, RGB565, YUV422, GRAYSCALE
    .frame_size     = FRAMESIZE_VGA,
    .jpeg_quality   = 25, //0 to 63, lower is better quality //have it less then 10 and more than 5
    .fb_count       = 2,
    .fb_location    = CAMERA_FB_IN_PSRAM,
    .grab_mode      = CAMERA_GRAB_WHEN_EMPTY
    };

    //video settings, values between -2 to 2
    // sensor_t *s = esp_camera_sensor_get();
    // s->set_brightness(s, 1);
    // s->set_contrast(s, 2);
    // s->set_saturation(s, 1);
    // s->set_sharpness(s, 1);

    ESP_ERROR_CHECK(esp_camera_init(&camera_config));
    
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);  // 0 = normal, 1 = flipped vertically
    s->set_hmirror(s, 1);
}

void scan_wifi() {
    uint16_t ap_num = 0;
    wifi_ap_record_t ap_records[20];

    esp_wifi_scan_start(NULL, true);  // blocking scan
    esp_wifi_scan_get_ap_num(&ap_num);
    esp_wifi_scan_get_ap_records(&ap_num, ap_records);

    ESP_LOGI(TAG, "Scan complete. %d APs found:", ap_num);
    for (int i = 0; i < ap_num; i++) {
        ESP_LOGI(TAG, "SSID: %s, RSSI: %d", ap_records[i].ssid, ap_records[i].rssi);
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // WiFi started, try to connect
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Disconnected, retry if allowed
        if (retry_count < MAX_RETRIES) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Retrying to connect to the AP...");
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // Got IP — connection successful!
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

         // Start public IP fetch task after successful WiFi connection
    }
}

uint8_t wifi_init_sta(void) {
    // Create event group
    wifi_event_group = xEventGroupCreate();

    // Initialize networking stack
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta(); // Use default station (STA) interface

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Register WiFi and IP event handlers
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    // Configure WiFi credentials
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    // Set WiFi mode and configuration
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // Start WiFi
    esp_wifi_start();

    ESP_LOGI(TAG, "WiFi init complete, waiting for connection...");

    // Wait for either success or fail bit to be set
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(10000)); // 10 second timeout

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to WiFi");
        return 0;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect after %d retries", MAX_RETRIES);
    } else {
        ESP_LOGE(TAG, "Connection timed out");
    }

    return 1;

    // Optionally clean up event handlers here
}

esp_err_t stream_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=123456789000000000000987654321");
    static const char* boundary = "123456789000000000000987654321";
    char part_buf[128];
    esp_err_t res;

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW("stream", "Camera capture failed");
            continue;
        }

        // Compose MJPEG chunk header
        int header_len = snprintf(part_buf, sizeof(part_buf),
                                  "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                  boundary, fb->len);

        // Send header
        res = httpd_resp_send_chunk(req, part_buf, header_len);
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            ESP_LOGI("stream", "Client disconnected (header), exiting stream");
            break;
        }

        // Send JPEG image
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (res != ESP_OK) {
            ESP_LOGI("stream", "Client disconnected (image), exiting stream");
            break;
        }

        // Send chunk delimiter
        res = httpd_resp_send_chunk(req, "\r\n", 2);
        if (res != ESP_OK) {
            ESP_LOGI("stream", "Client disconnected (delimiter), exiting stream");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // ~10 FPS
    }

    return ESP_OK;
}

void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = NULL;
    httpd_start(&server, &config);

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &stream_uri);
}

void uart_stream_task(void *pvParameters) {
    const int uart_num = UART_NUM_1;  // or UART_NUM_0
    const int tx_pin = GPIO_NUM_1;   // e.g., GPIO1 (TX)
    const int rx_pin = GPIO_NUM_3;   // e.g., GPIO3 (RX)

    uart_config_t uart_config = {
        .baud_rate = 1152000, // Higher is better (check USB-UART bridge limits)
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(uart_num, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }

        // Send frame size (4 bytes, big endian)
        uint32_t size = fb->len;
        uint8_t size_buf[4] = {
            (size >> 24) & 0xFF,
            (size >> 16) & 0xFF,
            (size >> 8) & 0xFF,
            size & 0xFF
        };
        uart_write_bytes(uart_num, (const char *)size_buf, 4);

        // Send JPEG data
        uart_write_bytes(uart_num, (const char *)fb->buf, fb->len);

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(50)); // ~20 FPS
    }
}

void app_main() {
    ESP_LOGW(TAG, "Program Started\r\n");
    nvs_flash_init(); //works
    init_camera(); //works, do not touch much
    scan_wifi();
    if(wifi_init_sta()) { //works
        return;
    }
    //init_mdns();
    start_webserver(); //does not through errors lol?
    //xTaskCreate(uart_stream_task, "uart_stream_task", 4096, NULL, 5, NULL);
    ESP_LOGW(TAG, "Program Setup Ended\r\n");
}