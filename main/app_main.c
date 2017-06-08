// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"
#include "camera.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "bitmap.h"

static const char* TAG = "camera_demo";

int state = 0;
const static char http_hdr[] = "HTTP/1.1 200 OK\r\n";
const static char http_stream_hdr[] = 
        "Content-type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n\r\n";
const static char http_jpg_hdr[] =
        "Content-type: image/jpg\r\n\r\n";
const static char http_pgm_hdr[] =
        "Content-type: image/x-portable-graymap\r\n\r\n";
const static char http_stream_boundary[] = "--123456789000000000000987654321\r\n";
const static char http_bitmap_hdr[] =
        "Content-type: image/bitmap\r\n\r\n";

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;
static camera_pixelformat_t s_pixel_format;

#define CAMERA_PIXEL_FORMAT CAMERA_PF_RGB565
#define CAMERA_FRAME_SIZE CAMERA_FS_QQVGA

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            s_ip_addr = event->event_info.got_ip.ip_info.ip;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
             auto-reassociate. */
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");

}
static void http_server_netconn_serve(struct netconn *conn)
{
    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;

    /* Read the data from the port, blocking if nothing yet there.
     We assume the request (the part we care about) is in one netbuf */
    err = netconn_recv(conn, &inbuf);

    if (err == ERR_OK) {
        netbuf_data(inbuf, (void**) &buf, &buflen);

        /* Is this an HTTP GET command? (only check the first 5 chars, since
         there are other formats for GET, and we're keeping it very simple )*/
        if (buflen >= 5 && buf[0] == 'G' && buf[1] == 'E' && buf[2] == 'T'
                && buf[3] == ' ' && buf[4] == '/') {
          /* Send the HTTP header
             * subtract 1 from the size, since we dont send the \0 in the string
             * NETCONN_NOCOPY: our data is const static, so no need to copy it
             */
            netconn_write(conn, http_hdr, sizeof(http_hdr) - 1,
                    NETCONN_NOCOPY);
            //check if a stream is requested.
            if (buf[5] == 's' || buf[5] == 'b') {
                //Send mjpeg stream header
                err = netconn_write(conn, http_stream_hdr, sizeof(http_stream_hdr) - 1,
                    NETCONN_NOCOPY);
                ESP_LOGD(TAG, "Stream started.");
                
                //Run while everyhting is ok and connection open.
                while(err == ERR_OK) {
                    ESP_LOGD(TAG, "Capture frame");
                    err = camera_run();
                    if (err != ESP_OK) {
                        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                    } else {
                        ESP_LOGD(TAG, "Done");
                        //Send jpeg header
                        if(s_pixel_format == CAMERA_PF_RGB565) {
                            err = netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1,
                                NETCONN_NOCOPY);
                            char *bmp = bmp_create_header(camera_get_fb_width(), camera_get_fb_height());
                            err = netconn_write(conn, bmp, sizeof(bitmap), NETCONN_NOCOPY);
                            free(bmp);
                        }
                        else
                            err = netconn_write(conn, http_jpg_hdr, sizeof(http_jpg_hdr) - 1,
                                NETCONN_NOCOPY);
                        //Send frame
                        err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),
                            NETCONN_NOCOPY);
                        if(err == ERR_OK)
                        {
                            //Send boundary to next jpeg
                            err = netconn_write(conn, http_stream_boundary,
                                    sizeof(http_stream_boundary) -1, NETCONN_NOCOPY);
                        }
                    }
                }
                ESP_LOGD(TAG, "Stream ended.");
            } else {
                if (s_pixel_format == CAMERA_PF_JPEG) {
                    netconn_write(conn, http_jpg_hdr, sizeof(http_jpg_hdr) - 1, NETCONN_NOCOPY);
                } else if (s_pixel_format == CAMERA_PF_GRAYSCALE) {
                    netconn_write(conn, http_pgm_hdr, sizeof(http_pgm_hdr) - 1, NETCONN_NOCOPY);
                    if (memcmp(&buf[5], "pgm", 3) == 0) {
                        char pgm_header[32];
                        snprintf(pgm_header, sizeof(pgm_header), "P5 %d %d %d\n", camera_get_fb_width(), camera_get_fb_height(), 255);
                        netconn_write(conn, pgm_header, strlen(pgm_header), NETCONN_COPY);
                    }
                }

                ESP_LOGD(TAG, "Image requested.");
                err = camera_run();
                if (err != ESP_OK) {
                    ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                } else {
                    ESP_LOGD(TAG, "Done");
                    //Send jpeg
                    err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),
                        NETCONN_NOCOPY);
                }
            }
        }
    }
    /* Close the connection (server closes in HTTP) */
    netconn_close(conn);

    /* Delete the buffer (netconn_recv gives us ownership,
     so we have to make sure to deallocate the buffer) */
    netbuf_delete(inbuf);
}

static void http_server(void *pvParameters)
{
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while (err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}

void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);

    nvs_flash_init();
    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = CONFIG_D0,
        .pin_d1 = CONFIG_D1,
        .pin_d2 = CONFIG_D2,
        .pin_d3 = CONFIG_D3,
        .pin_d4 = CONFIG_D4,
        .pin_d5 = CONFIG_D5,
        .pin_d6 = CONFIG_D6,
        .pin_d7 = CONFIG_D7,
        .pin_xclk = CONFIG_XCLK,
        .pin_pclk = CONFIG_PCLK,
        .pin_vsync = CONFIG_VSYNC,
        .pin_href = CONFIG_HREF,
        .pin_sscb_sda = CONFIG_SDA,
        .pin_sscb_scl = CONFIG_SCL,
        .pin_reset = CONFIG_RESET,
        .xclk_freq_hz = CONFIG_XCLK_FREQ,
    };

    camera_model_t camera_model;
    esp_err_t err = camera_probe(&config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        return;
    }
    if (camera_model == CAMERA_OV7725) {
        ESP_LOGI(TAG, "Detected OV7725 camera, using grayscale bitmap format");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        config.frame_size = CAMERA_FRAME_SIZE;
    } else if (camera_model == CAMERA_OV2640) {
        ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
        s_pixel_format = CAMERA_PF_JPEG;
        config.frame_size = CAMERA_FS_VGA;
        config.jpeg_quality = 15;
    } else {
        ESP_LOGE(TAG, "Camera not supported");
        return;
    }
    config.pixel_format = s_pixel_format;
    err = camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
    initialise_wifi();
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Camera demo ready");
    ESP_LOGI(TAG, "open http://" IPSTR "/get for single frame", IP2STR(&s_ip_addr));
    if (s_pixel_format == CAMERA_PF_GRAYSCALE) {
        ESP_LOGI(TAG, "open http://" IPSTR "/pgm for a single image/x-portable-graymap image", IP2STR(&s_ip_addr));
    }
    if (s_pixel_format == CAMERA_PF_RGB565) {
        ESP_LOGI(TAG, "open http://" IPSTR "/bitmap for multipart/x-mixed-replace stream (use with BITMAPs)", IP2STR(&s_ip_addr));
    }
    if (s_pixel_format == CAMERA_PF_JPEG) {
        ESP_LOGI(TAG, "open http://" IPSTR "/stream for multipart/x-mixed-replace stream (use with JPEGs)", IP2STR(&s_ip_addr));
    }
    xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL);
}

