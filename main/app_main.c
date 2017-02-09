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

static const char* TAG = "camera_demo";

int state = 0;
const static char http_html_hdr[] =
        "HTTP/1.1 200 OK\r\nContent-type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n\r\n";

const static char http_section_begin[] = "Content-type: image/jpg\r\n\r\n";
const static char http_section_end[] = "--123456789000000000000987654321\r\n";

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;

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
            .ssid = "WiFi-Name",
            .password = "WiFi-Password",
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

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
            /* Send the HTML header
             * subtract 1 from the size, since we dont send the \0 in the string
             * NETCONN_NOCOPY: our data is const static, so no need to copy it
             */
            netconn_write(conn, http_html_hdr, sizeof(http_html_hdr) - 1,
                    NETCONN_NOCOPY);

            /* Send our HTML page */
            //netconn_write(conn, http_index_hml, sizeof(http_index_hml)-1, NETCONN_NOCOPY);
            int n = 10000;
            while (n--) {
                if (buf[5] == 's') {
                    err = netconn_write(conn, http_section_begin,
                            sizeof(http_section_begin) - 1, NETCONN_NOCOPY);
                    if (err != ERR_OK) {
                        break;
                    }
                }
                err = camera_run();
                if (err != ESP_OK) {
                    ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                    break;
                } else {
                    ESP_LOGD(TAG, "Done");
                    err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),
                            NETCONN_NOCOPY);
                    if (err != ERR_OK) {
                        break;
                    }
                    if (buf[5] == 's') {
                        err = netconn_write(conn, http_section_end,
                                sizeof(http_section_end) - 1, NETCONN_NOCOPY);
                        if (err != ERR_OK) {
                            break;
                        }
                    }
                    else {
                        break;
                    }
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
    nvs_flash_init();
    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = 4,
        .pin_d1 = 5,
        .pin_d2 = 18,
        .pin_d3 = 19,
        .pin_d4 = 36,
        .pin_d5 = 39,
        .pin_d6 = 34,
        .pin_d7 = 35,
        .pin_xclk = 21,
        .pin_pclk = 22,
        .pin_vsync = 25,
        .pin_href = 23,
        .pin_sscb_sda = 26,
        .pin_sscb_scl = 27,
        .pin_reset = 2,
        .xclk_freq_hz = 20000000,
        .pixel_format = CAMERA_PF_JPEG,
        .frame_size = CAMERA_FS_VGA,
        .jpeg_quality = 15
    };

    esp_err_t err = camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error = %d", err);
        return;
    }
    initialise_wifi();
    ESP_LOGI(TAG, "Free heap: %u\n", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Camera demo ready");
    ESP_LOGI(TAG, "open http://" IPSTR "/get for single frame", IP2STR(&s_ip_addr));
    ESP_LOGI(TAG, "open http://" IPSTR "/stream for multipart/x-mixed-replace stream (use with JPEGs)", IP2STR(&s_ip_addr));
    xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL);
}

