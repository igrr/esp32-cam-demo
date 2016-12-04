/*
 * Portions of this file come from OpenMV project (see sensor_* functions in the end of file)
 * Here is the copyright for these parts:
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 *
 * Rest of the functions are licensed under Apache license as found below:
 */

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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/soc.h"
#include "sccb.h"
#include "wiring.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/io_mux_reg.h"
#include "sensor.h"
#include "ov7725.h"
#include <stdlib.h>
#include <string.h>
#include "rom/lldesc.h"
#include "esp_intr.h"
#include "camera.h"
#include "esp_log.h"
#include "driver/periph_ctrl.h"

# define ENABLE_TEST_PATTERN CONFIG_ENABLE_TEST_PATTERN

static const char* TAG = "camera";

static camera_config_t s_config;
static lldesc_t s_dma_desc[2];
static uint32_t* s_dma_buf[2];
static uint8_t* s_fb;
static sensor_t s_sensor;
static bool s_initialized = false;
static int s_fb_w;
static int s_fb_h;
static size_t s_fb_size;

static volatile int isr_count = 0;
static volatile int line_count = 0;
static volatile int cur_buffer = 0;
static int buf_line_width;
static int buf_height;
static volatile bool i2s_running = 0;
static SemaphoreHandle_t data_ready;
static SemaphoreHandle_t frame_ready;


const int resolution[][2] = {
    {40,    30 },    /* 40x30 */
    {64,    32 },    /* 64x32 */
    {64,    64 },    /* 64x64 */
    {88,    72 },    /* QQCIF */
    {160,   120},    /* QQVGA */
    {128,   160},    /* QQVGA2*/
    {176,   144},    /* QCIF  */
    {240,   160},    /* HQVGA */
    {320,   240},    /* QVGA  */
    {352,   288},    /* CIF   */
    {640,   480},    /* VGA   */
    {800,   600},    /* SVGA  */
    {1280,  1024},   /* SXGA  */
    {1600,  1200},   /* UXGA  */
};


static void i2s_init();
static void i2s_run(size_t line_width, int height);
static void IRAM_ATTR i2s_isr(void* arg);
static esp_err_t dma_desc_init(int line_width);
static void line_filter_task(void *pvParameters);


static void enable_out_clock() {
    periph_module_enable(PERIPH_LEDC_MODULE);

    ledc_timer_config_t timer_conf;
    timer_conf.bit_num = 3;
    timer_conf.freq_hz = s_config.xclk_freq_hz;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = s_config.ledc_timer;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
    }

    ledc_channel_config_t ch_conf;
    ch_conf.channel = s_config.ledc_channel;
    ch_conf.timer_sel = s_config.ledc_timer;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.duty = 4;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.gpio_num = s_config.pin_xclk;
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
    }
}

esp_err_t camera_init(const camera_config_t* config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(&s_config, config, sizeof(s_config));
    ESP_LOGD(TAG, "Enabling XCLK output");
    enable_out_clock();
    ESP_LOGD(TAG, "Initializing SSCB");
    SCCB_Init(s_config.pin_sscb_sda, s_config.pin_sscb_scl);
    ESP_LOGD(TAG, "Resetting camera");
    gpio_set_level(s_config.pin_reset, 0);
    delay(1);
    gpio_set_level(s_config.pin_reset, 1);
    delay(10);
    ESP_LOGD(TAG, "Searching for camera address");
    uint8_t addr = SCCB_Probe();
    if (addr == 0) {
        ESP_LOGE(TAG, "Camera address not found");
        return ESP_ERR_CAMERA_NOT_DETECTED;
    }
    ESP_LOGD(TAG, "Detected camera at address=0x%02x", addr);
    s_sensor.slv_addr = addr;

    ov7725_init(&s_sensor);
    ESP_LOGD(TAG, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
           s_sensor.id.pid, s_sensor.id.ver, s_sensor.id.midh, s_sensor.id.midl);

    ESP_LOGD(TAG, "Doing SW reset of sensor");
    s_sensor.reset(&s_sensor);

#if ENABLE_TEST_PATTERN
    /* Test pattern may get handy
       if you are unable to get the live image right.
       Once test pattern is enable, sensor will output
       vertical shaded bars instead of live image.
    */
    s_sensor.set_colorbar(&s_sensor, 1);
    ESP_LOGD(TAG, "Test pattern enabled");
#endif

    framesize_t framesize = FRAMESIZE_QVGA;
    s_fb_w = resolution[framesize][0];
    s_fb_h = resolution[framesize][1];
    ESP_LOGD(TAG, "Setting frame size at %dx%d", s_fb_w, s_fb_h);
    if (s_sensor.set_framesize(&s_sensor, framesize) != 0) {
        ESP_LOGE(TAG, "Failed to set frame size");
        return ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
    }

    s_fb_size = s_fb_w * s_fb_h;
    ESP_LOGD(TAG, "Allocating frame buffer (%dx%d, %d bytes)", s_fb_w, s_fb_h, s_fb_size);
    s_fb = (uint8_t*) malloc(s_fb_size);
    if (s_fb == NULL) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGD(TAG, "Initializing I2S and DMA");
    i2s_init();
    esp_err_t err = dma_desc_init(s_fb_w);
    if (err != ESP_OK) {
        free(s_fb);
        return err;
    }

    data_ready = xSemaphoreCreateBinary();
    frame_ready = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(&line_filter_task, "line_filter", 2048, NULL, 10, NULL, 0);
    ESP_LOGD(TAG, "Init done");

    s_initialized = true;
    return ESP_OK;
}

uint8_t* camera_get_fb()
{
    if (!s_initialized) {
        return NULL;
    }
    return s_fb;
}

int camera_get_fb_width()
{
    if (!s_initialized) {
        return 0;
    }
    return s_fb_w;
}

int camera_get_fb_height()
{
    if (!s_initialized) {
        return 0;
    }
    return s_fb_h;
}

esp_err_t camera_run()
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    i2s_run(s_fb_w, s_fb_h);
    ESP_LOGD(TAG, "Waiting for frame");
    xSemaphoreTake(frame_ready, portMAX_DELAY);
    ESP_LOGD(TAG, "Frame done");
    return ESP_OK;
}

void camera_print_fb()
{
	/* Number of pixels to skip
	   in order to fit into terminal screen.
	   Assumed picture to be 80 columns wide.
	   Skip twice as more rows as they look higher.
	 */
	int pixels_to_skip = s_fb_w / 80;

    for (int ih = 0; ih < s_fb_h; ih += pixels_to_skip * 2){
        for (int iw = 0; iw < s_fb_w; iw += pixels_to_skip){
    	    uint8_t px = (s_fb[iw + (ih * s_fb_w)]);
    	    if      (px <  26) printf(" ");
    	    else if (px <  51) printf(".");
    	    else if (px <  77) printf(":");
    	    else if (px < 102) printf("-");
    	    else if (px < 128) printf("=");
    	    else if (px < 154) printf("+");
    	    else if (px < 179) printf("*");
    	    else if (px < 205) printf("#");
    	    else if (px < 230) printf("%%");
    	    else               printf("@");
        }
        printf("\n");
    }
}

static esp_err_t dma_desc_init(int line_width)
{
	/* I2S peripheral captures 16 bit of data every clock cycle,
	   even though we are only using 8 bits.
	   On top of that we need two bytes per pixel.
	 */
    size_t buf_size = line_width * 4;
    for (int i = 0; i < 2; ++i) {
        ESP_LOGD(TAG, "Allocating DMA buffer #%d, size=%d", i, buf_size);
        s_dma_buf[i] = (uint32_t*) malloc(buf_size);
        if (s_dma_buf[i] == NULL) {
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGV(TAG, "dma_buf[%d]=%p\n", i, s_dma_buf[i]);

        s_dma_desc[i].length = buf_size;     // size of a single DMA buf
        s_dma_desc[i].size = buf_size;       // total size of the chain
        s_dma_desc[i].owner = 1;
        s_dma_desc[i].sosf = 1;
        s_dma_desc[i].buf = (uint8_t*) s_dma_buf[i];
        s_dma_desc[i].offset = i;
        s_dma_desc[i].empty = 0;
        s_dma_desc[i].eof = 1;
        s_dma_desc[i].qe.stqe_next = NULL;
    }
    return ESP_OK;
}

#define ETS_I2S0_INUM 13

static void i2s_init()
{
    xt_set_interrupt_handler(ETS_I2S0_INUM, &i2s_isr, NULL);
    intr_matrix_set(0, ETS_I2S0_INTR_SOURCE, ETS_I2S0_INUM);

    gpio_num_t pins[] = {
            s_config.pin_d7,
            s_config.pin_d6,
            s_config.pin_d5,
            s_config.pin_d4,
            s_config.pin_d3,
            s_config.pin_d2,
            s_config.pin_d1,
            s_config.pin_d0,
            s_config.pin_vsync,
            s_config.pin_href,
            s_config.pin_pclk
    };

    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < sizeof(pins)/sizeof(gpio_num_t); ++i) {
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

    gpio_matrix_in(s_config.pin_d0,    I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(s_config.pin_d1,    I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(s_config.pin_d2,    I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(s_config.pin_d3,    I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(s_config.pin_d4,    I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(s_config.pin_d5,    I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(s_config.pin_d6,    I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(s_config.pin_d7,    I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(s_config.pin_vsync, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(s_config.pin_href,  I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(s_config.pin_pclk,  I2S0I_WS_IN_IDX, false);

    periph_module_enable(PERIPH_I2S0_MODULE);

    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 1, I2S_IN_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 0, I2S_IN_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 1, I2S_AHBM_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 0, I2S_AHBM_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 1, I2S_AHBM_FIFO_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 0, I2S_AHBM_FIFO_RST_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_FIFO_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_FIFO_RESET_S);

    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_SLAVE_MOD_S);
    SET_PERI_REG_BITS(I2S_CONF2_REG(0), 0x1, 1, I2S_LCD_EN_S);
    SET_PERI_REG_BITS(I2S_CONF2_REG(0), 0x1, 1, I2S_CAMERA_EN_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_A, 1, I2S_CLKM_DIV_A_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_B, 0, I2S_CLKM_DIV_B_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_NUM, 2, I2S_CLKM_DIV_NUM_S);

    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), 0x1, 1, I2S_DSCR_EN_S);

    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_RIGHT_FIRST_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_MSB_RIGHT_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_MSB_SHIFT_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_MONO_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_SHORT_SYNC_S);
    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), I2S_RX_FIFO_MOD, 1, I2S_RX_FIFO_MOD_S);
    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), 0x1, 1, I2S_RX_FIFO_MOD_FORCE_EN_S);
    SET_PERI_REG_BITS(I2S_CONF_CHAN_REG(0), I2S_RX_CHAN_MOD, 1, I2S_RX_CHAN_MOD_S);
    SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_RX_BITS_MOD, 16, I2S_RX_BITS_MOD_S);

}

static void i2s_fill_buf(int index) {
    ESP_INTR_DISABLE(ETS_I2S0_INUM);

    SET_PERI_REG_BITS(I2S_RXEOF_NUM_REG(0), I2S_RX_EOF_NUM, (buf_line_width - 2) * 2, I2S_RX_EOF_NUM_S);
    SET_PERI_REG_BITS(I2S_IN_LINK_REG(0), I2S_INLINK_ADDR, ((uint32_t) &s_dma_desc), I2S_INLINK_ADDR_S);
    SET_PERI_REG_BITS(I2S_IN_LINK_REG(0), 0x1, 1, I2S_INLINK_START_S);

    REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG(0)) & 0xffffffc0) | 0x3f);

    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), (REG_READ(I2S_CONF_REG(0)) & 0xfffffff0) | 0xf);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);
    while (GET_PERI_REG_BITS2(I2S_STATE_REG(0), 0x1, I2S_TX_FIFO_RESET_BACK_S));

    SET_PERI_REG_BITS(I2S_INT_ENA_REG(0), 0x1, 1, I2S_IN_DONE_INT_ENA_S);
    ESP_INTR_ENABLE(ETS_I2S0_INUM);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_START_S);
}

static void i2s_stop() {
    ESP_INTR_DISABLE(ETS_I2S0_INUM);

    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), (REG_READ(I2S_CONF_REG(0)) & 0xfffffff0) | 0xf);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);

    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_START_S);
    i2s_running = false;
}

static void i2s_run(size_t line_width, int height)
{
    buf_line_width = line_width;
    buf_height = height;

    // wait for vsync
    ESP_LOGD(TAG, "Waiting for VSYNC");
    while(gpio_get_level(s_config.pin_vsync) != 0);
    while(gpio_get_level(s_config.pin_vsync) == 0);
    ESP_LOGD(TAG, "Got VSYNC");
    // wait a bit
    delay(2);

    // start RX
    cur_buffer = 0;
    line_count = 0;
    isr_count = 0;
    i2s_running = true;
    i2s_fill_buf(cur_buffer);
}

static void IRAM_ATTR i2s_isr(void* arg) {
    REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG(0)) & 0xffffffc0) | 0x3f);
    cur_buffer = !cur_buffer;
    if (isr_count == buf_height - 2) {
        i2s_stop();
    }
    else {
        i2s_fill_buf(cur_buffer);
        ++isr_count;
    }
    static BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(data_ready, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

static void line_filter_task(void *pvParameters) {
    static int prev_buf = -1;
    while (true) {
        xSemaphoreTake(data_ready, portMAX_DELAY);
        int buf_idx = !cur_buffer;
        if (prev_buf != -1 && prev_buf == buf_idx) {
            ets_printf("! %d\n", line_count);
        }
        uint8_t* pfb = s_fb + line_count * buf_line_width;
        if (line_count & 1) {
            uint8_t* psrc = s_fb + (line_count - 1) * buf_line_width;
            memcpy(pfb, psrc, buf_line_width);
        }
        else {
            const uint32_t* buf = s_dma_buf[buf_idx];
            for (int i = 0; i < buf_line_width; ++i) {
                uint32_t v = *buf;
                uint8_t comp = (v & 0xff0000) >> 16;
                *pfb = comp;
                ++buf;
                ++pfb;
            }
        }
        ++line_count;
        prev_buf = buf_idx;
        if (!i2s_running) {
            prev_buf = -1;
            xSemaphoreGive(frame_ready);
        }
    }
}
