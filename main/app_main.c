// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* component includes */
#include <stdio.h>

/* freertos includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_freertos_hooks.h"

#include "lvgl_gui.h"
#include "esp_log.h"
#include "spi_bus.h"

#include "lv_examples/src/lv_demo_printer/lv_demo_printer.h"
#include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"
#include "lv_examples/src/lv_ex_get_started/lv_ex_get_started.h"
#include "lv_examples/src/lv_demo_benchmark/lv_demo_benchmark.h"
#include "lv_examples/src/lv_demo_keypad_encoder/lv_demo_keypad_encoder.h"
#include "lv_examples/src/lv_demo_stress/lv_demo_stress.h"
#include "lv_examples/src/lv_ex_style/lv_ex_style.h"

static const char *TAG = "example_lvgl";

static spi_bus_handle_t board_spi_bus_init(void)
{ 
    /*spi总线初始化参数*/
    spi_config_t bus_conf = {
        .miso_io_num = -1, //主输入从输出 (=spi_q) 信号的 GPIO 引脚，如果不使用则为 -1
        .mosi_io_num = 13, //用于主输出从输入 (=spi_d) 信号的 GPIO 引脚，如果不使用则为 -1
        .sclk_io_num = 14, //SPI CLock 信号的 GPIO 引脚，如果不使用则为 -1
        .max_transfer_sz = 51200, //可发送的最大字节长度，如果< 4096，将设置4096
    };
    spi_bus_handle_t spi_bus_handle = spi_bus_create(SPI2_HOST, &bus_conf);
    ESP_LOGI(TAG, "spi2 bus create succeed");

    ESP_LOGW(TAG, "spi2 Init, but no spi2 io defined");

    return spi_bus_handle;
}

void app_main()
{
    spi_bus_handle_t spi2_bus = board_spi_bus_init();

    scr_driver_t lcd_drv;
    touch_panel_driver_t touch_drv;
    scr_interface_spi_config_t spi_lcd_cfg = {
        .spi_bus = spi2_bus,
        .pin_num_cs = 15,
        .pin_num_dc = 4,
        .clk_freq = 20000000,
        .swap_data = true,
    };

    scr_interface_driver_t *iface_drv;
    scr_interface_create(SCREEN_IFACE_SPI, &spi_lcd_cfg, &iface_drv);
     
    //屏幕控制器的配置
    scr_controller_config_t lcd_cfg = {
        .interface_drv = iface_drv,//屏幕接口驱动
        .pin_num_rst = 27,//固定到硬重置 LCD
        .pin_num_bckl = 26,//控制背光引脚
        .rst_active_level = 0,//复位引脚有效电平
        .bckl_active_level = 1,//背光激活级别
        .offset_hor = 0,//水平偏移
        .offset_ver = 0,//垂直偏移
        .width = 240,//宽
        .height = 240,//高
        .rotate = SCR_DIR_LRTB,屏幕旋转方向
    };
    scr_find_driver(SCREEN_CONTROLLER_ST7789, &lcd_drv);
    lcd_drv.init(&lcd_cfg);

    /* Initialize LittlevGL GUI */
    lvgl_init(&lcd_drv, NULL);

    // lvgl_acquire();
    // #ifdef CONFIG_LV_DEMO_BENCHMARK
    lv_demo_benchmark();
    // #elif defined CONFIG_LV_DEMO_PRINTER
    // lv_demo_printer();
    // #elif defined CONFIG_LV_DEMO_WIDGETS
    // lv_demo_widgets();
    // #elif defined CONFIG_LV_EX_GET_STARTED
    //     lv_ex_get_started_1();
    // #elif defined CONFIG_LV_DEMO_STRESS
    //     lv_demo_stress();
    // #elif defined CONFIG_LV_EX_STYLE
    // lv_ex_style_1();
    // #endif
    // lvgl_release();

    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
}
