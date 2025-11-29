#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"

#include "BoardInfo.h"
#include "LCD.hpp"
#include "Touch.hpp"

#include <stdexcept>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

static const char *TAG = "LVGL_PORT_TEST";

// LVGL image declare
LV_IMG_DECLARE(esp_logo)

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

static esp_err_t app_lvgl_init(const ESP32S3_FREENOVE_DEV_KIT::LCD &lcd, const ESP32S3_FREENOVE_DEV_KIT::Touch &touch)
{
    /* Initialize LVGL */
    lvgl_port_cfg_t lvgl_cfg = {};
    lvgl_cfg.task_priority = 4;       /* LVGL task priority */
    lvgl_cfg.task_stack = 4096;       /* LVGL task stack size */
    lvgl_cfg.task_affinity = -1;      /* LVGL task pinned to core (-1 is no affinity) */
    lvgl_cfg.task_max_sleep_ms = 500; /* Maximum sleep in LVGL task */
    lvgl_cfg.timer_period_ms = 5;     /* LVGL timer tick period in ms */
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    lvgl_port_display_cfg_t disp_cfg = {};
    disp_cfg.io_handle = lcd.getPanelIOHandle();
    disp_cfg.panel_handle = lcd.getPanelHandle();
    disp_cfg.buffer_size = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT;
    disp_cfg.double_buffer = LCD_DRAW_BUFF_DOUBLE;
    disp_cfg.hres = LCD_H_RES;
    disp_cfg.vres = LCD_V_RES;
    disp_cfg.monochrome = false;
    disp_cfg.rotation = {
        .swap_xy = false,
        .mirror_x = false,
        .mirror_y = false,
    };
    disp_cfg.flags.buff_dma = true;
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    /* Add touch input (for selected screen) */
    lvgl_port_touch_cfg_t touch_cfg = {};
    touch_cfg.disp = lvgl_disp;
    touch_cfg.handle = touch.getTouchHandle();
    lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);

    return ESP_OK;
}

static void _app_button_cb(lv_event_t *e)
{
    lv_disp_rot_t rotation = lv_disp_get_rotation(lvgl_disp);
    switch (rotation)
    {
    case LV_DISP_ROT_NONE:
        rotation = LV_DISP_ROT_90;
        break;
    case LV_DISP_ROT_90:
        rotation = LV_DISP_ROT_180;
        break;
    case LV_DISP_ROT_180:
        rotation = LV_DISP_ROT_270;
        break;
    case LV_DISP_ROT_270:
        rotation = LV_DISP_ROT_NONE;
        break;
    }

    /* LCD HW rotation */
    lv_disp_set_rotation(lvgl_disp, rotation);
}

static void app_main_display(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* Task lock */
    lvgl_port_lock(0);

    /* Your LVGL objects code here .... */

    /* Create image */
    lv_obj_t *img_logo = lv_img_create(scr);
    lv_img_set_src(img_logo, &esp_logo);
    lv_obj_align(img_logo, LV_ALIGN_TOP_MID, 0, 20);

    /* Label */
    lv_obj_t *label = lv_label_create(scr);
    lv_obj_set_width(label, LCD_H_RES);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_label_set_recolor(label, true);
    lv_label_set_text(label, "#FF0000 " LV_SYMBOL_BELL " Hello world" LV_SYMBOL_BELL "\n"
                             "#00FF00 " LV_SYMBOL_WARNING "Having Fun!" LV_SYMBOL_WARNING "\n"
                             "#0000FF Blue text #");

    lv_obj_align(label, LV_ALIGN_CENTER, 0, 20);

    /* Button */
    lv_obj_t *btn = lv_btn_create(scr);
    label = lv_label_create(btn);
    lv_label_set_text_static(label, "Rotate screen");
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_add_event_cb(btn, _app_button_cb, LV_EVENT_CLICKED, NULL);

    /* Task unlock */
    lvgl_port_unlock();
}

extern "C" void app_main(void)
{
    /* LCD HW initialization */
    try
    {
        ESP32S3_FREENOVE_DEV_KIT::LCD lcd{};

        ESP32S3_FREENOVE_DEV_KIT::Touch touch{};

        /* LVGL initialization */
        ESP_ERROR_CHECK(app_lvgl_init(lcd, touch));

        /* Show LVGL objects */
        app_main_display();

        while (true) {
            std::this_thread::sleep_for(20ms);
        }
    }
    catch (const std::exception &ex)
    {
        ESP_LOGD(TAG, "%s", ex.what());
    }
}
