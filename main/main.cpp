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
#include "LVGL.hpp"

#include <stdexcept>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using namespace ESP32S3_FREENOVE_DEV_KIT;

static const char *TAG = "LVGL_PORT_TEST";

// LVGL image declare
LV_IMG_DECLARE(esp_logo)

static void _app_button_cb(lv_event_t *e)
{
    lv_display_t *lvgl_display = static_cast<lv_display_t *>(e->user_data);
    if (lvgl_display == nullptr) {
        ESP_LOGD(TAG, "Invalid LVGL pointer");
        return;
    }

    lv_disp_rot_t rotation = lv_disp_get_rotation(lvgl_display);
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
    lv_disp_set_rotation(lvgl_display, rotation);
}

static void app_main_display(const LVGL& lvgl)
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
    lv_obj_add_event_cb(btn, _app_button_cb, LV_EVENT_CLICKED, (void *)lvgl.getDisplay());

    /* Task unlock */
    lvgl_port_unlock();
}

extern "C" void app_main(void)
{
    try
    {
        auto lvgl = std::make_unique<LVGL>(std::make_unique<LCD>(), std::make_unique<Touch>()); 

        /* Show LVGL objects */
        app_main_display(*lvgl);

        while (true) {
            std::this_thread::sleep_for(20ms);
        }
    }
    catch (const std::exception &ex)
    {
        ESP_LOGD(TAG, "%s", ex.what());
    }
}
