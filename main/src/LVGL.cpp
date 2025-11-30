#include "LVGL.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include "LCD.hpp"
#include "Touch.hpp"

#include "BoardInfo.h"

#include <stdexcept>

namespace ESP32S3_FREENOVE_DEV_KIT
{
    static const char *TAG = "LVGL";

    LVGL::LVGL(std::unique_ptr<LCD> lcd, std::unique_ptr<Touch> touch) : m_lcd(std::move(lcd)), m_touch(std::move(touch))
    {
        lvgl_port_cfg_t lvgl_cfg = {};
        lvgl_cfg.task_priority = 4;       /* LVGL task priority */
        lvgl_cfg.task_stack = 4096;       /* LVGL task stack size */
        lvgl_cfg.task_affinity = -1;      /* LVGL task pinned to core (-1 is no affinity) */
        lvgl_cfg.task_max_sleep_ms = 500; /* Maximum sleep in LVGL task */
        lvgl_cfg.timer_period_ms = 5;     /* LVGL timer tick period in ms */
        if (lvgl_port_init(&lvgl_cfg) != ESP_OK)
        {
            ESP_LOGD(TAG, "LVGL port init failed");
            throw std::runtime_error("LVGL port init failed");
        }

        ESP_LOGD(TAG, "Add LCD screen");
        lvgl_port_display_cfg_t disp_cfg = {};
        disp_cfg.io_handle = m_lcd->getPanelIOHandle();
        disp_cfg.panel_handle = m_lcd->getPanelHandle();
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

        lvgl_port_touch_cfg_t touch_cfg = {};
        touch_cfg.disp = lvgl_disp;
        touch_cfg.handle = m_touch->getTouchHandle();
        lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);
    }

    LVGL::~LVGL()
    {
        if (lvgl_touch_indev != nullptr) {
            lvgl_port_remove_touch(lvgl_touch_indev);
            lvgl_touch_indev = nullptr;
        }

        if (lvgl_disp != nullptr) {
            lvgl_port_remove_disp(lvgl_disp);
            lvgl_disp = nullptr;
        }
    }

    lv_display_t* LVGL::getDisplay() const
    {
        return lvgl_disp;
    }

}