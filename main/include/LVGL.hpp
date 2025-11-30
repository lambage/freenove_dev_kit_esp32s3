#pragma once

#include "esp_lvgl_port.h"

#include <memory>

namespace ESP32S3_FREENOVE_DEV_KIT
{

    class Touch;
    class LCD;

    class LVGL
    {
    public:
        LVGL(std::unique_ptr<LCD> lcd, std::unique_ptr<Touch> touch);
        ~LVGL();

        LVGL(LVGL &&) = default;
        LVGL& operator=(LVGL &&) = default;
        LVGL(const LVGL &) = delete;
        LVGL& operator=(const LVGL &) = delete;

        lv_display_t* getDisplay() const;

    private:
        std::unique_ptr<LCD> m_lcd;
        std::unique_ptr<Touch> m_touch;
        lv_display_t *lvgl_disp = nullptr;
        lv_indev_t *lvgl_touch_indev = nullptr;
    };

} // namespace ESP32S3_FREENOVE_DEV_KIT