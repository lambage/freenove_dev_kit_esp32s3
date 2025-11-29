#pragma once

#include "esp_lvgl_port_touch.h"

namespace ESP32S3_FREENOVE_DEV_KIT
{

    class Touch
    {
    public:
        Touch();
        ~Touch();

        esp_lcd_touch_handle_t getTouchHandle() const;
        
    private:
        esp_lcd_touch_handle_t touch_handle = nullptr;
    };

} // namespace ESP32S3_FREENOVE_DEV_KIT