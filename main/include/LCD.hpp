#pragma once

#include "esp_lcd_panel_io.h"

namespace ESP32S3_FREENOVE_DEV_KIT
{

    class LCD
    {
    public:
        LCD();
        ~LCD();

        esp_lcd_panel_io_handle_t getPanelIOHandle() const;
        esp_lcd_panel_handle_t getPanelHandle() const;

    private:
        esp_lcd_panel_io_handle_t lcd_io = nullptr;
        esp_lcd_panel_handle_t lcd_panel = nullptr;
    };

} // namespace ESP32S3_FREENOVE_DEV_KIT