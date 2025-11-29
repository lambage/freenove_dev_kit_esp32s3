#include "Touch.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_lcd_touch_ft6x36.h"
#include "driver/i2c_master.h"

#include "BoardInfo.h"

#include <stdexcept>

namespace ESP32S3_FREENOVE_DEV_KIT
{
    static const char *TAG = "Touch";

    Touch::Touch()
    {
        i2c_master_bus_handle_t i2c_handle = NULL;
        i2c_master_bus_config_t i2c_config = {};
        i2c_config.i2c_port = TOUCH_IIC_NUM;
        i2c_config.sda_io_num = IIC_SDA_PIN;
        i2c_config.scl_io_num = IIC_SCL_PIN;
        i2c_config.clk_source = I2C_CLK_SRC_DEFAULT;
        if (i2c_new_master_bus(&i2c_config, &i2c_handle) != ESP_OK)
        {
            ESP_LOGD(TAG, "I2C initialization failed");
            throw std::runtime_error("I2C initialization failed");
        }

        esp_lcd_touch_config_t tp_cfg = {};
        tp_cfg.x_max = LCD_H_RES;
        tp_cfg.y_max = LCD_V_RES;
        tp_cfg.rst_gpio_num = GPIO_NUM_NC;
        tp_cfg.int_gpio_num = IIC_INT_PIN;
        tp_cfg.levels = {
            .reset = 0,
            .interrupt = 0,
        };
        tp_cfg.flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        };

        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config{};

        tp_io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_FT6x36_ADDRESS;
        tp_io_config.control_phase_bytes = 1;
        tp_io_config.dc_bit_offset = 0;
        tp_io_config.lcd_cmd_bits = 8;
        tp_io_config.flags = {
            .dc_low_on_data = 0,
            .disable_control_phase = 1,
        };
        tp_io_config.scl_speed_hz = TOUCH_IIC_CLK_HZ;

        if (esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle) != ESP_OK)
        {
            ESP_LOGD(TAG, "New lcd io i2c panel initialization failed");
            throw std::runtime_error("New lcd io i2c panel initialization failed");
        }

        if (esp_lcd_touch_new_i2c_ft6x36(tp_io_handle, &tp_cfg, &touch_handle) != ESP_OK) {
            ESP_LOGD(TAG, "New i2c ft6x36 initialization failed");
            throw std::runtime_error("New i2c ft6x36 initialization failed");
        }
    }

    Touch::~Touch()
    {
    }

    esp_lcd_touch_handle_t Touch::getTouchHandle() const
    {
        return touch_handle;
    }
}