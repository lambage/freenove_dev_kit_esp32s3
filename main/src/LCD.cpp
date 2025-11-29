#include "LCD.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "BoardInfo.h"

#include <stdexcept>

namespace ESP32S3_FREENOVE_DEV_KIT
{

    static const char *TAG = "LCD";

    LCD::LCD() {
        spi_bus_config_t buscfg = {};
        esp_lcd_panel_dev_config_t panel_config = {};
        esp_lcd_panel_io_spi_config_t io_config = {};

        /* LCD initialization */
        ESP_LOGD(TAG, "Initialize SPI bus");
        buscfg.mosi_io_num = TFT_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = TFT_SCK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t);
        if (spi_bus_initialize(LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK)
        {
            ESP_LOGD(TAG, "SPI bus initialization failed");
            throw std::runtime_error("SPI bus initialization failed");
        }

        ESP_LOGD(TAG, "Install panel IO");
        io_config.dc_gpio_num = TFT_DC_PIN;
        io_config.cs_gpio_num = TFT_CS_PIN;
        io_config.pclk_hz = LCD_PIXEL_CLK_HZ;
        io_config.lcd_cmd_bits = LCD_CMD_BITS;
        io_config.lcd_param_bits = LCD_PARAM_BITS;
        io_config.spi_mode = 0;
        io_config.trans_queue_depth = 10;
        if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_NUM, &io_config, &lcd_io) != ESP_OK) {
            ESP_LOGD(TAG, "New panel IO failed");
            throw std::runtime_error("New panel IO failed");
        }  

        ESP_LOGD(TAG, "Install LCD driver");
        panel_config.reset_gpio_num = TFT_RST_PIN;
        panel_config.data_endian = LCD_RGB_DATA_ENDIAN_LITTLE;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = LCD_BITS_PER_PIXEL;
        if (esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel) != ESP_OK) {
            ESP_LOGD(TAG, "New panel failed");
            throw std::runtime_error("New panel failed");
        };

        esp_lcd_panel_reset(lcd_panel);
        esp_lcd_panel_init(lcd_panel);
        esp_lcd_panel_mirror(lcd_panel, true, true);
        esp_lcd_panel_disp_on_off(lcd_panel, true);

    }
    LCD::~LCD()
    {
        if (lcd_panel)
        {
            esp_lcd_panel_del(lcd_panel);
        }
        if (lcd_io)
        {
            esp_lcd_panel_io_del(lcd_io);
        }
        spi_bus_free(LCD_SPI_NUM);
    }

    esp_lcd_panel_io_handle_t LCD::getPanelIOHandle() const
    {
        return lcd_io;
    }

    esp_lcd_panel_handle_t LCD::getPanelHandle() const
    {
        return lcd_panel;
    }

} // namespace ESP32S3_FREENOVE_DEV_KIT