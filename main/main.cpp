#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"

#include "esp_lcd_touch_ft6x36.h"
#include "esp_lvgl_port_touch.h"

#include "BoardInfo.h"

static const char *TAG = "LVGL_PORT_TEST";

// LVGL image declare
LV_IMG_DECLARE(esp_logo)

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

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
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    io_config.dc_gpio_num = TFT_DC_PIN;
    io_config.cs_gpio_num = TFT_CS_PIN;
    io_config.pclk_hz = LCD_PIXEL_CLK_HZ;
    io_config.lcd_cmd_bits = LCD_CMD_BITS;
    io_config.lcd_param_bits = LCD_PARAM_BITS;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    panel_config.reset_gpio_num = TFT_RST_PIN;
    panel_config.data_endian = LCD_RGB_DATA_ENDIAN_LITTLE;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = LCD_BITS_PER_PIXEL;
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    return ret;

err:
    if (lcd_panel)
    {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io)
    {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(LCD_SPI_NUM);
    return ret;
}

static esp_err_t app_touch_init(void)
{
    /* Initilize I2C */
    i2c_master_bus_handle_t i2c_handle = NULL;
    i2c_master_bus_config_t i2c_config = {};
    i2c_config.i2c_port = TOUCH_IIC_NUM;
    i2c_config.sda_io_num = IIC_SDA_PIN;
    i2c_config.scl_io_num = IIC_SCL_PIN;
    i2c_config.clk_source = I2C_CLK_SRC_DEFAULT;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_config, &i2c_handle), TAG, "");

    /* Initialize touch HW */
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
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");

    return esp_lcd_touch_new_i2c_ft6x36(tp_io_handle, &tp_cfg, &touch_handle);
}

static esp_err_t app_lvgl_init(void)
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
    disp_cfg.io_handle = lcd_io;
    disp_cfg.panel_handle = lcd_panel;
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
    touch_cfg.handle = touch_handle;
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
    ESP_ERROR_CHECK(app_lcd_init());

    /* Touch initialization */
    ESP_ERROR_CHECK(app_touch_init());

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    /* Show LVGL objects */
    app_main_display();
}
