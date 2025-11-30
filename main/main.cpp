#include <chrono>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

#include "BoardInfo.h"
#include "Camera.hpp"
#include "LCD.hpp"
#include "LVGL.hpp"
#include "Touch.hpp"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace std::chrono_literals;
using namespace ESP32S3_FREENOVE_DEV_KIT;

static const char* TAG = "LVGL_PORT_TEST";

struct CameraTaskData {
    Camera* camera = nullptr;
    lv_obj_t* img_video_display_widget = nullptr;
    lv_img_dsc_t img_video_frame;
};

CameraTaskData cameraTaskData{};

TaskHandle_t cameraTaskHandle;

lv_obj_t* btnLabel = nullptr;
enum class Swizzles {
    None = 0,
    RGB565,
    RBG565,
    BGR565,
    ENDIAN,
    ENDIAN_RGB565,
    ENDIAN_RBG565,
    ENDIAN_BGR565,
};

std::map<Swizzles, std::string> SwizzlesStr = {
    {Swizzles::None, "None"},
    {Swizzles::RGB565, "RGB565"},
    {Swizzles::RBG565, "RBG565"},
    {Swizzles::BGR565, "BGR565"},
    {Swizzles::ENDIAN, "ENDIAN"},
    {Swizzles::ENDIAN_RGB565, "ENDIAN_RGB565"},
    {Swizzles::ENDIAN_RBG565, "ENDIAN_RBG565"},
    {Swizzles::ENDIAN_BGR565, "ENDIAN_BGR565"},
};

Swizzles currentSwizzle = Swizzles::None;

static void _app_button_cb(lv_event_t* e) {
    // lv_display_t *lvgl_display = static_cast<lv_display_t *>(e->user_data);
    // if (lvgl_display == nullptr)
    // {
    //     ESP_LOGD(TAG, "Invalid LVGL pointer");
    //     return;
    // }

    // lv_disp_rot_t rotation = lv_disp_get_rotation(lvgl_display);
    // switch (rotation)
    // {
    // case LV_DISP_ROT_NONE:
    //     rotation = LV_DISP_ROT_90;
    //     break;
    // case LV_DISP_ROT_90:
    //     rotation = LV_DISP_ROT_180;
    //     break;
    // case LV_DISP_ROT_180:
    //     rotation = LV_DISP_ROT_270;
    //     break;
    // case LV_DISP_ROT_270:
    //     rotation = LV_DISP_ROT_NONE;
    //     break;
    // }

    // /* LCD HW rotation */
    // lv_disp_set_rotation(lvgl_display, rotation);
    switch (currentSwizzle) {
        case Swizzles::None:
            currentSwizzle = Swizzles::RGB565;
            break;
        case Swizzles::RGB565:
            currentSwizzle = Swizzles::RBG565;
            break;
        case Swizzles::RBG565:
            currentSwizzle = Swizzles::BGR565;
            break;
        case Swizzles::BGR565:
            currentSwizzle = Swizzles::ENDIAN;
            break;
        case Swizzles::ENDIAN:
            currentSwizzle = Swizzles::ENDIAN_RGB565;
            break;
        case Swizzles::ENDIAN_RGB565:
            currentSwizzle = Swizzles::ENDIAN_RBG565;
            break;
        case Swizzles::ENDIAN_RBG565:
            currentSwizzle = Swizzles::ENDIAN_BGR565;
            break;
        case Swizzles::ENDIAN_BGR565:
            currentSwizzle = Swizzles::None;
            break;
    }

    lv_label_set_text_static(btnLabel, SwizzlesStr[currentSwizzle].c_str());
}

void init_img_video_frame(CameraTaskData* pCameraTaskData) {
    lv_img_header_t header;
    header.always_zero = 0;
    header.w = 240;
    header.h = 240;
    header.cf = LV_IMG_CF_TRUE_COLOR;
    pCameraTaskData->img_video_frame.header = header;
    pCameraTaskData->img_video_frame.data_size = 240 * 240 * 2;
    pCameraTaskData->img_video_frame.data = NULL;
}

void loopTask_camera(void* pvParameters) {
    CameraTaskData* pCameraTaskData = &cameraTaskData;
    Camera* pCamera = pCameraTaskData->camera;

    init_img_video_frame(pCameraTaskData);

    while (true) {
        pCamera->capture([&pCameraTaskData](camera_fb_t* fb) {
            uint16_t* pixels = (uint16_t*)fb->buf;
            size_t pixel_count = fb->len / 2;

            switch (currentSwizzle) {
                case Swizzles::None:
                    break;
                case Swizzles::RGB565:
                    for (size_t i = 0; i < pixel_count; i++) {
                        uint16_t pixel = pixels[i];

                        uint16_t red = (pixel >> 11) & 0x1F;
                        uint16_t green = (pixel >> 5) & 0x3F;
                        uint16_t blue = pixel & 0x1F;

                        pixels[i] = (red << 11) | (green << 5) | blue;
                    }
                    break;
                case Swizzles::RBG565:
                    for (size_t i = 0; i < pixel_count; i++) {
                        uint16_t pixel = pixels[i];

                        uint16_t red = (pixel >> 11) & 0x1F;
                        uint16_t blue = (pixel >> 5) & 0x3F;
                        uint16_t green = pixel & 0x1F;

                        pixels[i] = (red << 11) | (green << 5) | blue;
                    }
                    break;
                case Swizzles::BGR565:
                    for (size_t i = 0; i < pixel_count; i++) {
                        uint16_t pixel = pixels[i];

                        uint16_t blue = (pixel >> 11) & 0x1F;
                        uint16_t green = (pixel >> 5) & 0x3F;
                        uint16_t red = pixel & 0x1F;

                        pixels[i] = (red << 11) | (green << 5) | blue;
                    }
                    break;
                case Swizzles::ENDIAN:
                    for (size_t i = 0; i < pixel_count; i++) {
                        pixels[i] = ((0xff00 & pixels[i]) >> 8) | ((0xff & pixels[i]) << 8);
                    }
                    break;
                case Swizzles::ENDIAN_RGB565:
                    for (size_t i = 0; i < pixel_count; i++) {
                        uint16_t pixel = ((0xff00 & pixels[i]) >> 8) | ((0xff & pixels[i]) << 8);

                        uint16_t red = (pixel >> 11) & 0x1F;
                        uint16_t green = (pixel >> 5) & 0x3F;
                        uint16_t blue = pixel & 0x1F;

                        pixels[i] = (red << 11) | (green << 5) | blue;
                    }
                    break;
                case Swizzles::ENDIAN_RBG565:
                    for (size_t i = 0; i < pixel_count; i++) {
                        uint16_t pixel = ((0xff00 & pixels[i]) >> 8) | ((0xff & pixels[i]) << 8);

                        uint16_t red = (pixel >> 11) & 0x1F;
                        uint16_t blue = (pixel >> 5) & 0x3F;
                        uint16_t green = pixel & 0x1F;

                        pixels[i] = (red << 11) | (green << 5) | blue;
                    }
                    break;
                case Swizzles::ENDIAN_BGR565:
                    for (size_t i = 0; i < pixel_count; i++) {
                        uint16_t pixel = ((0xff00 & pixels[i]) >> 8) | ((0xff & pixels[i]) << 8);

                        uint16_t blue = (pixel >> 11) & 0x1F;
                        uint16_t green = (pixel >> 5) & 0x3F;
                        uint16_t red = pixel & 0x1F;

                        pixels[i] = (red << 11) | (green << 5) | blue;
                    }
                    break;
            }

            pCameraTaskData->img_video_frame.data = fb->buf;
            lv_img_set_src(pCameraTaskData->img_video_display_widget, &(pCameraTaskData->img_video_frame));
        });
    }
}

static void app_main_display(const LVGL& lvgl) {
    lv_obj_t* scr = lv_scr_act();

    /* Task lock */
    lvgl_port_lock(0);

    /* Create image */
    cameraTaskData.img_video_display_widget = lv_img_create(scr);
    lv_obj_set_pos(cameraTaskData.img_video_display_widget, 0, 0);
    lv_obj_set_size(cameraTaskData.img_video_display_widget, 240, 240);

    /* Label */
    lv_obj_t* label = lv_label_create(scr);
    lv_obj_set_width(label, LCD_H_RES);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_label_set_recolor(label, true);
    lv_label_set_text(label, "#FF0000 " LV_SYMBOL_BELL " Hello world" LV_SYMBOL_BELL
                             "\n"
                             "#00FF00 " LV_SYMBOL_WARNING "Having Fun!" LV_SYMBOL_WARNING
                             "\n"
                             "#0000FF Blue text #");

    lv_obj_align(label, LV_ALIGN_CENTER, 0, 20);

    /* Button */
    lv_obj_t* btn = lv_btn_create(scr);
    btnLabel = lv_label_create(btn);
    lv_label_set_text_static(btnLabel, SwizzlesStr[currentSwizzle].c_str());
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_add_event_cb(btn, _app_button_cb, LV_EVENT_CLICKED, (void*)lvgl.getDisplay());

    /* Task unlock */
    lvgl_port_unlock();
}

extern "C" void app_main(void) {
    try {
        auto lvgl = std::make_unique<LVGL>(std::make_unique<LCD>(), std::make_unique<Touch>());

        Camera camera;

        cameraTaskData.camera = &camera;

        /* Show LVGL objects */
        app_main_display(*lvgl);

        init_img_video_frame(&cameraTaskData);
        xTaskCreate(loopTask_camera, "Camera loop task", 8192, nullptr, 1, &cameraTaskHandle);
        while (true) {
            std::this_thread::sleep_for(20ms);
        }
    } catch (const std::exception& ex) {
        ESP_LOGD(TAG, "%s", ex.what());
    }
}
