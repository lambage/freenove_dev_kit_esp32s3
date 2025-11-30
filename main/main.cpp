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
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pedestrian_detect.hpp"

using namespace std::chrono_literals;
using namespace ESP32S3_FREENOVE_DEV_KIT;

static const char* TAG = "LVGL_PORT_TEST";

struct CameraTaskData {
    Camera* camera = nullptr;
    lv_obj_t* img_video_display_widget = nullptr;
    lv_img_dsc_t img_video_frame;
};

#define NUM_BUFFERS 2
#define FRAME_SIZE 240 * 240 * 2
#define PROCESSING_TASK_PRIORITY 3

uint8_t* frame_buffers[NUM_BUFFERS];

QueueHandle_t xFreeBufferQueue;
QueueHandle_t xFullBufferQueue;

CameraTaskData cameraTaskData{};

lv_obj_t* btnLabel = nullptr;
enum class Swizzles {
    COLOR = 0,
    GRAYSCALE,
};

std::map<Swizzles, std::string> SwizzlesStr = {
    {Swizzles::COLOR, "Color"},
    {Swizzles::GRAYSCALE, "Grayscale"},
};

struct DetectionBox {
    int x;
    int y;
    int w;
    int h;
};

SemaphoreHandle_t lvgl_mutex;
std::vector<DetectionBox> detectionBoxes;

Swizzles currentSwizzle = Swizzles::COLOR;

void setup_buffers() {
    xFreeBufferQueue = xQueueCreate(NUM_BUFFERS, sizeof(uint8_t*));
    xFullBufferQueue = xQueueCreate(NUM_BUFFERS, sizeof(uint8_t*));

    // Populate the Free queue with all buffer pointers
    for (int i = 0; i < NUM_BUFFERS; i++) {
        frame_buffers[i] = (uint8_t*)heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_SPIRAM);
        uint8_t* ptr = frame_buffers[i];
        xQueueSend(xFreeBufferQueue, &ptr, portMAX_DELAY);
    }
}

static void _app_button_cb(lv_event_t* e) {
    switch (currentSwizzle) {
        case Swizzles::COLOR:
            currentSwizzle = Swizzles::GRAYSCALE;
            break;
        case Swizzles::GRAYSCALE:
            currentSwizzle = Swizzles::COLOR;
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

static uint16_t swap_endian_16(uint16_t value) {
    uint16_t high_byte = (value & 0xFF00) >> 8;
    uint16_t low_byte = (value & 0x00FF) << 8;
    return high_byte | low_byte;
}

static void drawOverlaysOnImage() {

    auto buf = (uint16_t*)cameraTaskData.img_video_frame.data;

    if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY) == pdTRUE) {
        const int thickness = 1;

        // pCameraTaskData->img_video_frame.data, detectionBoxes
        for (auto& box : detectionBoxes) {
            int x_min = box.x;
            int x_max = box.x + box.w;
            int y_min = box.y;
            int y_max = box.y + box.h;

            if (x_min > x_max || y_min > y_max) {
                return;
            }

            // Iterate through the thickness of the outline
            for (int t = 0; t < thickness; ++t) {
                // --- 2. Draw Top and Bottom Edges ---
                // Top edge: y = Y_min + t
                int y_top = y_min + t;
                if (y_top <= y_max) {
                    for (int x = x_min; x <= x_max; ++x) {
                        // Calculate index for the top row
                        buf[y_top * 240 + x] = 0xFFFF;
                    }
                }

                // Bottom edge: y = Y_max - t
                int y_bottom = y_max - t;
                if (y_bottom >= y_min) {
                    for (int x = x_min; x <= x_max; ++x) {
                        // Calculate index for the bottom row
                        buf[y_bottom * 240 + x] = 0xFFFF;
                    }
                }

                // --- 3. Draw Left and Right Edges ---
                // Iterate only between the drawn top and bottom lines to avoid overdrawing
                for (int y = y_min + t; y <= y_max - t; ++y) {
                    // Left edge: x = X_min + t
                    int x_left = x_min + t;
                    if (x_left <= x_max) {
                        // Calculate index for the left column
                        buf[y * 240 + x_left] = 0xFFFF;
                    }

                    // Right edge: x = X_max - t
                    int x_right = x_max - t;
                    if (x_right >= x_min) {
                        // Calculate index for the right column
                        buf[y * 240 + x_right] = 0xFFFF;
                    }
                }
            }
        }
        xSemaphoreGive(lvgl_mutex);
    }
}

void loopTask_camera(void* pvParameters) {
    CameraTaskData* pCameraTaskData = &cameraTaskData;
    Camera* pCamera = pCameraTaskData->camera;

    init_img_video_frame(pCameraTaskData);

    uint8_t* pCurrentWriteBuffer = NULL;

    while (true) {
        pCamera->capture([&pCameraTaskData, &pCurrentWriteBuffer](camera_fb_t* fb) {
            uint16_t* pixels = (uint16_t*)fb->buf;
            size_t pixel_count = fb->len / 2;

            switch (currentSwizzle) {
                case Swizzles::COLOR:
                    for (size_t i = 0; i < pixel_count; i++) {
                        pixels[i] = swap_endian_16(pixels[i]);
                    }
                    break;
                case Swizzles::GRAYSCALE:
                    for (size_t i = 0; i < pixel_count; i++) {
                        uint16_t pixel = swap_endian_16(pixels[i]);

                        uint16_t red = ((pixel >> 11) & 0x1F) << 3;
                        uint16_t green = ((pixel >> 5) & 0x3F) << 2;
                        uint16_t blue = (pixel & 0x1F) << 3;

                        uint8_t grayscale_value = static_cast<uint8_t>((red + green + blue) / 3);

                        uint8_t fivebit = grayscale_value >> 3;  // Take the top 5 bits of red
                        uint8_t sixbit = grayscale_value >> 2;   // Take the top 6 bits of green

                        pixels[i] = (fivebit << 11) | (sixbit << 5) | fivebit;
                    }
                    break;
            }
            if (xQueueReceive(xFreeBufferQueue, &pCurrentWriteBuffer, 0) == pdPASS) {
                auto buf = (uint16_t*)pCurrentWriteBuffer;
                for (size_t i = 0; i < pixel_count; i++) {
                    buf[i] = swap_endian_16(pixels[i]);
                }
                if (xQueueSend(xFullBufferQueue, &pCurrentWriteBuffer, (TickType_t)0) != pdPASS) {
                    xQueueSend(xFreeBufferQueue, &pCurrentWriteBuffer, portMAX_DELAY);
                }
            }

            pCameraTaskData->img_video_frame.data = fb->buf;

            drawOverlaysOnImage();

            lv_img_set_src(pCameraTaskData->img_video_display_widget, &(pCameraTaskData->img_video_frame));
        });
    }
}

void vImageProcessingTask(void* pvParameters) {
    uint8_t* pCurrentReadBuffer = NULL;

    PedestrianDetect* detect = new PedestrianDetect();

    while (true) {
        if (xQueueReceive(xFullBufferQueue, &pCurrentReadBuffer, portMAX_DELAY) == pdPASS) {
            dl::image::img_t img = {.data = pCurrentReadBuffer, .width = 240, .height = 240, .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565};
            std::list<dl::detect::result_t>& res = detect->run(img);

            if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY) == pdTRUE) {
                detectionBoxes.clear();
                if (res.size() > 0) {
                    for (auto& r : res) {
                        detectionBoxes.emplace_back(DetectionBox{r.box[0], r.box[1], r.box[2] - r.box[0], r.box[3] - r.box[1]});
                    }
                }
                xSemaphoreGive(lvgl_mutex);
            }

            xQueueSend(xFreeBufferQueue, &pCurrentReadBuffer, portMAX_DELAY);
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
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
        lvgl_mutex = xSemaphoreCreateMutex();

        auto lvgl = std::make_unique<LVGL>(std::make_unique<LCD>(), std::make_unique<Touch>());

        Camera camera;

        cameraTaskData.camera = &camera;

        setup_buffers();

        /* Show LVGL objects */
        app_main_display(*lvgl);

        init_img_video_frame(&cameraTaskData);
        xTaskCreate(vImageProcessingTask, "ProcessingTask", 4096, nullptr, PROCESSING_TASK_PRIORITY, nullptr);
        xTaskCreate(loopTask_camera, "Camera loop task", 2048, nullptr, 5, nullptr);

        while (true) {
            std::this_thread::sleep_for(20ms);
        }
    } catch (const std::exception& ex) {
        ESP_LOGD(TAG, "%s", ex.what());
    }
}
