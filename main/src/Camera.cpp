#include "Camera.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include "BoardInfo.h"

#include <algorithm>
#include <stdexcept>

namespace ESP32S3_FREENOVE_DEV_KIT
{
    static const char *TAG = "LVGL";

    Camera::Camera()
    {
        m_cameraConfig.pin_pwdn = GPIO_NUM_NC;
        m_cameraConfig.pin_reset = GPIO_NUM_NC;
        m_cameraConfig.pin_xclk = CAM_XCLK_PIN;
        m_cameraConfig.pin_sccb_sda = CAM_SIOD_PIN;
        m_cameraConfig.pin_sccb_scl = CAM_SIOC_PIN;
        m_cameraConfig.pin_d7 = CAM_Y2_PIN;
        m_cameraConfig.pin_d6 = CAM_Y3_PIN;
        m_cameraConfig.pin_d5 = CAM_Y4_PIN;
        m_cameraConfig.pin_d4 = CAM_Y5_PIN;
        m_cameraConfig.pin_d3 = CAM_Y6_PIN;
        m_cameraConfig.pin_d2 = CAM_Y7_PIN;
        m_cameraConfig.pin_d1 = CAM_Y8_PIN;
        m_cameraConfig.pin_d0 = CAM_Y9_PIN;
        m_cameraConfig.pin_vsync = CAM_VSYNC_PIN;
        m_cameraConfig.pin_href = CAM_HERF_PIN;
        m_cameraConfig.pin_pclk = CAM_PCLK_PIN;
        m_cameraConfig.xclk_freq_hz = 16000000;
        m_cameraConfig.ledc_timer = LEDC_TIMER_0;
        m_cameraConfig.ledc_channel = LEDC_CHANNEL_0;
        m_cameraConfig.pixel_format = PIXFORMAT_RGB565; // YUV422,GRAYSCALE,RGB565,JPEG
        m_cameraConfig.frame_size = FRAMESIZE_240X240;  // QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.
        m_cameraConfig.jpeg_quality = 10;               // 0-63, for OV series camera sensors, lower number means higher quality
        m_cameraConfig.fb_count = 2;                    // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
        m_cameraConfig.grab_mode = CAMERA_GRAB_LATEST;  // CAMERA_GRAB_LATEST. Sets when buffers should be filled
        m_cameraConfig.fb_location = CAMERA_FB_IN_PSRAM;

        esp_camera_deinit();

        if (esp_camera_init(&m_cameraConfig) != ESP_OK)
        {
            ESP_LOGD(TAG, "Camera initialization failed");
            throw std::runtime_error("Camera initialization failed");
        }

        esp_camera_set_psram_mode(true);

        bool camera_flip_vertical_state = 0;
        bool camera_mirror_horizontal_state = 1;

        sensor_t *s = esp_camera_sensor_get();
        // initial sensors are flipped vertically and colors are a bit saturated
        s->set_vflip(s, camera_flip_vertical_state);       // flip it back
        s->set_hmirror(s, camera_mirror_horizontal_state); // horizontal mirror image
        s->set_brightness(s, 0);                           // up the brightness just a bit
        s->set_saturation(s, 0);                           // lower the saturation
    }
    Camera::~Camera() {}

    void Camera::capture(FrameCallback frameCallback) const
    {
        camera_fb_t *fb = esp_camera_fb_get();

        if (fb == nullptr)
        {
            return;
        }

        frameCallback(fb);

        esp_camera_fb_return(fb);
    }

}