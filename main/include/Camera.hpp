#pragma once

#include "esp_err.h"
#include "esp_camera.h"

#include <functional>

namespace ESP32S3_FREENOVE_DEV_KIT
{

    class Camera
    {
    public:
        Camera();
        ~Camera();

        Camera(Camera &&) = default;
        Camera& operator=(Camera &&) = default;
        Camera(const Camera &) = delete;
        Camera& operator=(const Camera &) = delete;
        
        using FrameCallback = std::function<void(camera_fb_t *)>;
        
        void capture(FrameCallback frameCallback) const;

    private:
        camera_config_t m_cameraConfig{};
    };

} // namespace ESP32S3_FREENOVE_DEV_KIT