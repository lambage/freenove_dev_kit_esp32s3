set(requires        pedestrian_detect)

idf_component_register(SRCS "main.cpp" "src/LCD.cpp" "src/Touch.cpp" "src/LVGL.cpp" "src/Camera.cpp"
                    INCLUDE_DIRS "." "include")

