idf_component_register(SRCS "main.cpp" "src/LCD.cpp"
                    INCLUDE_DIRS "." "include")

lvgl_port_create_c_image("images/esp_logo.png" "images/" "TRUECOLOR_ALPHA" "NONE")
lvgl_port_add_images(${COMPONENT_LIB} "images/")
