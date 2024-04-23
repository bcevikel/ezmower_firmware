idf_component_register(SRCS "sonar_sensor.c" "main.c" "encoders.c" "l298n_driver.c" "sonar_sensor.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    REQUIRES esp_timer)