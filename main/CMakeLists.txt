idf_component_register(SRCS "xedoline.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES lineanalog BT_PID motor esp_driver_gpio esp_driver_ledc)
