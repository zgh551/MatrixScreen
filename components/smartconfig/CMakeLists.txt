idf_component_register(SRCS "smartconfig.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash wpa_supplicant
                    )

