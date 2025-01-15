idf_component_register(SRCS "myConsole.c" "system_comands.c"
                       REQUIRES console driver nvs_flash esp_system
                       INCLUDE_DIRS "."
                      )