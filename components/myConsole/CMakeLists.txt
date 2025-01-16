idf_component_register(SRCS "console_encmot.c" "myConsole.c" "system_comands.c" "console_encmot.c"
                       REQUIRES console driver nvs_flash esp_system EncMot
                       INCLUDE_DIRS "."
                      )