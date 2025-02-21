idf_component_register(SRCS "EncMot.c"
                       REQUIRES driver esp_timer Encoder Motor PIDcontroller console_interface
                       INCLUDE_DIRS "."
                      )