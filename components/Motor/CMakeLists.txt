idf_component_register(SRCS     
                                "motor.c" 
                                "./PORT/motor_DC_GA25.c"
                                "./PORT/motor_SERVO_24H55M020.c"
                       REQUIRES driver
                       INCLUDE_DIRS 
                                "."
                      )