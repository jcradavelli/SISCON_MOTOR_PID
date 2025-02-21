idf_component_register(
    SRCS 
    "mep.c" 

                    
    INCLUDE_DIRS 
    "." 
         
    REQUIRES 
    Encoder 
    PIDcontroller 
    Motor 
    EncMot 
    console_interface 
    MEP_CINEMATICS

)
