


idf_component_register(

    # Arquivos fonte compiláveis privados
    SRCS 
        "console_interface.c" 
        

    # Dependências    
    REQUIRES      
        console 
        driver 
        nvs_flash
        EncMot
        esp_driver_usb_serial_jtag

                  
    # diretórios de arquivos públicos    
    INCLUDE_DIRS 
        "."
)
