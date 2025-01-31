
set(EXTRA_COMPONENT_DIRS modules) 

idf_component_register(

    # Arquivos fonte compiláveis privados
    SRCS 
        "console_interface.c" 
        

    # Dependências    
    REQUIRES      
        console 
        driver 
        nvs_flash

                  
    # diretórios de arquivos públicos    
    INCLUDE_DIRS 
        "."
)
