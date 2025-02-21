/**
 * @file mep_instance.h
 * 
 * @author Júlio César Radavelli (julio.radavelli@tecnoflex-rs.com.br or jc.radavelli@hotmail.com)
 * 
 * @brief Fornece uma interface tipo console para comunicação com o usuário e acesso ao mep_interface.
 * 
 * @version 0.1
 * @date 2025-02-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include "console_interface.h"



typedef struct console_interface_mep_{
    void (*setNormal) (void* handler, const double normal[3]);

    void* handler; //<! Contexto do objeto mep
}console_interface_mep_t;


/**
 * @brief Função que inicializa todas as funções de console do objeto mep
 * 
 * @param instance Numero da instância do objeto
 */
int register_mep (console_interface_mep_t* interface_mep);


#ifdef __cplusplus
}
#endif