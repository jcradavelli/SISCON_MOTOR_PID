/**
 * @file console_encmot.h
 * 
 * @author Júlio César Radavelli (julio.radavelli@tecnoflex-rs.com.br or jc.radavelli@hotmail.com)
 * 
 * @brief Fornece uma interface tipo console para comunicação com o usuário.
 * 
 * @version 0.1
 * @date 2024-01-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include "myConsole.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

#define MAX_ENC_MOT_INSTANCES 3
extern void* registered_encmot_instances[MAX_ENC_MOT_INSTANCES];





/**
 * @brief Função que inicializa todas as funções de console do objeto encmot
 * 
 * @param instance Numero da instância do objeto
 */
int register_encmot_newInstance (void* handler);


#ifdef __cplusplus
}
#endif