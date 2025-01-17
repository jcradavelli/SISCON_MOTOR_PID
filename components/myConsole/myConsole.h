/**
 * @file Console.h
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

#include "freertos/FreeRTOS.h"
#include "esp_console.h"


typedef struct tskConsole_args_
{
    const char* gretings;
    const char* version;
}tskConsole_args_t;


void create_tsk_console (const tskConsole_args_t* tskConsolArgs, UBaseType_t prioridade ,const BaseType_t xCoreID);

#ifdef __cplusplus
}
#endif
