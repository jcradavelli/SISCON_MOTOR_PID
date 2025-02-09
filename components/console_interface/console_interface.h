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

/*
 * Submodulos adicionais adicionados atraves do menuconfig
 * ***********************
 */
#ifdef CONFIG_CONSOLE_INTERFACE_MODULE_SYS_COMMON
#include "modules/sys_common/sys_common.h"
#endif

#ifdef CONFIG_CONSOLE_INTERFACE_MODULE_MEP_KINEMATICS_INTERFACE
#include "modules/mep_interface/mep_interface.h" 
#endif

#ifdef CONFIG_CONSOLE_INTERFACE_MODULE_ENCMOT_INTERFACE
#include "modules/encmot_interface/encmot_interface.h" 
#endif

typedef struct tskConsole_args_
{
    const char* gretings;
    const char* version;
}tskConsole_args_t;


void create_tsk_console (const tskConsole_args_t* tskConsolArgs, UBaseType_t prioridade ,const BaseType_t xCoreID);

#ifdef __cplusplus
}
#endif
