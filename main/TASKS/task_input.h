
#ifndef __TASK_INPUT_H__
#define __TASK_INPUT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tasks.h"
#include "EncMot.h"

#define BUTTON_1                21
#define BUTTON_2                22
#define BUTTON_3                23
#define BUTTON_4                14

#define ENCODER_P               34
#define ENCODER_N               35


typedef struct tskInput_args_
{
    encmot_h encmot;
    EventGroupHandle_t xEventGroup;
    encoder_h encoder;
}tskInput_args_t;

void create_tsk_input (tskInput_args_t* tskInputArgs, UBaseType_t prioridade ,const BaseType_t xCoreID);


#ifdef __cplusplus
}
#endif

#endif // __TASK_INPUT_H__