
#ifndef __TASK_INPUT_H__
#define __TASK_INPUT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tasks.h"
#include "EncMot.h"

#define BUTTON_1                12// 21
#define BUTTON_2                13// 22
#define BUTTON_3                14// 23
#define BUTTON_4                9// 5

#define ENCODER_P               11//  18
#define ENCODER_N               10//  19

#define EXAMPLE_EC11_GPIO_A     34// 34
#define EXAMPLE_EC11_GPIO_B     34// 35


typedef struct tskInput_args_
{
    encmot_h encmot;
    EventGroupHandle_t xEventGroup;
    encoder_h encoder;
    QueueHandle_t logQueue;
}tskInput_args_t;

void create_tsk_input (tskInput_args_t* tskInputArgs, UBaseType_t prioridade ,const BaseType_t xCoreID);


#ifdef __cplusplus
}
#endif

#endif // __TASK_INPUT_H__