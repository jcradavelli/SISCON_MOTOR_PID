
#ifndef __TASK_ENCMOT_H__
#define __TASK_ENCMOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tasks.h"
#include "EncMot.h"


typedef struct tskEncmot_args_
{
    encmot_h encmot;
    QueueHandle_t logQueue;
    EventGroupHandle_t xEventGroup;
}tskEncmot_args_t;


void create_tsk_encmot (tskEncmot_args_t* tskInputArgs, UBaseType_t prioridade ,const BaseType_t xCoreID);





#ifdef __cplusplus
}
#endif

#endif // __TASK_ENCMOT_H__