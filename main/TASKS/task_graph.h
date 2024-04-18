

#ifndef __TASK_GRAPH_H__
#define __TASK_GRAPH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tasks.h"


typedef struct tskGraph_args_
{
    QueueHandle_t logQueue;
    EventGroupHandle_t xEventGroup;
}tskGraph_args_t;

void create_tsk_graph (tskGraph_args_t* tskInputArgs, UBaseType_t prioridade ,const BaseType_t xCoreID);



#ifdef __cplusplus
}
#endif

#endif // __TASK_GRAPH_H__
