

#ifndef __TASK_GRAPH_H__
#define __TASK_GRAPH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "PIDcontroller.h"
#include "tasks.h"


typedef struct tskGraph_args_
{
    QueueHandle_t logQueue;
    EventGroupHandle_t xEventGroup;
}tskGraph_args_t;

void create_tsk_graph (tskGraph_args_t* tskInputArgs, UBaseType_t prioridade ,const BaseType_t xCoreID);


void update_increment       (QueueHandle_t queue, double value);
void update_pidDebugStream  (QueueHandle_t queue, PIDControllerDebugStream_t value);
void update_KP              (QueueHandle_t queue, double value);
void update_KI              (QueueHandle_t queue, double value);
void update_KD              (QueueHandle_t queue, double value);
void update_SP              (QueueHandle_t queue, double value);
void update_SEL             (QueueHandle_t queue, const char *value);
void update_newValue        (QueueHandle_t queue, double value);


#ifdef __cplusplus
}
#endif

#endif // __TASK_GRAPH_H__
