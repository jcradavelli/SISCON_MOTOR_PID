

#ifndef __TASKS_H__
#define __TASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"





// typedef enum{
//     BUTTON_PRESSED_1,
//     BUTTON_RELEASE_1,
//     BUTTON_PRESSED_2,
//     BUTTON_RELEASE_2,
//     BUTTON_PRESSED_3,
//     BUTTON_RELEASE_3,
//     BUTTON_PRESSED_4,
//     BUTTON_RELEASE_4,
// }eventBits_t;

#ifndef BIT
#define BIT(__eventbit__) (1<<__eventbit__)
#endif // BIT

#ifdef __cplusplus
}
#endif

#endif // __TASKS_H__