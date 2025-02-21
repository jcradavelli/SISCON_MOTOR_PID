
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include "EncMot.h"


typedef void* mep_h;
mep_h mep_init(QueueHandle_t LogQueue);
void mep_setPosition_byNormal (mep_h this, const double normal[3]);


#ifdef __cplusplus
}
#endif
