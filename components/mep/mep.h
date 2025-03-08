
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include "EncMot.h"


typedef void* mep_h;
mep_h mep_init(QueueHandle_t LogQueue);
void mep_setPosition_byNormal (mep_h instance, const double azimuth, const double polar);


#ifdef __cplusplus
}
#endif
