
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include "EncMot.h"


typedef void* mep_h;
mep_h mep_init(QueueHandle_t LogQueue);

/**
 * @brief Posiciona a plataforma de acordo com o azimute e o polar da normal à posição desejada
 * 
 * @param instance instancia do MEP
 * @param azimuth angulo da projeção da normal sobre o eixo X [em graus]
 * @param polar angulo da normal em relação ao eixo Z [em graus]
 */
void mep_setPosition_byNormal (mep_h instance, const double azimuth, const double polar);


#ifdef __cplusplus
}
#endif
