
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

/**
 * @brief Libera os motores para girar livremente
 * 
 * @param instance instancia do MEP
 */
void mep_release_motors (mep_h instance);

/**
 * @brief Posiciona a plataforma na home position
 * 
 * @param instance instancia do MEP
 */
void mep_home_position (mep_h instance);

#ifdef __cplusplus
}
#endif
