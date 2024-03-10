
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "hal/pcnt_ll.h"
#include "driver/gpio.h"

typedef void* motor_h;

/**
 * @brief Estrutura de configuração do motor a ser conectado
 * 
 */
typedef struct motor_config_{
    int motor_control_a;
    int motor_control_b;
}motor_config_t;

/**
 * @brief conecta o motor à aplicação
 * 
 * @param config Matriz de configurações do motor
 * @return motor_h handler do objeto motor 
 */
motor_h motor_attach    (motor_config_t config);

/**
 * @brief gira motor no sentido horário na velocidade máxima
 * 
 * @param handler handler do objeto motor
 */
void    motor_run_cw    (motor_h handler);

/**
 * @brief gira o motor no sentido anti horário na velocidade máxima
 * 
 * @param handler 
 */
void    motor_run_ccw   (motor_h handler);

/**
 * @brief permite que o motor gire livremente
 * 
 * @param handler handler do objeto motor
 */
void    motor_release   (motor_h handler);

/**
 * @brief controla a velocidade do motor
 * velocidades positivas roda no sentido horário, negativas no sentio anti-horário
 * 
 * @param handler handler do objeto motor
 * @param speed velocidade desejada
 */
void    motor_set_speed (motor_h handler, float speed);

#ifdef __cplusplus
}
#endif
