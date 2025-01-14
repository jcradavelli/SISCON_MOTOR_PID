
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
#include "driver/ledc.h"
#include "driver/gpio.h"



typedef enum motor_models_
{
    MOTOR_MODEL_DC_GA25,
    MOTOR_MODEL_SERVO_24H55M020,
}motor_models_t;

/**
 * @brief Estrutura de configuração do motor a ser conectado
 * 
 */
typedef struct motor_config_{

    motor_models_t model;
    union{
        struct{
            const int motor_control_a;
            const int motor_control_b;

        }DC_GA25;
        struct
        {
            const int speed_gpio;
            const int start_stop_gpio;
            const int break_gpio;
            const int cw_ccw_gpio;
            const int status_gpio;
            const ledc_timer_t  timer_num;
            const double speed_offset;
            const double speed_gain;
            const double speed_max;
            const double speed_min;
        }SERVO_24H55M020;
    };

}motor_config_t;


typedef void* motor_h;

typedef struct motor_drv_
{
    motor_h (*motor_attach   ) (motor_config_t config);
    void    (*motor_run_cw   ) (motor_h handler);
    void    (*motor_run_ccw  ) (motor_h handler);
    void    (*motor_release  ) (motor_h handler);
    void    (*motor_set_speed) (motor_h handler, double speed);
}motor_drv_t;

extern motor_drv_t motor_servo_24h55m020_driver;
extern motor_drv_t motor_dc_ga25_driver;

typedef void* motor_hand_h;
typedef motor_drv_t* motor_drv_h;

typedef struct motor_
{
    motor_drv_h drv;        // ponteiro para lista de funções do driver
    motor_hand_h handler;   // ponteiro para o contexto do driver
}motor_t;






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
void    motor_set_speed (motor_h handler, double speed);

#ifdef __cplusplus
}
#endif
