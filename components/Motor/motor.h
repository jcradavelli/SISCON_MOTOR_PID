
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

typedef struct motor_config_{
    int motor_control_a;
    int motor_control_b;
}motor_config_t;


motor_h motor_attach    (motor_config_t config);
void    motor_run_cw    (motor_h handler);
void    motor_run_ccw   (motor_h handler);
void    motor_release   (motor_h handler);

void    motor_set_speed (motor_h handler, float speed);

#ifdef __cplusplus
}
#endif
