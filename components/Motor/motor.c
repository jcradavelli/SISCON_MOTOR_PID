
#include <string.h>
#include "bdc_motor.h"
#include "motor.h"



static const char *TAG = "motor";



motor_h motor_attach (motor_config_t config)
{
    motor_h pObject = malloc(sizeof(motor_t));
    
    assert (pObject != NULL);
    motor_t* object = pObject;
    
    
    switch (config.model)
    {
        case MOTOR_MODEL_DC_GA25:
            object->drv = &motor_dc_ga25_driver;
            break;

        case MOTOR_MODEL_SERVO_24H55M020:
            object->drv = &motor_servo_24h55m020_driver;
            break;
    }

    object->handler = object->drv->motor_attach(config);

    return(pObject);
}

void motor_run_cw (motor_h handler)
{
    motor_t *object = handler;

    assert(handler!=NULL);

    object->drv->motor_run_cw(object->handler);

}

void motor_run_ccw (motor_h handler)
{
    motor_t *object = handler;

    assert(handler!=NULL);
    
    object->drv->motor_run_ccw(object->handler);

}

void motor_release (motor_h handler)
{
    motor_t *object = handler;

    assert(handler!=NULL);
    
    object->drv->motor_release(object->handler);
}

void    motor_set_speed (motor_h handler, double speed)
{
    motor_t *object = handler;
    assert(handler!=NULL);

    ESP_LOGI(TAG, "Set Speed: %0.2f", speed);

    object->drv->motor_set_speed(object->handler, speed);

}