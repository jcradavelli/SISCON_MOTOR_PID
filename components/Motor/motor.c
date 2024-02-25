
#include <string.h>
#include "bdc_motor.h"
#include "motor.h"

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks


static const char *TAG = "motor";


typedef struct motor_{
    int motor_control_a;
    int motor_control_b;
    bdc_motor_handle_t bdc_motor_handle;
}motor_t;


static motor_t __motor_attach (    
    int motor_control_a,
    int motor_control_b
)
{
    motor_t retval = {0};

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = motor_control_a,
        .pwmb_gpio_num = motor_control_b,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));

    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_ERROR_CHECK(bdc_motor_forward(motor));
    
    retval.motor_control_a = motor_control_a;
    retval.motor_control_b = motor_control_b;
    retval.bdc_motor_handle = motor;

    return(retval);
}

motor_h motor_attach (motor_config_t config)
{
    motor_t aux;
    motor_h object = NULL;
    
    // Aloca memoria para o objeto EcnMot
    object = malloc(sizeof(motor_t));
    if (object == NULL)
    {
        ESP_LOGE(TAG,"Não foi possivel criar o objeto.");
        return(NULL);
    }

    // Configura os periféricos e o objeto EncMot
    aux = __motor_attach (
        config.motor_control_a,
        config.motor_control_b
    );
    memcpy(object, &aux, sizeof(aux));

    return(object);
}

void motor_run_cw (motor_h handler)
{
    motor_t *object = handler;

    assert(handler!=NULL);

    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_a,1));
    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_b,0));

}

void motor_run_ccw (motor_h handler)
{
    motor_t *object = handler;

    assert(handler!=NULL);
    
    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_a,0));
    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_b,1));
}

void motor_release (motor_h handler)
{
    motor_t *object = handler;

    assert(handler!=NULL);
    
    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_a,1));
    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_b,1));
}

void    motor_set_speed (motor_h handler, float speed)
{
    motor_t *object = handler;
    assert(handler!=NULL);

    if (speed < 0)
    {
        speed = -1 * speed;
        ESP_ERROR_CHECK(bdc_motor_reverse(object->bdc_motor_handle));
    }
    else
    {
        ESP_ERROR_CHECK(bdc_motor_forward(object->bdc_motor_handle));
    }

    bdc_motor_set_speed(object->bdc_motor_handle, (uint32_t) speed);

}