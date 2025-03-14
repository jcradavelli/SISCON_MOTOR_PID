
#include <string.h>
#include "bdc_motor.h"
#include "motor.h"

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks


static const char *TAG = "motor";


typedef struct motor_hand_DC_GA25_{
    int motor_control_a;
    int motor_control_b;
    bdc_motor_handle_t bdc_motor_handle;
}motor_hand_t;


static motor_hand_t __motor_dc_ga25_attach (    
    int motor_control_a,
    int motor_control_b
)
{
    motor_hand_t retval = {0};

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

motor_hand_h motor_dc_ga25_attach (motor_config_t config)
{
    motor_hand_t aux;
    motor_hand_h object = NULL;
    
    // Aloca memoria para o objeto EcnMot
    object = malloc(sizeof(motor_hand_t));
    if (object == NULL)
    {
        ESP_LOGE(TAG,"Não foi possivel criar o objeto.");
        return(NULL);
    }

    // Configura os periféricos e o objeto EncMot
    aux = __motor_dc_ga25_attach (
        config.DC_GA25.motor_control_a,
        config.DC_GA25.motor_control_b
    );
    memcpy(object, &aux, sizeof(aux));

    return(object);
}

void motor_dc_ga25_run_cw (motor_hand_h handler)
{
    motor_hand_t *object = handler;

    assert(handler!=NULL);

    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_b,0));
    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_a,1));

}

void motor_dc_ga25_run_ccw (motor_hand_h handler)
{
    motor_hand_t *object = handler;

    assert(handler!=NULL);
    
    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_a,0));
    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_b,1));
}

void motor_dc_ga25_release (motor_hand_h handler)
{
    motor_hand_t *object = handler;

    assert(handler!=NULL);
    
    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_a,1));
    ESP_ERROR_CHECK(gpio_set_level(object->motor_control_b,1));
}

void    motor_dc_ga25_set_speed (motor_hand_h handler, double speed)
{
    motor_hand_t *object = handler;
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






motor_drv_t motor_dc_ga25_driver = {
    .motor_attach       =   motor_dc_ga25_attach   ,
    .motor_release      =   motor_dc_ga25_release  ,
    .motor_run_ccw      =   motor_dc_ga25_run_ccw  ,
    .motor_run_cw       =   motor_dc_ga25_run_cw   ,
    .motor_set_speed    =   motor_dc_ga25_set_speed,
};