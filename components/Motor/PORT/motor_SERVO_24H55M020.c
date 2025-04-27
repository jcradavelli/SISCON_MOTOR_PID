
#include <string.h>
#include <math.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "motor.h"



#define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution up to 13 bits
#define LEDC_DUTY               (1<<(LEDC_DUTY_RES-1)) // Set duty to 50%
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 4 kHz

#define FREQ_MAX                (800) // 20KHz limite fisico
#define FREQ_MIN                (50)   // 300Hz

// #define speed_gpio          (2) // MCU OUTPUT speed singnal
// #define enable_gpio         (23) // MCU OUTPUT
// #define break_gpio          (22) // MCU OUTPUT
// #define cw_ccw_gpio         (21) // MCU OUTPUT
// #define status_gpio         (19) // MCU INPUT Low level if faill


static const char *TAG = "motor_24H55M020";


typedef struct motor_hand_SERVO_24H55M020_{
    // TODO: Definir os pios de cada funcao
    int speed_gpio     ;
    int enable_gpio    ;
    int break_gpio     ;
    int cw_ccw_gpio    ;
    int status_gpio    ;

    // TODO: Inserir as variáveis de contexto
    ledc_timer_t  timer_num;
    double speed_gain;
    double speed_offset;
    double speed_max;
    double speed_min;
}motor_hand_t;





static motor_hand_t __motor_servo_24h55m020_attach (    
    // TODO: Parametros são os pios de cada funcao
    const uint64_t speed_gpio     ,
    const uint64_t enable_gpio,
    const uint64_t break_gpio     ,
    const uint64_t cw_ccw_gpio    ,
    const uint64_t status_gpio    ,
    const ledc_timer_t  timer_num,
    const double speed_offset,
    const double speed_gain,
    const double speed_max,
    const double speed_min    
)
{
    

    ESP_LOGI(TAG, "Configure interface");

  // Prepare and then apply the LEDC PWM timer configuration
    const ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = timer_num,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_USE_RC_FAST_CLK,
        .deconfigure      = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    const ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = timer_num,
        .timer_sel      = timer_num,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = speed_gpio,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Configura os sinais de controle do motor
    const gpio_config_t gpio_config_mcu_inputs = {
        .pin_bit_mask = BIT64(status_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    // Configura os sinais de controle do motor
    const gpio_config_t gpio_config_mcu_outputs = {
        .pin_bit_mask = BIT64((uint64_t) enable_gpio) |
                        BIT64((uint64_t) break_gpio) |
                        BIT64((uint64_t) cw_ccw_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_config_mcu_inputs));
    ESP_ERROR_CHECK(gpio_config(&gpio_config_mcu_outputs));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, timer_num, LEDC_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, timer_num));


    ESP_LOGI(TAG, "Create driver");
    motor_hand_t drv_context = {0};

    drv_context.speed_gpio      = speed_gpio     ;
    drv_context.enable_gpio     = enable_gpio;
    drv_context.break_gpio      = break_gpio     ;
    drv_context.cw_ccw_gpio     = cw_ccw_gpio    ;
    drv_context.status_gpio     = status_gpio    ;
    drv_context.timer_num       = timer_num      ;
    drv_context.speed_offset    = speed_offset   ;
    drv_context.speed_gain      = speed_gain     ;
    drv_context.speed_max       = speed_max      ;
    drv_context.speed_min       = speed_min      ;

    return(drv_context);
}

motor_hand_h motor_servo_24h55m020_attach (motor_config_t config)
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
    aux = __motor_servo_24h55m020_attach (
        config.SERVO_24H55M020.speed_gpio       ,
        config.SERVO_24H55M020.start_stop_gpio      ,
        config.SERVO_24H55M020.break_gpio       ,
        config.SERVO_24H55M020.cw_ccw_gpio      ,
        config.SERVO_24H55M020.status_gpio      ,
        config.SERVO_24H55M020.timer_num        ,
        config.SERVO_24H55M020.speed_offset     ,
        config.SERVO_24H55M020.speed_gain       ,
        config.SERVO_24H55M020.speed_max        ,
        config.SERVO_24H55M020.speed_min        
    );
    memcpy(object, &aux, sizeof(aux));

    return(object);
}

void motor_servo_24h55m020_run_cw (motor_hand_h handler)
{
    motor_hand_t *object = handler;

    assert(handler!=NULL);

    ESP_ERROR_CHECK(gpio_set_level(object->cw_ccw_gpio,1));

}

void motor_servo_24h55m020_run_ccw (motor_hand_h handler)
{
    motor_hand_t *object = handler;

    assert(handler!=NULL);
    
    ESP_ERROR_CHECK(gpio_set_level(object->cw_ccw_gpio,0));
}

void motor_servo_24h55m020_release (motor_hand_h handler)
{
    motor_hand_t *object = handler;

    assert(handler!=NULL);
    
    if (xPortInIsrContext() == 0)
        printf("\r\n ERRO Funcao nao implementada\r\n");

}

/**
 * @brief 
 * 
 * Deve receber um valor entre -1 e 1
 * 
 * @param handler 
 * @param speed 
 */
void motor_servo_24h55m020_set_speed (motor_hand_h handler, double speed)
{
    motor_hand_t *object = handler;
    assert(handler!=NULL);

    uint32_t freq_hz = fabs(speed) * FREQ_MAX;

    ESP_ERROR_CHECK(gpio_set_level(object->enable_gpio,1));

    ESP_LOGI(TAG, "Set Speed: %0.2f | %lu Hz", speed, freq_hz);

    if (freq_hz > FREQ_MAX)
    {
        ESP_LOGW(TAG, "Velocidade configurada (%0.2f) excede à velocidade maxima (%0.4f)", speed, (double)FREQ_MAX);
        freq_hz = FREQ_MAX;

    }
    else if (freq_hz < FREQ_MIN)
    {
        ESP_LOGW(TAG, "Velocidade configurada (%0.2f) excede à velocidade minima (%0.4f)", speed, (double)FREQ_MIN);
        freq_hz = FREQ_MIN;
        //TODO: Parar o motor!
        
        ESP_ERROR_CHECK(gpio_set_level(object->enable_gpio,0));
        ESP_ERROR_CHECK(gpio_set_level(object->break_gpio,0));



        return;
    }


    if (speed < 0)
    {
        motor_servo_24h55m020_run_cw(handler);
        ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, object->timer_num, freq_hz));
        ledc_set_freq(LEDC_MODE, object->timer_num, freq_hz);
    }
    else
    {
        motor_servo_24h55m020_run_ccw(handler);
        ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, object->timer_num, freq_hz));
    }

    ESP_ERROR_CHECK(gpio_set_level(object->break_gpio,1));

}





motor_drv_t motor_servo_24h55m020_driver = {
    .motor_attach       =   motor_servo_24h55m020_attach   ,
    .motor_release      =   motor_servo_24h55m020_release  ,
    .motor_run_ccw      =   motor_servo_24h55m020_run_ccw  ,
    .motor_run_cw       =   motor_servo_24h55m020_run_cw   ,
    .motor_set_speed    =   motor_servo_24h55m020_set_speed,
};