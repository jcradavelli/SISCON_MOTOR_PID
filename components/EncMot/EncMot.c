
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "hal/pcnt_ll.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include "motor.h"
#include "Encoder.h"
#include "PIDcontroller.h"
#include "EncMot.h"
#include <string.h>
#include <math.h>

static const char *TAG = "EncMot";


#define EXAMPLE_PCNT_HIGH_LIMIT PCNT_LL_MAX_LIM
#define EXAMPLE_PCNT_LOW_LIMIT  PCNT_LL_MIN_LIN
#define WATCHPOINT_SUPERIOR     5000
#define WATCHPOINT_TARGETT      0
#define WATCHPOINT_INFERIOR     -5000



typedef struct encmot_{
    /* Sub Objetos */
    encoder_h        encoder;
    motor_h          motor;
    pid_controller_h PIDcontroller;

    /* Contexto do objeto principal */
    float            pid_out;
    pid_sample_t     pid_input;
    float            pid_setpoint;
    bool*            isSatured;
}encmot_t;

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}



encmot_h encmot_attach (encmot_config_t config)
{
    encmot_t aux;
    encmot_h object = NULL;
    
    // Aloca memoria para o objeto EcnMot
    object = malloc(sizeof(encmot_t));
    if (object == NULL)
    {
        ESP_LOGE(TAG,"Não foi possivel criar o objeto.");
        return(NULL);
    }

    // Conecta o encoder
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    encoder_config_t encoder_config = {

        .gpio_encoder_a                     =   config.encoder_config.gpio_encoder_a, 
        .gpio_encoder_b                     =   config.encoder_config.gpio_encoder_b, 
        .max_glitch_ns                      =   config.encoder_config.max_glitch_ns, 
        .pcnt_high_limit                    =   EXAMPLE_PCNT_HIGH_LIMIT,
        .pcnt_low_limit                     =   EXAMPLE_PCNT_LOW_LIMIT,
        .watchpoint_inferior                =   WATCHPOINT_INFERIOR,
        .watchpoint_targett                 =   WATCHPOINT_TARGETT, 
        .watchpoint_superior                =   WATCHPOINT_SUPERIOR,
        .example_pcnt_on_reach              =   example_pcnt_on_reach,
        .example_pcnt_on_reach_user_data    =   queue,
        .gear_ratio_numerator               =   config.encoder_config.gear_ratio_numerator,
        .gear_ration_denominator            =   config.encoder_config.gear_ration_denominator,
    };
    aux.encoder = encoder_attach (encoder_config);
    
    // conecta os drivers de motores
    motor_config_t motor_config = {
        .motor_control_a                    =   config.motor_config.motor_control_a,
        .motor_control_b                    =   config.motor_config.motor_control_b
    };
    aux.motor   = motor_attach  (motor_config);
    motor_set_speed(aux.motor, 0);

    // Conecta o controlador PID
    pid_config_t pid_config = {
        .kd                                 =   config.pid_config.kd,
        .ki                                 =   config.pid_config.ki,
        .kp                                 =   config.pid_config.kp,
        .out                                =   &((encmot_t*)object)->pid_out,
        .sample                             =   &((encmot_t*)object)->pid_input,
        .samplerate_ms                      =   config.pid_config.samplerate_ms,
        .set                                =   &((encmot_t*)object)->pid_setpoint,
    };
    aux.PIDcontroller = pid_attach(pid_config);
    pid_limits(aux.PIDcontroller, -400, 400); //TODO: Definir os limites conforme a resolução do PWM do motor {Resolução HZ / Freq do PWM}
    pid_auto(aux.PIDcontroller);
    aux.pid_setpoint = 0; 
    aux.pid_input.input = 0;
    aux.pid_input.time_ms = 0;
    aux.isSatured = config.pid_config.isSatured;
    
    memcpy(object, &aux, sizeof(aux));

    /* Configura o nivel de LOG da biblioteca encoder */
    esp_log_level_set("encoder", ESP_LOG_INFO);

    return(object);
}

//Wrapper function
inline int encmot_get_enconderCount_raw(encmot_h handler)
{
    encmot_t *object = handler;
    return (encoder_get_enconderCount_raw(object->encoder));
}

//Wrapper function
inline float encmot_get_encoderPosition_grad(encmot_h handler)
{
    encmot_t *object = handler;
    return(encoder_get_encoderPosition_grad(object->encoder));
}

//Wrapper function
inline float encmot_get_encoderPosition_rad(encmot_h handler)
{
    encmot_t *object = handler;
    return(encoder_get_encoderPosition_rad(object->encoder));
}


#define GET_OBJECT(__handler__) (assert(__handler__!=NULL);encmot_t *object = handler;)

void encmot_stop (encmot_h handler)
{
    assert(handler!=NULL);
    encmot_t *object = handler;

    // Coloca o PID em modo manual
    pid_manual(object->PIDcontroller);

    // Colocar a velocidade do motor em 0
    motor_set_speed(object->motor, 0);

}

void encmot_continue (encmot_h handler)
{
    assert(handler!=NULL);
    encmot_t *object = handler;

    pid_auto(object->PIDcontroller);
}

void encmot_runn (encmot_h handler) //sugestão de nome: encmot_tick
{
    encmot_t *object = handler;
    encoder_sample_t encoder_sample;

    assert(handler!=NULL);

    /* Roda a subrotina do encoder */
    encoder_runn(object->encoder);

    /* Coloca os dados de saída do encoder, na entrada do PID  */
    encoder_sample = encoder_get_lastSample(object->encoder);
    object->pid_input.input = encoder_sample.count;
    object->pid_input.time_ms = encoder_sample.time/1000;

    /* Roda a subrotina de controle, se necessário atualiza a saida */
    if (pid_compute(object->PIDcontroller)){

        /* Propaga a saída do controle para o atuador*/
        motor_set_speed(object->motor, object->pid_out);

        /* check PID saturation */
        *object->isSatured = pid_IsSatured(object->PIDcontroller);
    }
}

void encmot_set_position (encmot_h handler, int setpoint)
{
    encmot_t *object = handler;
    assert(handler!=NULL);

    ESP_LOGD(TAG,"changing setpoint to %f", (float)setpoint);
    object->pid_setpoint = (float)setpoint;
}

void encmot_tune_pid (encmot_h handler, float kp, float ki, float kd)
{
    encmot_t *object = handler;
    assert(handler!=NULL);
    pid_tune(object->PIDcontroller, kp, ki, kd);


}
