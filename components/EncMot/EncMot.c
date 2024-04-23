
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


// TODO: Passar essa configuração como parâmetro
#define PID_TAU                 0.02f
#define PID_LIM_MIN             -400.0f
#define PID_LIM_MAX             400.0f
#define PID_LIM_MIN_INT         -300.0f
#define PID_LIM_MAX_INT         300.0f


typedef enum controlerMode_
{
    CONTMODE_OPEN_LOOP,
    CONTMODE_PID_SPEED,
}controlerMode_t;

typedef struct encmot_{
    /* Sub Objetos */
    encoder_h        encoder;
    motor_h          motor;
    PIDController_h  PIDcontroller;

    /* Contexto do objeto principal */
    double           pid_out;
    double           pid_measure;
    double           setpoint;
    bool*            isSatured;
    controlerMode_t  contmode;
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
    PIDController_h pid = PIDController_create();
    PIDController_Init(pid,config.pid_config.kp,config.pid_config.ki,config.pid_config.kd,PID_TAU,PID_LIM_MIN, PID_LIM_MAX, PID_LIM_MIN_INT, PID_LIM_MAX_INT, config.pid_config.samplerate_ms/1000.0);

    aux.PIDcontroller   = pid;
    aux.setpoint    = 0; 
    aux.pid_measure     = 0;
    aux.isSatured       = config.pid_config.isSatured;
    aux.contmode        = CONTMODE_OPEN_LOOP;

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
// inline float encmot_get_encoderPosition_grad(encmot_h handler)
// {
//     encmot_t *object = handler;
    // return(encoder_get_encoderPosition_grad(object->encoder));
// }

//Wrapper function
// inline float encmot_get_encoderPosition_rad(encmot_h handler)
// {
//     encmot_t *object = handler;
//     return(encoder_get_encoderPosition_rad(object->encoder));
// }


// void encmot_stop (encmot_h handler)
// {
//     assert(handler!=NULL);
//     encmot_t *object = handler;

//     // Coloca o PID em modo manual
//     //TODO: disable PID

//     // Colocar a velocidade do motor em 0
//     motor_set_speed(object->motor, 0);

// }

// void encmot_continue (encmot_h handler)
// {
//     assert(handler!=NULL);
//     encmot_t *object = handler;

//     // TODO: Enable PID
// }


static double __contmode_openloop (encmot_h handler)
{
    encmot_t *object = handler;

    /* Atualiza a saída */
    motor_set_speed(object->motor, object->setpoint);

    return(object->setpoint);
}

static double __contmode_pid_speed (encmot_h handler, encoder_sample_t encoder_sample, PIDControllerDebugStream_t *debugOut)
{
    encmot_t *object = handler;
    double pid_out;

    /* Coloca os dados de saída do encoder, na entrada do PID  */
    object->pid_measure = encoder_sample.speed; //TODO, converter para rad ou rad/s

    /* Roda a subrotina de controle */
    pid_out = PIDController_Update(object->PIDcontroller, object->setpoint, object->pid_measure, debugOut);


    return(pid_out);
}

//TODO: run inside a interrupt routine to mantain time 
void encmot_job (encmot_h handler, encmotDebugStream_t *stream_out) //sugestão de nome: encmot_tick
{
    encmot_t *object = handler;
    encoder_sample_t encoder_sample;
    double controller_output;
    PIDControllerDebugStream_t *PIDStream = NULL;

    assert(handler!=NULL);
    if (stream_out != NULL)
    {
        PIDStream = &stream_out->PID;
    }


    /* Roda a subrotina do encoder */
    encoder_job(object->encoder);
    encoder_sample = encoder_get_lastSample(object->encoder);

    switch(object->contmode)
    {
        case CONTMODE_OPEN_LOOP:
            controller_output = __contmode_openloop(handler);
            break;

        case CONTMODE_PID_SPEED:
            controller_output = __contmode_pid_speed(handler, encoder_sample, PIDStream);
            motor_set_speed(object->motor, controller_output);
            break;
    }

    /*stream controll data */
    // TODO: implementar fila e estrimar os dados de controle
}

void encmot_set_speed (encmot_h handler, double setpoint)
{
    encmot_t *object = handler;
    assert(handler!=NULL);

    //TODO: Check if contmode is running

    if (object->contmode != CONTMODE_PID_SPEED)
    {
        object->contmode = CONTMODE_PID_SPEED;
        //PIDController_reset (object->PIDcontroller);
    }

    ESP_LOGD(TAG,"changing setpoint to %e", setpoint);
    object->setpoint = setpoint;
}

void encmot_tune_pid (encmot_h handler, float kp, float ki, float kd)
{
    encmot_t *object = handler;
    assert(handler!=NULL);

    //TODO: Check if contmode is running
    
    PIDController_tune_pid(object->PIDcontroller, kp, ki, kd);
}
