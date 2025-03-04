
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

#ifdef CONFIG_CONSOLE_INTERFACE_MODULE_ENCMOT_INTERFACE
#include "console_interface.h"
#endif


static const char *TAG = "EncMot";
static const char *TAG_ACTION_SET_SPEED = "Seting_Speed";
static const char *TAG_ACTION_SET_POSITION = "Seting_Position";

#define EXAMPLE_PCNT_HIGH_LIMIT PCNT_LL_MAX_LIM
#define EXAMPLE_PCNT_LOW_LIMIT  PCNT_LL_MIN_LIN
#define WATCHPOINT_SUPERIOR     5000
#define WATCHPOINT_TARGETT      0
#define WATCHPOINT_INFERIOR     -5000


// TODO: Passar essa configuração como parâmetro
#define PID_TAU                 0.02f   // Coeficiente do filtro derivativo do PID
#define PID_LIM_MIN             -1.0f   // valor mínimo na saida do PID
#define PID_LIM_MAX             1.0f    // Valor máximo na saída do PID
#define PID_LIM_MIN_INT         -0.7f   // valor mínimo no termo integral do PID
#define PID_LIM_MAX_INT         0.7f    // Valor máximo no termo integral do PID


#define SPEED_CONSTANT          (1.0/314.15)



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
        // .gear_ratio_numerator               =   config.encoder_config.gear_ratio_numerator,
        // .gear_ration_denominator            =   config.encoder_config.gear_ration_denominator,
    };
    aux.encoder = encoder_attach (encoder_config);
    
    // conecta os drivers de motores
    aux.motor   = motor_attach  (config.motor_config);
    motor_set_speed(aux.motor, 0);

    // Conecta o controlador PID
    PIDController_h pid = PIDController_create();
    PIDController_Init(pid,config.pid_config.kp,config.pid_config.ki,config.pid_config.kd,PID_TAU,PID_LIM_MIN, PID_LIM_MAX, PID_LIM_MIN_INT, PID_LIM_MAX_INT, config.pid_config.samplerate_ms/1000.0);

    aux.PIDcontroller   = pid;
    aux.setpoint        = 0; 
    aux.pid_measure     = 0;
    aux.isSatured       = config.pid_config.isSatured;
    aux.contmode        = CONTMODE_OPEN_LOOP;

    memcpy(object, &aux, sizeof(aux));

    /* Adiciona os comandos de terminal */
#ifdef CONFIG_CONSOLE_INTERFACE_MODULE_ENCMOT_INTERFACE
    register_encmot_newInstance(object);
#endif

    return(object);
}

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
    object->pid_measure = encoder_sample.speed * SPEED_CONSTANT;

    /* Roda a subrotina de controle */
    pid_out = PIDController_Update(object->PIDcontroller, object->setpoint, object->pid_measure, debugOut);

    return(pid_out);
}

static double __contmode_pid_position (encmot_h handler, encoder_sample_t encoder_sample, PIDControllerDebugStream_t *debugOut)
{
    encmot_t *object = handler;
    double pid_out;

    /* Coloca os dados de saída do encoder, na entrada do PID  */
    object->pid_measure = encoder_sample.angle;

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
        stream_out->type = DEBUG_STREAM;
        PIDStream = &stream_out->PID;
    }


    /* Roda a subrotina do encoder */
    encoder_job(object->encoder);
    encoder_sample = encoder_get_lastSample(object->encoder);



    switch(object->contmode)
    {
        case CONTMODE_OPEN_LOOP:
            controller_output = __contmode_openloop(handler);
            memset(PIDStream, 0, sizeof(PIDControllerDebugStream_t));
            break;

        case CONTMODE_PID_SPEED:
            controller_output = __contmode_pid_speed(handler, encoder_sample, PIDStream);   // retorna um valor entre -1 e 1
            motor_set_speed(object->motor, controller_output);                              // Configura um valor entre -1 e 1
            break;

        case CONTMODE_PID_POSITION:
            controller_output = __contmode_pid_position(handler, encoder_sample, PIDStream);
            motor_set_speed(object->motor, controller_output);
            break;
    }

}

void encmot_get_setpoint (encmot_h handler, double *setpoint, controlerMode_t *contmode)
{
    encmot_t *object = handler;
    assert(handler!=NULL);   

    if (setpoint != NULL)
        *setpoint = object->setpoint;

    if (contmode != NULL)
        *contmode = object->contmode;
}

/**
 * @brief Define a velocidade desejada para o motor
 * 
 * @param handler Contexto do objeto
 * @param setpoint Velocidade desejada em radianos por segundo
 */
void encmot_set_speed (encmot_h handler, double setpoint)
{
    const char *ACTION = TAG_ACTION_SET_SPEED;
    encmot_t *object = handler;
    assert(handler!=NULL);

    ESP_LOGI(ACTION,"(%p) setpoint = %e rad/s",handler, setpoint);

    // Coloca o PID em modo de controle de velocidade
    if (object->contmode != CONTMODE_PID_SPEED)
    {
        ESP_LOGD(ACTION,"Switching to CONTMODE_PID_SPEED");
        object->contmode = CONTMODE_PID_SPEED;
    }

    // TODO: Normalizar a velocidade entre -1 e 1
    object->setpoint = setpoint * SPEED_CONSTANT;

    ESP_LOGI(ACTION,"Done!");
}

void encmot_read_speed (encmot_h handler, double *speed)
{
    encmot_t *object = handler;
    assert(handler!=NULL);
    *speed = encoder_get_lastSample(object->encoder).speed * SPEED_CONSTANT;
}



void encmot_set_position (encmot_h handler, double setpoint)
{
    const char *ACTION = TAG_ACTION_SET_POSITION;
    encmot_t *object = handler;
    assert(handler!=NULL);

    ESP_LOGI(ACTION,"(%p) setpoint = %e rad",handler, setpoint);

    // Coloca o PID em modo de controle de velocidade
    if (object->contmode != CONTMODE_PID_POSITION)
    {
        ESP_LOGD(ACTION,"Switching to CONTMODE_PID_POSITION");
        object->contmode = CONTMODE_PID_POSITION;
    }

    // TODO: Normalizar a posicao para radianos
    object->setpoint = setpoint;

    ESP_LOGI(ACTION,"Done!");
}

void encmot_read_position (encmot_h handler, double *position)
{
    encmot_t *object = handler;
    assert(handler!=NULL);
    *position = encoder_get_lastSample(object->encoder).angle;
}

void encmot_tune_pid (encmot_h handler, double kp, double ki, double kd)
{
    encmot_t *object = handler;
    assert(handler!=NULL);

    //TODO: Check if contmode is running
    
    PIDController_tune_pid(object->PIDcontroller, kp, ki, kd);
}


void encmot_get_pid (encmot_h handler, double *kp, double *ki, double *kd)
{
    encmot_t *object = handler;
    assert(handler!=NULL);

    PIDController_get_pid(object->PIDcontroller, kp, ki, kd);
}   




 





