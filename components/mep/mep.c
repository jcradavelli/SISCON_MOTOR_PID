#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "sdkconfig.h"


#include "EncMot.h"
// #include "task_encmot.h"
#include "MEP_CINEMATICS.h"


#include "mep.h"

bool satured = false;

// #if CONFIG_DC_GA25 == y
// #error "MEP no soporta DC_GA25"
// #endif

#define TAG "mep"



typedef struct mep_{

    encmot_h encmot[3];
    QueueHandle_t LogQueue;

}mep_t;




#ifdef CONFIG_SERVO_24H55M020

#define GAIN_KP     ((double)CONFIG_MEP_SERVO_24H55M020_KP_NUM/(double)CONFIG_MEP_SERVO_24H55M020_KP_DEN)
#define GAIN_KI     ((double)CONFIG_MEP_SERVO_24H55M020_KI_NUM/(double)CONFIG_MEP_SERVO_24H55M020_KI_DEN)
#define GAIN_KD     ((double)CONFIG_MEP_SERVO_24H55M020_KD_NUM/(double)CONFIG_MEP_SERVO_24H55M020_KD_DEN)
#define SPEED_MAX   ((double)CONFIG_MEP_SERVO_24H55M020_SPEED_MAX_NUM/(double)CONFIG_MEP_SERVO_24H55M020_SPEED_MAX_DEN)   
#define SPEED_MIN   ((double)CONFIG_MEP_SERVO_24H55M020_SPEED_MIN_NUM/(double)CONFIG_MEP_SERVO_24H55M020_SPEED_MIN_DEN)




const encmot_config_t encmot_config[3]     =
{
    {
        .encoder_config = {
            .gpio_encoder_a                     = CONFIG_MEP_SERVO_24H55M020_CH1_NCODER_A_GPIO,
            .gpio_encoder_b                     = CONFIG_MEP_SERVO_24H55M020_CH1_NCODER_B_GPIO,
            .max_glitch_ns                      = CONFIG_MEP_SERVO_24H55M020_MAX_GLITCH_NS,
            .pulses_per_revolution              = CONFIG_MEP_SERVO_24H55M020_PULSES_PER_REVOLUTION,
        },
        .motor_config = {
            .model                              = MOTOR_MODEL_SERVO_24H55M020,
            .SERVO_24H55M020 = {
                .speed_gpio                     = CONFIG_MEP_SERVO_24H55M020_CH1_SPEED_GPIO,
                .break_gpio                     = CONFIG_MEP_SERVO_24H55M020_CH1_BREAK_GPIO,
                .cw_ccw_gpio                    = CONFIG_MEP_SERVO_24H55M020_CH1_CW_CCW_GPIO,
                .start_stop_gpio                = CONFIG_MEP_SERVO_24H55M020_CH1_START_STOP_GPIO,
                .status_gpio                    = CONFIG_MEP_SERVO_24H55M020_CH1_STATUS_GPIO,
                .speed_max                      = SPEED_MAX,
                .speed_min                      = SPEED_MIN,
                .speed_offset                   = CONFIG_MEP_SERVO_24H55M020_SPEED_OFFSET,
                .speed_gain                     = CONFIG_MEP_SERVO_24H55M020_SPEED_GAIN,
                .timer_num                      = LEDC_TIMER_0,
            },
        },
        .pid_config = {
            .kp                                 = GAIN_KP,
            .ki                                 = GAIN_KI,
            .kd                                 = GAIN_KD,
            .samplerate_ms                      = CONFIG_MEP_SERVO_24H55M020_SAMPLERATE_MS,
            .isSatured                          = &satured,
        }
    },
    {
        .encoder_config = {
            .gpio_encoder_a                     = CONFIG_MEP_SERVO_24H55M020_CH2_NCODER_A_GPIO,
            .gpio_encoder_b                     = CONFIG_MEP_SERVO_24H55M020_CH2_NCODER_B_GPIO,
            .max_glitch_ns                      = CONFIG_MEP_SERVO_24H55M020_MAX_GLITCH_NS,
            .pulses_per_revolution              = CONFIG_MEP_SERVO_24H55M020_PULSES_PER_REVOLUTION,
        },
        .motor_config = {
            .model                              = MOTOR_MODEL_SERVO_24H55M020,
            .SERVO_24H55M020 = {
                .speed_gpio                     = CONFIG_MEP_SERVO_24H55M020_CH2_SPEED_GPIO,
                .break_gpio                     = CONFIG_MEP_SERVO_24H55M020_CH2_BREAK_GPIO,
                .cw_ccw_gpio                    = CONFIG_MEP_SERVO_24H55M020_CH2_CW_CCW_GPIO,
                .start_stop_gpio                = CONFIG_MEP_SERVO_24H55M020_CH2_START_STOP_GPIO,
                .status_gpio                    = CONFIG_MEP_SERVO_24H55M020_CH2_STATUS_GPIO,
                .speed_max                      = SPEED_MAX,
                .speed_min                      = SPEED_MIN,
                .speed_offset                   = CONFIG_MEP_SERVO_24H55M020_SPEED_OFFSET,
                .speed_gain                     = CONFIG_MEP_SERVO_24H55M020_SPEED_GAIN,
                .timer_num                      = LEDC_TIMER_1,
            },
        },
        .pid_config = {
            .kp                                 = GAIN_KP,
            .ki                                 = GAIN_KI,
            .kd                                 = GAIN_KD,
            .samplerate_ms                      = CONFIG_MEP_SERVO_24H55M020_SAMPLERATE_MS,
            .isSatured                          = &satured,
        }
    },
    {
        .encoder_config = {
            .gpio_encoder_a                     = CONFIG_MEP_SERVO_24H55M020_CH3_NCODER_A_GPIO,
            .gpio_encoder_b                     = CONFIG_MEP_SERVO_24H55M020_CH3_NCODER_B_GPIO,
            .max_glitch_ns                      = CONFIG_MEP_SERVO_24H55M020_MAX_GLITCH_NS,
            .pulses_per_revolution              = CONFIG_MEP_SERVO_24H55M020_PULSES_PER_REVOLUTION,
        },
        .motor_config = {
            .model                              = MOTOR_MODEL_SERVO_24H55M020,
            .SERVO_24H55M020 = {
                .speed_gpio                     = CONFIG_MEP_SERVO_24H55M020_CH3_SPEED_GPIO,
                .break_gpio                     = CONFIG_MEP_SERVO_24H55M020_CH3_BREAK_GPIO,
                .cw_ccw_gpio                    = CONFIG_MEP_SERVO_24H55M020_CH3_CW_CCW_GPIO,
                .start_stop_gpio                = CONFIG_MEP_SERVO_24H55M020_CH3_START_STOP_GPIO,
                .status_gpio                    = CONFIG_MEP_SERVO_24H55M020_CH3_STATUS_GPIO,
                .speed_max                      = SPEED_MAX,
                .speed_min                      = SPEED_MIN,
                .speed_offset                   = CONFIG_MEP_SERVO_24H55M020_SPEED_OFFSET,
                .speed_gain                     = CONFIG_MEP_SERVO_24H55M020_SPEED_GAIN,
                .timer_num                      = LEDC_TIMER_2,
            },
        },
        .pid_config = {
            .kp                                 = GAIN_KP,
            .ki                                 = GAIN_KI,
            .kd                                 = GAIN_KD,
            .samplerate_ms                      = CONFIG_MEP_SERVO_24H55M020_SAMPLERATE_MS,
            .isSatured                          = &satured,
        }
    }
};
#endif



static void mep_task(mep_h instance)
{
    assert(instance != NULL);
    mep_t *this = instance;
    assert(this->LogQueue != NULL);

    encmotDebugStream_t encmotStream;

    int log_motor = -1; // seliciona o log de motor a ser exibido


    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        // roda os jobs do encmot
        for (int i = 0; i<3; i++)
        {
            encmot_job (this->encmot[i], &encmotStream);

            if (i == log_motor)
            {
                if (xQueueSend(this->LogQueue, &encmotStream, 0) != pdPASS)
                    ESP_LOGE(TAG, "LogQueue FULL!");
            }
        }
    }
}



mep_h mep_init(QueueHandle_t LogQueue)
{
    mep_t* this = malloc(sizeof(mep_t));

    assert(this != NULL);
    assert(LogQueue != NULL);

    for (int i = 0; i<3; i++)
    {
        this->encmot[i] = encmot_attach(encmot_config[i]);
    }

    xTaskCreate(mep_task, "mep_task", /* Stack Size = */ 4*2048 , this, configMAX_PRIORITIES/2+1, NULL);



    return(this);

}


void mep_setPosition_byNormal (mep_h instance, const double normal[3])
{
    assert(instance != NULL);
    mep_t *this = instance;

    double out_v1[3], out_v2[3], out_v3[3];// intermediate values
    double Vx[3], Vy[3], Vz[3];
    double theta[3];
    mep_cinematics_t mep_parameters = {
        .alpha = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
        .gamma = DEG_TO_RAD(0),
        .mounting_mode = MEP_CINEMATICS_MODE_RRR
    };


    getVectorsFromNormal(normal, out_v1, out_v2, out_v3);

    Vx[0] = out_v1[0];
    Vx[1] = out_v2[0];
    Vx[2] = out_v3[0];

    Vy[0] = out_v1[1];
    Vy[1] = out_v2[1];
    Vy[2] = out_v3[1];

    Vz[0] = out_v3[2];
    Vz[1] = out_v3[2];
    Vz[2] = out_v3[2];

    getAnglesFromVectors(mep_parameters, theta, Vx, Vy, Vz);

    encmot_set_position(this->encmot[0], theta[0]);
    encmot_set_position(this->encmot[1], theta[1]);
    encmot_set_position(this->encmot[2], theta[2]);
}