#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
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

    const char *TAGS[] = {
        "CH1",
        "CH2",
        "CH3",
    };
    


    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        // roda os jobs do encmot
        for (int i = 0; i<3; i++)
        {
            encmot_job (this->encmot[i], &encmotStream);

            // Stream de dados do PID para exibição 
            //      - formatado para uso com a extensão Teleplot
            // Para habilitar os logs, usa-se o comando log_level do terminal
            //      ou a função set_log_level da esp-idf
            ESP_LOGI(TAGS[i],
                ">CH%d_measurement:\t\t%e\n"
                ">CH%d_setpoint:\t%e\n"
                ">CH%d_error:\t\t%e\n"
                ">CH%d_kp: %e\n"
                ">CH%d_ki: %e\n"
                ">CH%d_kd: %e\n"
                // ">CH%d_proportional:\t%e\n"
                // ">CH%d_integrator:\t\t%e\n"
                // ">CH%d_differentiator:\t\t%e\n"
                ">CH%d_out:\t\t%e\n",
                i, encmotStream.PID.measurement,
                i, encmotStream.PID.setpoint,
                i, encmotStream.PID.error,
                i, encmotStream.PID.Kp,
                i, encmotStream.PID.Ki,
                i, encmotStream.PID.Kd,
                // i, encmotStream.PID.proportional,
                // i, encmotStream.PID.integrator,
                // i, encmotStream.PID.differentiator,
                i, encmotStream.PID.out
            );
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

    xTaskCreate(mep_task, "mep_task", /* Stack Size = */ 4*2048 , this, 2*configMAX_PRIORITIES/3, NULL);



    return(this);

}


void mep_setPosition_byNormal (mep_h instance, const double azimuth_graus, const double polar_graus)
{
    assert(instance != NULL);
    mep_t *this = instance;
    double normal[3];

    double out_v1[3], out_v2[3], out_v3[3];// intermediate values
    double Vx[3], Vy[3], Vz[3];
    double theta[3];
    mep_cinematics_t mep_parameters = {
        .alpha = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
        .gamma = DEG_TO_RAD(0),
        .mounting_mode = MEP_CINEMATICS_MODE_LLL
    };

    const double azimuth = DEG_TO_RAD(azimuth_graus);
    const double polar = DEG_TO_RAD(polar_graus);

    // Obtém o vetor normal a partir do azimute e polar
    // Corrigido em 22/04/2025
    normal[0] = cos(azimuth) * sin(polar);
    normal[1] = sin(azimuth) * sin(polar);
    normal[2] = cos(polar);


    getVectorsFromNormal(normal, out_v1, out_v2, out_v3);
    printf("out_v1: %f %f %f\n", out_v1[0], out_v1[1], out_v1[2]);
    printf("out_v2: %f %f %f\n", out_v2[0], out_v2[1], out_v2[2]);
    printf("out_v3: %f %f %f\n", out_v3[0], out_v3[1], out_v3[2]);

    Vx[0] = out_v1[0];
    Vx[1] = out_v2[0];
    Vx[2] = out_v3[0];

    Vy[0] = out_v1[1];
    Vy[1] = out_v2[1];
    Vy[2] = out_v3[1];

    Vz[0] = out_v1[2];
    Vz[1] = out_v2[2];
    Vz[2] = out_v3[2];

    getAnglesFromVectors(mep_parameters, theta, Vx, Vy, Vz);
    printf("theta: %f %f %f\n", theta[0], theta[1], theta[2]);

    encmot_set_position(this->encmot[0], theta[0]);
    encmot_set_position(this->encmot[1], theta[1]);
    encmot_set_position(this->encmot[2], theta[2]);
}