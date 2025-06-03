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

#define MEP_ENABLE_POWER_GPIO (CONFIG_MEP_ENABLE_POWER_GPIO)
#define MEP_ENABLE_POWER_ENABLE_ON_HIGH (CONFIG_MEP_ENABLE_POWER_ENABLE_ON_HIGH)


const encmot_config_t encmot_config[3]     =
{
    { // 1
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
                .start_stop_gpio                = -1,   // não é usado
                .status_gpio                    = -1,       // não é usado - utiliza-se uma entrada para todos os motores 
                .speed_max                      = SPEED_MAX,
                .speed_min                      = SPEED_MIN,
                .speed_offset                   = CONFIG_MEP_SERVO_24H55M020_SPEED_OFFSET,
                .speed_gain                     = CONFIG_MEP_SERVO_24H55M020_SPEED_GAIN,
                .timer_num                      = LEDC_TIMER_2,
            },
        },
        .pid_config = {
            .kp                                 = 0.8,
            .ki                                 = 0.45,
            .kd                                 = 0.0085,
            .samplerate_ms                      = CONFIG_MEP_SERVO_24H55M020_SAMPLERATE_MS,
            .isSatured                          = &satured,
        }
    },
    { //2
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
                .start_stop_gpio                = -1, // não é usado
                .status_gpio                    = -1,     // não é usado - utiliza-se uma entrada para todos os motores  
                .speed_max                      = SPEED_MAX,
                .speed_min                      = SPEED_MIN,
                .speed_offset                   = CONFIG_MEP_SERVO_24H55M020_SPEED_OFFSET,
                .speed_gain                     = CONFIG_MEP_SERVO_24H55M020_SPEED_GAIN,
                .timer_num                      = LEDC_TIMER_0,
            },
        },
        .pid_config = {
            .kp                                 = 0.7,
            .ki                                 = 0.4,
            .kd                                 = 0.009,
            .samplerate_ms                      = CONFIG_MEP_SERVO_24H55M020_SAMPLERATE_MS,
            .isSatured                          = &satured,
        }
    },
    { // 3
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
                .start_stop_gpio                = -1,   // não é usado
                .status_gpio                    = -1,       // não é usado - utiliza-se uma entrada para todos os motores  
                .speed_max                      = SPEED_MAX,
                .speed_min                      = SPEED_MIN,
                .speed_offset                   = CONFIG_MEP_SERVO_24H55M020_SPEED_OFFSET,
                .speed_gain                     = CONFIG_MEP_SERVO_24H55M020_SPEED_GAIN,
                .timer_num                      = LEDC_TIMER_1,
            },
        },
        .pid_config = {
            .kp                                 = 0.7,
            .ki                                 = 0.5,
            .kd                                 = 0.01,
            .samplerate_ms                      = CONFIG_MEP_SERVO_24H55M020_SAMPLERATE_MS,
            .isSatured                          = &satured,
        }
    }
};
#endif



static void __power_on ()
{
    ESP_LOGD(TAG, "Habilitando alimnetação dos motores [GPIO %d]", CONFIG_MEP_ENABLE_POWER_GPIO);
    if (CONFIG_MEP_ENABLE_POWER_ENABLE_ON_HIGH)
    {
        ESP_ERROR_CHECK(gpio_set_level(CONFIG_MEP_ENABLE_POWER_GPIO, 1));
    }
    else
    {
        ESP_ERROR_CHECK(gpio_set_level(CONFIG_MEP_ENABLE_POWER_GPIO, 0));
    }
}

static void __power_off ()
{
    ESP_LOGD(TAG, "Desabilitando alimnetação dos motores [GPIO %d]", CONFIG_MEP_ENABLE_POWER_GPIO);
    if (!CONFIG_MEP_ENABLE_POWER_ENABLE_ON_HIGH)
    {
        ESP_ERROR_CHECK(gpio_set_level(CONFIG_MEP_ENABLE_POWER_GPIO, 1));
    }
    else
    {
        ESP_ERROR_CHECK(gpio_set_level(CONFIG_MEP_ENABLE_POWER_GPIO, 0));
    }
}



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

    
    __power_on();


    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        // Verifica status dos motores


        if (gpio_get_level(CONFIG_MEP_MOTOR_STATUS_GPIO) == 0)
        {        
            // Se ok - nada para fazer
            // Se erro - desliga o sistema
            ESP_LOGE(TAG, "Erro detectado na linha status");
            __power_off();

            ESP_LOGE(TAG, "Interrompendo mep_task!");
            vTaskSuspend(NULL);//interrompe a task
        }


        // roda os jobs do encmot
        for (int i = 0; i<3; i++)
        {
            encmot_job (this->encmot[i], &encmotStream);

            // Stream de dados do PID para exibição 
            //      - formatado para uso com a extensão Teleplot
            // Para habilitar os logs, usa-se o comando log_level do terminal
            //      ou a função set_log_level da esp-idf
            ESP_LOGI(TAGS[i],
                "\n"
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

    const gpio_config_t config_enable_power_gpio = {
        .pin_bit_mask = (1ULL << MEP_ENABLE_POWER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK( gpio_config(&config_enable_power_gpio));

    const gpio_config_t config_motor_status_gpio = {
        .pin_bit_mask = (1ULL << CONFIG_MEP_MOTOR_STATUS_GPIO),
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    ESP_ERROR_CHECK( gpio_config(&config_motor_status_gpio));

    
    

    for (int i = 0; i<3; i++)
    {
        this->encmot[i] = encmot_attach(encmot_config[i]);
    }

    xTaskCreatePinnedToCore(mep_task, "mep_task", /* Stack Size = */ 4*2048 , this, 2*configMAX_PRIORITIES/3, NULL, 1);




    return(this);

}

/**
 * @brief versão modificada da função que posiciona a plataforma de acordo com o azimute e o polar da normal à posição desejada
 * 
 * percebi que a função original estava falhando quando o azimute era muito grande
 * mas o azimute pode ser inserido diretamente noas angulos de saida, sem precisar ser convertido através do vetor normal
 * provavelmente algum erro de arredondamento nessa conversão resulta em erros grandes
 * 
 * Dessa forma a calcula-se apenas a inclinação com o angulo polar e o azimute é adicionado diretamente aos angulos de saída
 * 
 * @param instance          objeto mep sendo controados
 * @param azimuth_graus     graus do azimute, entre a projeção da normal e o eixo X
 * @param polar_graus       graus do polar, em relação ao eixo Z
 */
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

    double azimuth = DEG_TO_RAD(azimuth_graus);
    double polar = DEG_TO_RAD(polar_graus);

    if (azimuth == 0) azimuth = azimuth + 0.00001;
    if (polar == 0) polar = polar + 0.00001;

    // Obtém o vetor normal a partir do azimute e polar
    // Corrigido em 22/04/2025 - desfeito em 29/04/2025 [quando corrigido não resultou em posição coerenteS]
    normal[0] = sin(0) * sin(polar);
    normal[1] = cos(0) * sin(polar);
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

    theta[0] += azimuth;
    theta[1] += azimuth;
    theta[2] += azimuth;

    printf("theta: %f %f %f\n", theta[0], theta[1], theta[2]);

    encmot_set_position(this->encmot[0], theta[0]);
    encmot_set_position(this->encmot[1], theta[1]);
    encmot_set_position(this->encmot[2], theta[2]);
}




void mep_setPosition_byNormal_original (mep_h instance, const double azimuth_graus, const double polar_graus)
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
    // Corrigido em 22/04/2025 - desfeito em 29/04/2025 [quando corrigido não resultou em posição coerenteS]
    normal[0] = sin(azimuth) * sin(polar);
    normal[1] = cos(azimuth) * sin(polar);
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