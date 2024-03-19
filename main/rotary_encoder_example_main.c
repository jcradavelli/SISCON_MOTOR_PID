/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "hal/pcnt_ll.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "EncMot.h"
#include "encoder.h"
#include "motor.h"

static const char *TAG = "example";



#define EXAMPLE_EC11_GPIO_A     16
#define EXAMPLE_EC11_GPIO_B     17
#define MOTOR_CONTROL_A         5
#define MOTOR_CONTROL_B         18
#define BUTTON_1                21
#define BUTTON_2                22
#define BUTTON_3                23
#define BUTTON_4                14
#define LED_OUT                 19


void togle_setpoint (encmot_h encmot)
{
    float set;

    static bool status = false;

    if (status == true)
        set = 0.0027;
    else
        set = -0.0010;

    status = !status;

    encmot_set_speed (encmot, set);
}

void stop_mottor (encmot_h encmot)
{
    // encmot_stop(encmot); // TODO criar função
}
                

typedef struct tskEncmot_args_
{
    encmot_h encmot;
    QueueHandle_t logQueue;
}tskEncmot_args_t;
void tsk_encmot (void *args)
{
    tskEncmot_args_t *this;
    encmotDebugStream_t encmotStream;
    this = args;
    assert (args != NULL);
    assert (this->logQueue != NULL);

    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        encmot_job (this->encmot, &encmotStream);

        if (xQueueSend(this->logQueue, &encmotStream, 0) != pdPASS)
        {
            ESP_LOGE(TAG, "LogQueue FULL!");
        }
    }
}

typedef struct tskGraph_args_
{
    QueueHandle_t logQueue;
}tskGraph_args_t;
void tsk_graph (void *args)
{
    tskGraph_args_t *this;
    encmotDebugStream_t received;
    this = args;

    assert (args != NULL);
    assert (this->logQueue != NULL);

    while(true)
    {
        xQueueReceive(this->logQueue, &received, portMAX_DELAY);

        printf(
            "\n---------- PID controller job ----------------\n"
            ">measurement:\t\t%e\n"
            ">setpoint:\t%e\n"
            // ">error:\t\t%e\n"
            // ">kp: %e\n"
            // ">ki: %e\n"
            // ">kd: %e\n"
            // ">proportional:\t%e\n"
            // ">integrator:\t\t%e\n"
            // ">differentiator:\t\t%e\n"
            ">out:\t\t%d\n"
            "\n----------------------------------------------\n",
            received.PID.measurement,
            received.PID.setpoint,
            // received.PID.error,
            // received.PID.Kp,
            // received.PID.Ki,
            // received.PID.Kd,
            // received.PID.proportional,
            // received.PID.integrator,
            // received.PID.differentiator,
            received.PID.out
        );
    }
}

typedef struct tskInput_args_
{
    encmot_h encmot;
}tskInput_args_t;
float kp=5000.0, ki=15000.0, kd = 50.0;
float* selected = &kp;
void tsk_input (void *args)
{
    tskInput_args_t *this;
    assert (args != NULL);
    this = args;

    bool butonsClear = false;

    while (1)
    {

        vTaskDelay(pdMS_TO_TICKS(100));

        if (butonsClear == true)
        {

            if (!gpio_get_level(BUTTON_1))
            {
                ESP_LOGD(TAG,"CONTINUE/SELECT");
                togle_setpoint (this->encmot);
                //encmot_continue(encmot);
                butonsClear = false;
            }

            if (!gpio_get_level(BUTTON_2))
            {
                // incrementa o ganho
                *selected += 0.1;
                butonsClear = false;
            }

            if (!gpio_get_level(BUTTON_3))
            {
                // decrementa o ganho
                *selected -= 0.1;
                butonsClear = false;
            }

            if (!gpio_get_level(BUTTON_4))
            {
                ESP_LOGD(TAG,"STOP");
                stop_mottor(this->encmot);

                if (selected == &kp) selected = &ki;
                else if (selected == &ki) selected = &kd;
                else if (selected == &kd) selected = &kp;
                butonsClear = false;
            }

        }
        else if (gpio_get_level(BUTTON_1) && gpio_get_level(BUTTON_2) && gpio_get_level(BUTTON_3) && gpio_get_level(BUTTON_4))
        {
            butonsClear = true;
            encmot_tune_pid(this->encmot,kp,ki,kd); // TODO: Ajustar PID
            ESP_LOGD(TAG,"GANHOS-------------------\n>kp:%e\n>ki:%e\n>kd:%e\n------------------",kp,ki,kd);

        }
    }
}



tskInput_args_t tskInputArgs;
tskGraph_args_t tskGraphArgs;
tskEncmot_args_t tskEncmotArgs;
void app_main(void)
{

    bool butonsClear = false;
    bool satured = false;
    
    const encmot_config_t encmot_contig     = {
        .encoder_config = {
            .gpio_encoder_a                     = EXAMPLE_EC11_GPIO_A,
            .gpio_encoder_b                     = EXAMPLE_EC11_GPIO_B,
            .max_glitch_ns                      = 100,
            .gear_ratio_numerator               = 49,
            .gear_ration_denominator            = 20000,
        },
        .motor_config = {
            .motor_control_a                    = MOTOR_CONTROL_A,
            .motor_control_b                    = MOTOR_CONTROL_B,

        },
        .pid_config = {
            .kp                                 = kp,
            .ki                                 = ki,
            .kd                                 = kd,           
            .samplerate_ms                      = 130,
            .isSatured                          = &satured,
        }
    };
    encmot_h encmot = NULL;
    encmot = encmot_attach(encmot_contig);

    // Configura as entradas dos botões
    const gpio_config_t gpio_config_buttons = {
        .pin_bit_mask = BIT(BUTTON_1) | BIT(BUTTON_2) | BIT(BUTTON_3) | BIT(BUTTON_4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_config_buttons));

    // Configura a saída dos leds
    const gpio_config_t gpio_config_led = {
        .pin_bit_mask = BIT(LED_OUT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_config_led));


    QueueHandle_t LogQueue = xQueueCreate(10, sizeof(encmotDebugStream_t));
    
    tskInputArgs.encmot = encmot;
    xTaskCreatePinnedToCore(tsk_input, "input", /* Stack Size = */ 2048 , &tskInputArgs, /* Priority= */configMAX_PRIORITIES/2, NULL, 1);

    tskGraphArgs.logQueue = LogQueue;
    xTaskCreatePinnedToCore(tsk_graph, "graph", /* Stack Size = */ 2048 , &tskGraphArgs, /* Priority= */configMAX_PRIORITIES/2-1, NULL, 0);

    tskEncmotArgs.encmot = encmot;
    tskEncmotArgs.logQueue = LogQueue;
    xTaskCreatePinnedToCore(tsk_encmot, "input", /* Stack Size = */ 2048 , &tskEncmotArgs, /* Priority= */configMAX_PRIORITIES/2+1, NULL, 1);

    esp_log_level_set("encoder", ESP_LOG_NONE);

}
