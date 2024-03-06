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
        set = 0.0015;
    else
        set = -0.0015;

    status = !status;

    encmot_set_position (encmot, set);
}

void stop_mottor (encmot_h encmot)
{
    // encmot_stop(encmot); // TODO criar função
}
                

float kp=10000.0, ki=0.0, kd = 0.0;
float* selected = &kp;
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

    while (1) {
       
        vTaskDelay(pdTICKS_TO_MS(1));

        encmot_runn (encmot);
        
        gpio_set_level(LED_OUT,satured);


        if (butonsClear == true)
        {

            if (!gpio_get_level(BUTTON_1))
            {
                ESP_LOGD(TAG,"CONTINUE/SELECT");
                togle_setpoint (encmot);
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
                stop_mottor(encmot);

                if (selected == &kp) selected = &ki;
                else if (selected == &ki) selected = &kd;
                else if (selected == &kd) selected = &kp;
                butonsClear = false;
            }

        }
        else if (gpio_get_level(BUTTON_1) && gpio_get_level(BUTTON_2) && gpio_get_level(BUTTON_3) && gpio_get_level(BUTTON_4))
        {
            butonsClear = true;
            encmot_tune_pid(encmot,kp,ki,kd); // TODO: Ajustar PID
            ESP_LOGD(TAG,"GANHOS-------------------\n>kp:%e\n>ki:%e\n>kd:%e\n------------------",kp,ki,kd);

        }
    }
}
