/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "EncMot.h"
#include "Encoder.h"
#include "motor.h"


#include "task_input.h"
#include "task_encmot.h"
#include "task_graph.h"

static const char *TAG = "example";






         











//TODO: Display
// https://github.com/espressif/esp-idf/blob/v5.2.1/examples/peripherals/lcd/i2c_oled/main/i2c_oled_example_main.c



tskInput_args_t tskInputArgs;
tskGraph_args_t tskGraphArgs;
tskEncmot_args_t tskEncmotArgs;
static float kp=5000.0, ki=15000.0, kd = 50.0;
void app_main(void)
{

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
    EventGroupHandle_t xEventGroup = xEventGroupCreate();

    assert(LogQueue != NULL);
    assert(xEventGroup != NULL);
    
    tskInputArgs.encmot = encmot;
    tskInputArgs.logQueue = LogQueue;
    create_tsk_input (&tskInputArgs, configMAX_PRIORITIES/2, 1);
    
    tskGraphArgs.logQueue = LogQueue;
    create_tsk_graph(&tskGraphArgs, configMAX_PRIORITIES/2-1, 0);

    tskEncmotArgs.encmot = encmot;
    tskEncmotArgs.logQueue = LogQueue;
    create_tsk_encmot (&tskEncmotArgs, configMAX_PRIORITIES/2+1 ,1);


    esp_log_level_set("encoder", ESP_LOG_NONE);

}
