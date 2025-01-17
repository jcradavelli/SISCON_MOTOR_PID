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
#include "myConsole.h"
#include "Version_tools.h"

#include "esp_rom_sys.h"
#include "soc/reset_reasons.h"
  





//TODO: Display
// https://github.com/espressif/esp-idf/blob/v5.2.1/examples/peripherals/lcd/i2c_oled/main/i2c_oled_example_main.c


#define USE_SERVO_24H55M020
//#define USE_SERVO_DC_GA25



tskInput_args_t tskInputArgs;
tskGraph_args_t tskGraphArgs;
tskEncmot_args_t tskEncmotArgs;
tskConsole_args_t tskConsoleArgs;

#ifdef USE_SERVO_DC_GA25
static double kp=5000.0, ki=15000.0, kd = 50.0;
#endif

#ifdef USE_SERVO_24H55M020
static double kp=0.07, ki=0.01, kd = 0.0;
#endif





void app_main(void)
{
    const tskConsole_args_t console_config = {    
        .gretings = 
            "\n\n\n\n\n\n\n\n"
            "\033[0;36m" // Cor ciano
            "===============================================================================================================\n\n\n"                                      
            "  @@@@@@@@@@@@@ \n"
            " @@@          @@ \n"
            " @@     @@@    @@ \n"
            " @     @@@@@@@  @@@@@@@@@@ \n"
            " @     @@   @@@@  @@@@@@@@@             ## ##     ## ##       ####  #######     ########  ######   ######     \n"
            " @@@   @@      @@@@@      @@            ## ##     ## ##        ##  ##     ##       ##    ##    ## ##    ##    \n"
            "  @@@  @@         @@@@@    @@           ## ##     ## ##        ##  ##     ##       ##    ##       ##          \n"
            "   @@  @@                   @           ## ##     ## ##        ##  ##     ##       ##    ##       ##          \n"
            "  @@@  @@          @@@@    @@     ##    ## ##     ## ##        ##  ##     ##       ##    ##       ##          \n" 
            " @@    @@       @@@@@     @@      ##    ## ##     ## ##        ##  ##     ##       ##    ##    ## ##    ##    \n" 
            " @     @@    @@@@  @@@@@@@@        ######   #######  ######## ####  #######        ##     ######   ######     \n"
            " @     @@ @@@@@ @@@@@@@@@@ \n"
            " @@    @@@@@   @@              Controlador de Manipulador Esferico Paralelo   \n"
            "  @@          @@               2025/1                                         \n"
            "   @@@@@@@@@@@@ \n"
            "    @@@@@@@@@ \n\n\n"
            "===============================================================================================================\n"
            LOG_RESET_COLOR
            ,
        .version = version,
    };


    switch (esp_reset_reason())
    {
        case ESP_RST_UNKNOWN    : //!< Reset reason can not be determined
            ESP_LOGI("RESET_CAUSE", "Reset reason can not be determined");
            break;
        case ESP_RST_POWERON    : //!< Reset due to power-on event
            ESP_LOGI("RESET_CAUSE", "Reset due to power-on event");
            break;
        case ESP_RST_EXT        : //!< Reset by external pin (not applicable for ESP32)
            ESP_LOGI("RESET_CAUSE", "Reset by external pin (not applicable for ESP32)");
            break;
        case ESP_RST_SW         : //!< Software reset via esp_restart
            ESP_LOGI("RESET_CAUSE", "Software reset via esp_restart");
            break;
        case ESP_RST_PANIC      : //!< Software reset due to exception/panic
            ESP_LOGI("RESET_CAUSE", "Software reset due to exception/panic");
            break;
        case ESP_RST_INT_WDT    : //!< Reset (software or hardware) due to interrupt watchdog
            ESP_LOGI("RESET_CAUSE", "Reset (software or hardware) due to interrupt watchdog");
            break;
        case ESP_RST_TASK_WDT   : //!< Reset due to task watchdog
            ESP_LOGI("RESET_CAUSE", "Reset due to task watchdog");
            break;
        case ESP_RST_WDT        : //!< Reset due to other watchdogs
            ESP_LOGI("RESET_CAUSE", "Reset due to other watchdogs");
            break;
        case ESP_RST_DEEPSLEEP  : //!< Reset after exiting deep sleep mode
            ESP_LOGI("RESET_CAUSE", "Reset after exiting deep sleep mode");
            break;
        case ESP_RST_BROWNOUT   : //!< Brownout reset (software or hardware)
            ESP_LOGI("RESET_CAUSE", "Brownout reset (software or hardware)");
            break;
        case ESP_RST_SDIO       : //!< Reset over SDIO
            ESP_LOGI("RESET_CAUSE", "Reset over SDIO");
            break;
        case ESP_RST_USB        : //!< Reset by USB peripheral
            ESP_LOGI("RESET_CAUSE", "Reset by USB peripheral");
            break;
        case ESP_RST_JTAG       : //!< Reset by JTAG
            ESP_LOGI("RESET_CAUSE", "Reset by JTAG");
            break;
        case ESP_RST_EFUSE      : //!< Reset due to efuse error
            ESP_LOGI("RESET_CAUSE", "Reset due to efuse error");
            break;
        case ESP_RST_PWR_GLITCH : //!< Reset due to power glitch detected
            ESP_LOGI("RESET_CAUSE", "Reset due to power glitch detected");
            break;
        case ESP_RST_CPU_LOCKUP : //!< Reset due to CPU lock up
            ESP_LOGI("RESET_CAUSE", "Reset due to CPU lock up");
            break;
        default:
            ESP_LOGI("RESET_CAUSE", "Reset reason code %d can not be determined", esp_reset_reason());

    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    create_tsk_console(&console_config, configMAX_PRIORITIES/2, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

        /*
    ESP_LOG_INFO
    ESP_LOG_ERROR
    ESP_LOG_NONE
    */
    esp_log_level_set("*",                  ESP_LOG_NONE);
    // esp_log_level_set("EncMot",             ESP_LOG_INFO);
    // esp_log_level_set("Seting_Speed",       ESP_LOG_INFO);
    // esp_log_level_set("motor_24H55M020",    ESP_LOG_INFO);
    // esp_log_level_set("motor",              ESP_LOG_ERROR);
    // esp_log_level_set("encoder",            ESP_LOG_NONE);


    bool satured = false;

#ifdef USE_SERVO_DC_GA25    
    const encmot_config_t encmot_contig     = {
        .encoder_config = {
            .gpio_encoder_a                     = EXAMPLE_EC11_GPIO_A,
            .gpio_encoder_b                     = EXAMPLE_EC11_GPIO_B,
            .max_glitch_ns                      = 100,
            // .gear_ratio_numerator               = 49,
            // .gear_ration_denominator            = 20000,
        },
        .motor_config = {
            .DC_GA25.motor_control_a            = MOTOR_CONTROL_A,
            .DC_GA25.motor_control_b            = MOTOR_CONTROL_B,
        },
        .pid_config = {
            .kp                                 = kp,
            .ki                                 = ki,
            .kd                                 = kd,           
            .samplerate_ms                      = 130,
            .isSatured                          = &satured,
        }
    };
#endif

#ifdef USE_SERVO_24H55M020
    const encmot_config_t encmot_contig     = {
        .encoder_config = {
            .gpio_encoder_a                     = GPIO_NUM_5,
            .gpio_encoder_b                     = GPIO_NUM_4,
            .max_glitch_ns                      = 10,
            // .gear_ratio_numerator               = 49,
            // .gear_ration_denominator            = 20000,
            .pulses_per_revolution              = 100,
        },
        .motor_config = {
            .model                              = MOTOR_MODEL_SERVO_24H55M020,
            .SERVO_24H55M020 = {
                .break_gpio                     = GPIO_NUM_1,
                .cw_ccw_gpio                    = GPIO_NUM_21,
                .speed_gpio                     = GPIO_NUM_2,
                .start_stop_gpio                = GPIO_NUM_42,
                .status_gpio                    = GPIO_NUM_19,
                .speed_max                      = 1.39,
                .speed_min                      = 0.00863,
                .speed_offset                   = 0,
                .speed_gain                     = 39834,
                .timer_num                      = LEDC_TIMER_0,
            },
        },
        .pid_config = {
            .kp                                 = kp,
            .ki                                 = ki,
            .kd                                 = kd,           
            .samplerate_ms                      = 130,
            .isSatured                          = &satured,
        }
    };
#endif



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







}
