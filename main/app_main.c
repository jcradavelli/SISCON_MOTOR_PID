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
#include "console_interface.h"
#include "git_tools.h"

#include "esp_rom_sys.h"
#include "soc/reset_reasons.h"

#include "mep.h"
#include "console_interface.h"






//TODO: Display
// https://github.com/espressif/esp-idf/blob/v5.2.1/examples/peripherals/lcd/i2c_oled/main/i2c_oled_example_main.c


#define USE_SERVO_24H55M020
//#define USE_SERVO_DC_GA25



tskInput_args_t tskInputArgs;
tskGraph_args_t tskGraphArgs;
tskEncmot_args_t tskEncmotArgs[3];
tskConsole_args_t tskConsoleArgs;

#ifdef USE_SERVO_DC_GA25
static double kp=5000.0, ki=15000.0, kd = 50.0;
#endif

#ifdef USE_SERVO_24H55M020
//static double kp=0.01, ki=0.0005, kd = 0.0; // posição
static double kp=1, ki=0.0005, kd = 0.0; // velocidade
#endif





void app_main(void)
{
    const tskConsole_args_t console_config = {
        .gretings =
            "\n\n\n\n\n\n\n\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#####################################################################################################################################################\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#####################################################################################################################################################\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)"::::::::::::::                     ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)":::::::::::                            ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)":::::::::                                ::::::::::                                                     :::::::::::::::::::::::::::::::::::::::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)":::::::       "LOG_COLOR(LOG_COLOR_GREEN)" _ _ _ _ _"LOG_COLOR(LOG_COLOR_PURPLE)"                   ::::::::      cCCCCCCC    mM   Mm    eeEEEEEEEe  PPPPPPPpp   :::::::::::::::::::::::::::::::::::::::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)":::::                   #                    ::::::     cCC         mMMM MMMM  eEEe         PPP    PPp  :::::::::::::::::::::::::::::::::::::::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)"::::                   #.#                    :::::     CCC         MM mmm MM  eEEEEEEEEee  PPPPPPPpp   :::::::::::::::::::::::::::::::::::::::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)"::::                  #...#      "LOG_COLOR(LOG_COLOR_BLUE)" / "LOG_COLOR(LOG_COLOR_PURPLE)"          :::::.    cCC         MM  M  MM  eEEe         PP         .:::::::::::::::::::::::::::::::::::::::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)"::::                 #.....#    "LOG_COLOR(LOG_COLOR_BLUE)" /  "LOG_COLOR(LOG_COLOR_PURPLE)"          :::::...   cCCCCCCC   MM     MM   eEEEEEEEee  PP       ...:::::::::::::::::        :::    ::::    :: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)"::::                #.......#  "LOG_COLOR(LOG_COLOR_BLUE)" / "LOG_COLOR(LOG_COLOR_PURPLE)"            :::::....                                            .....::::::::::::::::::::  :::::  ::::::  ::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)"::::               #.........# "LOG_COLOR(LOG_COLOR_BLUE)"/ "LOG_COLOR(LOG_COLOR_PURPLE)"             ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::  :::::  ::::::  ::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)"::::              # # # # # # #               ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::  ::::::    ::::    :: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)"::::              "LOG_COLOR(LOG_COLOR_RED)"\\  "LOG_COLOR(LOG_COLOR_PURPLE)"                         :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)"::::               "LOG_COLOR(LOG_COLOR_RED)"\\  "LOG_COLOR(LOG_COLOR_PURPLE)"                        :::::::::::::::::::::::::::::::::::::::::::::::::::::  :::::::    :::::  ::::     ::::::  ::::  :::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)"::::                "LOG_COLOR(LOG_COLOR_RED)"\\  "LOG_COLOR(LOG_COLOR_PURPLE)"                       :::::::::::::::::::::::::::::::::::::::::::::::::::  ::  ::::  ::  ::  ::  ::  ::::::::  :::  ::  :: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)":::::                "LOG_COLOR(LOG_COLOR_RED)"\\  "LOG_COLOR(LOG_COLOR_PURPLE)"                     ::::::::::::::::::::::::::::::::::::::::::::::::::::::::  :::  :::: ::::::  ::    :::::  ::::::::  :: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)":::::::                                    ::::::::::::::::::::::::::::::::::::::::::::::::::::::::  ::::::  ::  ::::  :::::::  :::  :::::::  :::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)":::::::::::                              ::::::::::::::::::::::::::::::::::::::::::::::::::::::::      :::::    :::      ::    :::  ::::::      :: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#"LOG_COLOR(LOG_COLOR_PURPLE)":::::::::::::::                      ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: "LOG_COLOR(LOG_COLOR_CYAN)"#\n"
            LOG_COLOR(LOG_COLOR_CYAN)"#####################################################################################################################################################\n"
            LOG_COLOR(LOG_COLOR_CYAN)"######################################################################################################################## "LOG_BOLD(LOG_COLOR_PURPLE)" Por julio cesar radavelli"LOG_COLOR(LOG_COLOR_CYAN)" #\n"
            LOG_RESET_COLOR
            ,
        .version = STRING_GIT_DESCRIBE_TAGS_DIRTY_ALWAYS,
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


    //esp_log_level_set("*",                  ESP_LOG_NONE);




    // Configura as entradas dos botões
    // const gpio_config_t gpio_config_buttons = {
    //     .pin_bit_mask = BIT(BUTTON_1) | BIT(BUTTON_2) | BIT(BUTTON_3) | BIT(BUTTON_4),
    //     .mode = GPIO_MODE_INPUT,
    //     .pull_up_en = GPIO_PULLUP_ENABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE,
    // };
    // ESP_ERROR_CHECK(gpio_config(&gpio_config_buttons));


    QueueHandle_t LogQueue = xQueueCreate(10, sizeof(encmotDebugStream_t));
    EventGroupHandle_t xEventGroup = xEventGroupCreate();

    assert(LogQueue != NULL);
    assert(xEventGroup != NULL);
    
    //encmot_h            encmot[3] = {};
    mep_h mep = mep_init(LogQueue);

    // registra o MEP no console
    console_interface_mep_t* interface_mep = malloc(sizeof(console_interface_mep_t));
    assert(interface_mep != NULL);
    interface_mep->handler = mep;
    interface_mep->setNormal = &mep_setPosition_byNormal;
    register_mep(interface_mep);


    // tskInputArgs.encmot = encmot;
    // tskInputArgs.logQueue = LogQueue;
    // create_tsk_input (&tskInputArgs, configMAX_PRIORITIES/2, 1);

    tskGraphArgs.logQueue = LogQueue;
    create_tsk_graph(&tskGraphArgs, configMAX_PRIORITIES/2-1, 0);




    vTaskDelay(pdMS_TO_TICKS(1000));
    /*
    ESP_LOG_INFO
    ESP_LOG_ERROR
    ESP_LOG_NONE
    */
    //esp_log_level_set("*",                  ESP_LOG_NONE);
    // esp_log_level_set("EncMot",             ESP_LOG_INFO);
    // esp_log_level_set("Seting_Speed",       ESP_LOG_INFO);
    // esp_log_level_set("motor_24H55M020",    ESP_LOG_INFO);
    // esp_log_level_set("motor",              ESP_LOG_ERROR);
    // esp_log_level_set("encoder",            ESP_LOG_NONE);


}
