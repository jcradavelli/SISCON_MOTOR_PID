#include "myConsole.h"



#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_vfs_cdcacm.h"
#include "nvs.h"
#include "nvs_flash.h"
// #include "cmd_nvs.h"
// #include "cmd_system.h"
// #include "cmd_wifi.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "system_comands.h"
#include "driver/uart.h"







static const char *TAG = "myConsole";



/*
 * We warn if a secondary serial console is enabled. A secondary serial console is always output-only and
 * hence not very useful for interactive console applications. If you encounter this warning, consider disabling
 * the secondary serial console in menuconfig unless you know what you are doing.
 */
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#if !CONFIG_ESP_CONSOLE_SECONDARY_NONE
#warning "A secondary serial console is not useful when using the console component. Please disable it in menuconfig."
#endif
#endif


static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void initialize_console(void)
{
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_cdcacm_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_cdcacm_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, 0);
    fcntl(fileno(stdin), F_SETFL, 0);

    /* Initialize the console */
    esp_console_config_t console_config = {
            .max_cmdline_args = 8,
            .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
            .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(10);
}

void console_task(void* args)
{
    assert(args != NULL);
    const char* prompt = args;

    /* Main loop */
    while(true) {
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char* line = linenoise(prompt);
        if (line == NULL) { /* Ignore empty lines */
            continue;
        }
        /* Add the command to the history */
        linenoiseHistoryAdd(line);

        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }
}


//typedef int (*vprintf_like_t)(const char *, va_list);

void vprintf_into_uart0_init ()
{
    const int BUF_SIZE = 256;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
}

int vprintf_into_uart0(const char * str, va_list argList)
{
    const int BUF_SIZE = 256;
    char buf[BUF_SIZE];
    int len = vsnprintf(buf, BUF_SIZE, str, argList);
    uart_write_bytes(UART_NUM_0, (const char*)buf, len);
    return len;
}



void create_tsk_console (tskConsole_args_t* tskConsolArgs, UBaseType_t prioridade ,const BaseType_t xCoreID)
{

    initialize_nvs();

    vprintf_into_uart0_init();
    esp_log_set_vprintf(vprintf_into_uart0);


    initialize_console();

    /* Register commands */
    esp_console_register_help_command();
    register_system();
    // register_system_sleep();
    // register_nvs();
    // register_wifi();

    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char* prompt = LOG_COLOR_I CONFIG_IDF_TARGET "> " LOG_RESET_COLOR;

    printf("\n"
           "%s"
           "Version: %s\n"
           "Type 'help' to get the list of commands.\n", tskConsolArgs->gretings, tskConsolArgs->version);

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
    #if CONFIG_LOG_COLORS
            /* Since the terminal doesn't support escape sequences,
            * don't use color codes in the prompt.
            */
            prompt = CONFIG_IDF_TARGET "> ";
    #endif //CONFIG_LOG_COLORS

    }


    xTaskCreatePinnedToCore(console_task, "console", /* Stack Size = */ 4*2048 , prompt, prioridade, NULL, xCoreID);
}
