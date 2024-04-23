#include "task_graph.h"
#include "EncMot.h"


#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_err.h"
#include "esp_log.h"

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif


#define TAG "tskGraph"

#define I2C_BUS_PORT  0

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           26
#define EXAMPLE_PIN_NUM_SCL           25
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

#define CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306   true

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64

// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#include "lvgl.h"

void example_lvgl_demo_ui(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, "Hello Espressif, Hello LVGL.");
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

}


void init_display ()
{
    ESP_LOGI(TAG, "Initialize I2C bus");
///////////////////////////////////////////////////////////////////////////////////////////////////////
    // i2c_master_bus_handle_t i2c_bus = NULL;
    esp_lcd_i2c_bus_handle_t i2c_bus = NULL;
    i2c_config_t bus_config = {
        .mode               = I2C_MODE_MASTER,
        .sda_io_num         = EXAMPLE_PIN_NUM_SDA,          // select SDA GPIO specific to your project
        .sda_pullup_en      = GPIO_PULLUP_ENABLE,
        .scl_io_num         = EXAMPLE_PIN_NUM_SCL,          // select SCL GPIO specific to your project
        .scl_pullup_en      = GPIO_PULLUP_ENABLE,
        .master.clk_speed   = 10000,                        // select frequency specific to your project
        .clk_flags          = 0,                            // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
    };
    i2c_param_config(I2C_BUS_PORT, &bus_config);
    i2c_driver_install(I2C_BUS_PORT, I2C_MODE_MASTER, 0, 0, 0);
///////////////////////////////////////////////////////////////////////////////////////////////////////

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr           = EXAMPLE_I2C_HW_ADDR,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits       = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits     = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset      = 6,                     // According to SSD1306 datasheet

    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));


    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");

    example_lvgl_demo_ui(disp);
}




typedef enum state_
{
    STATE_UNKNOW,
    STATE_UP,
    STATE_DOW,
}state_t;

int period;
double last_measurement;
state_t state = STATE_UNKNOW;
state_t oldState = STATE_UNKNOW;


void tsk_graph (void *args)
{
    tskGraph_args_t *this;
    encmotDebugStream_t received;
    this = args;

    assert (args != NULL);
    assert (this->logQueue != NULL);

    //init_display();

    while(true)
    {
        xQueueReceive(this->logQueue, &received, portMAX_DELAY);

        printf(
            "\n---------- PID controller job ----------------\n"
            ">measurement(x1000):\t\t%e\n"
            ">setpoint(x1000):\t%e\n"
            // ">error:\t\t%e\n"
            ">kp: %e\n"
            ">ki: %e\n"
            ">kd: %e\n"
            // ">proportional:\t%e\n"
            // ">integrator:\t\t%e\n"
            // ">differentiator:\t\t%e\n"
            ">out:\t\t%d\n"
            "\n----------------------------------------------\n",
            received.PID.measurement*1000,
            received.PID.setpoint*1000,
            // received.PID.error,
            received.PID.Kp,
            received.PID.Ki,
            received.PID.Kd,
            // received.PID.proportional,
            // received.PID.integrator,
            // received.PID.differentiator,
            received.PID.out
        );

        if (received.PID.measurement > last_measurement)
        {
            state = STATE_UP;
        }
        else
        {
            state = STATE_DOW;
        }

        if (oldState == STATE_UP && state == STATE_DOW)
        {
            //ocorreu uma troca de derivada
            printf(">Period_ms: %d", period*100);
            period = 0;
        }

        last_measurement = received.PID.measurement;
        oldState = state;
        period++;
    }
}

void create_tsk_graph (tskGraph_args_t* tskInputArgs, UBaseType_t prioridade ,const BaseType_t xCoreID)
{
    xTaskCreatePinnedToCore(tsk_graph, "graph", /* Stack Size = */ 4*2048 , tskInputArgs, prioridade, NULL, xCoreID);
}




