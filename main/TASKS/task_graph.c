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
#define EXAMPLE_PIN_NUM_SDA           17
#define EXAMPLE_PIN_NUM_SCL           16
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
    lv_label_set_text(label, "Te amo Camila");
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

}

void example_lvgl_demo_u2(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);

    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, "Te amo Camila");
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

}

lv_obj_t *scr_default;
lv_obj_t *label_value_kp;
lv_obj_t *label_value_ki;
lv_obj_t *label_value_kd;
lv_obj_t *label_value_sp;
lv_obj_t *label_value_read;

lv_obj_t *scr_updating;
lv_obj_t *label_selected;
lv_obj_t *label_incremen;
lv_obj_t *label_usrValue;

void init_screens(lv_disp_t *disp)
{
    static lv_style_t style_small;
    scr_default  = lv_disp_get_scr_act(disp);
    scr_updating = lv_obj_create(NULL);

    lv_style_init(&style_small);
    lv_style_set_text_font(&style_small, &lv_font_montserrat_10); //TODO: Add font 10

    label_value_kp = lv_label_create(scr_default);
    label_value_ki = lv_label_create(scr_default);
    label_value_kd = lv_label_create(scr_default);
    label_value_sp = lv_label_create(scr_default);
    label_value_read = lv_label_create(scr_default);

    lv_obj_add_style(label_value_kp, &style_small, 0);
    lv_label_set_long_mode(label_value_kp, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label_value_kp, "carregando...");
    lv_obj_set_width(label_value_kp, disp->driver->hor_res);
    lv_obj_align(label_value_kp, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_add_style(label_value_ki, &style_small, 0);
    lv_label_set_long_mode(label_value_ki, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label_value_ki, "");
    lv_obj_set_width(label_value_ki, disp->driver->hor_res);
    lv_obj_align(label_value_ki, LV_ALIGN_TOP_LEFT, 0, 11);

    lv_obj_add_style(label_value_kd, &style_small, 0);
    lv_label_set_long_mode(label_value_kd, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label_value_kd, "");
    lv_obj_set_width(label_value_kd, disp->driver->hor_res);
    lv_obj_align(label_value_kd, LV_ALIGN_TOP_LEFT, 0, 22);

    lv_obj_add_style(label_value_sp, &style_small, 0);
    lv_label_set_long_mode(label_value_sp, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label_value_sp, "");
    lv_obj_set_width(label_value_sp, disp->driver->hor_res);
    lv_obj_align(label_value_sp, LV_ALIGN_TOP_LEFT, 0, 33);

    lv_obj_add_style(label_value_read, &style_small, 0);
    lv_label_set_long_mode(label_value_read, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label_value_read, "");
    lv_obj_set_width(label_value_read, disp->driver->hor_res);
    lv_obj_align(label_value_read, LV_ALIGN_TOP_LEFT, 0, 44);


    label_selected = lv_label_create(scr_updating);
    label_incremen = lv_label_create(scr_updating);
    label_usrValue = lv_label_create(scr_updating);

    lv_label_set_long_mode(label_selected, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label_selected, "<SELECTED>");
    lv_obj_set_width(label_selected, disp->driver->hor_res);
    lv_obj_align(label_selected, LV_ALIGN_TOP_MID, 0, 0);

    lv_label_set_long_mode(label_usrValue, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label_usrValue, "<value>");
    lv_obj_set_width(label_usrValue, disp->driver->hor_res);
    lv_obj_align(label_usrValue, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_style(label_incremen, &style_small, 0);
    lv_label_set_long_mode(label_incremen, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label_incremen, "incremento: <increment>");
    lv_obj_set_width(label_incremen, disp->driver->hor_res);
    lv_obj_align(label_incremen, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_scr_load(scr_default); //carrega a scr_default
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
        .master.clk_speed   = 350000,                        // select frequency specific to your project
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
            .mirror_x = true,
            .mirror_y = true,
        }
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");

    init_screens(disp);
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
    int count = 0;

    assert (args != NULL);
    assert (this->logQueue != NULL);

    init_display();

    while(true)
    {
        xQueueReceive(this->logQueue, &received, portMAX_DELAY);

        switch (received.type)
        {
            case DEBUG_STREAM:
                printf(
                    "\n---------- PID controller job ----------------\n"
                    ">measurement:\t\t%e\n"
                    ">setpoint:\t%e\n"
                    // ">error:\t\t%e\n"
                    ">kp: %e\n"
                    ">ki: %e\n"
                    ">kd: %e\n"
                    // ">proportional:\t%e\n"
                    // ">integrator:\t\t%e\n"
                    // ">differentiator:\t\t%e\n"
                    ">out:\t\t%e\n"
                    "\n----------------------------------------------\n",
                    received.PID.measurement,
                    received.PID.setpoint,
                    // received.PID.error,
                    received.PID.Kp,
                    received.PID.Ki,
                    received.PID.Kd,
                    // received.PID.proportional,
                    // received.PID.integrator,
                    // received.PID.differentiator,
                    received.PID.out
                );

                if (count++ >= 30)
                {
                    lv_label_set_text_fmt(label_value_read, "read: %0.5f", received.PID.measurement);
                    count=0;
                }

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

            break;

            case USR_KP:
                lv_label_set_text_fmt(label_value_kp, "Kp:\t%0.4f", received.kp);
                lv_scr_load(scr_default);
                break;

            case USR_KI:
                lv_label_set_text_fmt(label_value_ki, "Ki:\t%0.4f", received.ki);
                lv_scr_load(scr_default);
                break;

            case USR_KD:
                lv_label_set_text_fmt(label_value_kd, "Kd:\t%0.4f", received.kd);
                lv_scr_load(scr_default);
                break;

            case USR_SP:
                lv_label_set_text_fmt(label_value_sp, "S.P.:\t%0.4f", received.sp);
                lv_scr_load(scr_default);
                break;

            case USR_SEL:
                lv_label_set_text(label_selected, received.sel);
                lv_scr_load(scr_updating);
                break;

            case USR_INC:
                lv_label_set_text_fmt(label_incremen, "incremento:\t%0.4f", received.inc);
                lv_scr_load(scr_updating);
                break;

            case USR_VAL:
                lv_label_set_text_fmt(label_usrValue, "%0.4f", received.usrVal);
                lv_scr_load(scr_updating);
                break;
        }
    }
}

void create_tsk_graph (tskGraph_args_t* tskInputArgs, UBaseType_t prioridade ,const BaseType_t xCoreID)
{
    xTaskCreatePinnedToCore(tsk_graph, "graph", /* Stack Size = */ 4*2048 , tskInputArgs, prioridade, NULL, xCoreID);
}



void __update_inqueue (QueueHandle_t queue, encmotDebugStream_t *display)
{
    if (xQueueSend(queue, display, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "LogQueue FULL!");
    }
}

void update_newValue (QueueHandle_t queue, float value)
{
    encmotDebugStream_t display;
    display.type = USR_VAL;
    display.usrVal = value;
   
    __update_inqueue(queue, &display);
}

void update_increment (QueueHandle_t queue, float value)
{
    encmotDebugStream_t display;
    display.type = USR_INC;
    display.inc = value;
   
    __update_inqueue(queue, &display);
}

void update_pidDebugStream (QueueHandle_t queue, PIDControllerDebugStream_t value)
{
    encmotDebugStream_t display;
    display.type = DEBUG_STREAM;
    display.PID = value;
   
    __update_inqueue(queue, &display);
}

void update_KP (QueueHandle_t queue, float value)
{
    encmotDebugStream_t display;
    display.type = USR_KP;
    display.kp = value;
   
    __update_inqueue(queue, &display);
}

void update_KI (QueueHandle_t queue, float value)
{
    encmotDebugStream_t display;
    display.type = USR_KI;
    display.ki = value;
   
    __update_inqueue(queue, &display);
}

void update_KD (QueueHandle_t queue, float value)
{
    encmotDebugStream_t display;
    display.type = USR_KD;
    display.kd = value;
   
    __update_inqueue(queue, &display);
}

void update_SP (QueueHandle_t queue, float value)
{
    encmotDebugStream_t display;
    display.type = USR_SP;
    display.sp = value;
   
    __update_inqueue(queue, &display);
}

void update_SEL (QueueHandle_t queue, const char *value)
{
    encmotDebugStream_t display;
    display.type = USR_SEL;

    strncpy(display.sel, value, sizeof(display.sel));
   
    __update_inqueue(queue, &display);
}