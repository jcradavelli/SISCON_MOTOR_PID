#include "task_input.h"
#define TAG "tskInput"



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
       

// static float kp=5000.0, ki=15000.0, kd = 50.0, setPoint = 0;
// static float kp=0.0, ki=0.0, kd = 0.0, setPoint = 0;
static float kp=5000.0, ki=15000.0, kd = 50.0, setPoint = 0;

static float* selected = &kp;
static float increment = 1;

static void tsk_input (void *args)
{
    tskInput_args_t *this;
    float counter = 0;
    int counter_aux = 0;
    bool update = true;

    assert (args != NULL);
    this = args;

    bool butonsClear = false;

    vTaskDelay(pdMS_TO_TICKS(1000));
    encmot_set_speed (this->encmot, setPoint);
    encmot_tune_pid(this->encmot,kp,ki,kd);


    printf(">Selected: kp|t\r\n");

    while (1)
    {
        // Update telemetry values
        if (update == true)
        {
            update = false;
            printf(">UsrEncoder: %4.4f\r\n", counter);
            printf(">Increment: %4.4f|t\r\n", increment);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
            
        counter_aux = encoder_get_enconderCount_raw(this->encoder); //se o encoder tiver um valor não multiplo de 4 (passo aprcial) esse passo é perdido

        if ((counter_aux%4 == 0) && (counter_aux/4 !=0))
        {
            counter_aux = counter_aux/4;
            counter += counter_aux*increment;
            encoder_clear_encoderCount(this->encoder);
            update = true;
        }


        if (butonsClear == true)
        {


            if (!gpio_get_level(BUTTON_1))
            {
                ESP_LOGD(TAG,"Aply");

                *selected = counter;

                encmot_set_speed (this->encmot, setPoint);
                encmot_tune_pid(this->encmot,kp,ki,kd);

                butonsClear = false;
            }

            if (!gpio_get_level(BUTTON_2))
            {
                // aumenta o incremento
                increment = increment * 10;
                update = true;
                butonsClear = false;
            }

            if (!gpio_get_level(BUTTON_3))
            {
                // diminui o incremento
                increment = increment / 10;
                update = true;
                butonsClear = false;
            }

            // Seleciona o valor a ser modificado
            if (!gpio_get_level(BUTTON_4))
            {
                ESP_LOGD(TAG,"Select");

                if (selected == &setPoint) 
                {
                    printf(">Selected: kp|t\r\n");
                    selected = &kp;
                }
                else if (selected == &kp)
                {
                    printf(">Selected: ki|t\r\n");
                    selected = &ki;
                } 
                else if (selected == &ki) 
                {
                    printf(">Selected: kd|t\r\n");
                    selected = &kd;
                }
                else if (selected == &kd) 
                {
                    printf(">Selected: setPoint|t\r\n");
                    selected = &setPoint;
                }

                counter = *selected;
                update = true;
                butonsClear = false;
            }

        }
        else if (gpio_get_level(BUTTON_1) && gpio_get_level(BUTTON_2) && gpio_get_level(BUTTON_3) && gpio_get_level(BUTTON_4))
        {
            butonsClear = true;
            // encmot_tune_pid(this->encmot,kp,ki,kd); // TODO: Ajustar PID
            // ESP_LOGD(TAG,"GANHOS-------------------\n>kp:%e\n>ki:%e\n>kd:%e\n------------------",kp,ki,kd);

        }
    }
}


void create_tsk_input (tskInput_args_t* tskInputArgs, UBaseType_t prioridade ,const BaseType_t xCoreID)
{
    encoder_config_t config = {
        .gpio_encoder_a                     =   ENCODER_N,
        .gpio_encoder_b                     =   ENCODER_P,
        .max_glitch_ns                      =   10000,
        .pcnt_high_limit                    =   1024,
        .pcnt_low_limit                     =   -1024,
        .watchpoint_inferior                =   0,
        .watchpoint_targett                 =   0,
        .watchpoint_superior                =   0,
        .example_pcnt_on_reach              =   NULL,
        .example_pcnt_on_reach_user_data    =   NULL,
        .gear_ratio_numerator               =   1,
        .gear_ration_denominator            =   1,
    };
    tskInputArgs->encoder = encoder_attach(config);
    xTaskCreatePinnedToCore(tsk_input, "input", /* Stack Size = */ 2048 , tskInputArgs, prioridade, NULL, xCoreID);
}