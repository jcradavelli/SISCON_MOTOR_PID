#include "task_input.h"
#include "task_graph.h"
#define TAG "tskInput"




static double kp, ki, kd, setPoint;

static double* selected = &kp;
static double increment = 1;

static void tsk_input (void *args)
{
    tskInput_args_t *this;
    double counter = 0;
    int counter_aux = 0;
    bool update = false;

    assert (args != NULL);
    this = args;

    bool butonsClear = false;

    vTaskDelay(pdMS_TO_TICKS(1000));
    encmot_get_setpoint(this->encmot, &setPoint, NULL);
    encmot_get_pid(this->encmot, &kp, &ki, &kd);

    ESP_LOGI(TAG,">Selected: kp|t\r\n");
    ESP_LOGI(TAG,">UsrEncoder: %4.4f\r\n", counter);
    ESP_LOGI(TAG,">Increment: %4.4f|t\r\n", increment);

    update_KP(this->logQueue, kp);  
    update_KI(this->logQueue, ki);  
    update_KD(this->logQueue, kd);  
    update_SP(this->logQueue, setPoint);  


    while (1)
    {
        // Atualiza o setpoint
        double newSP;
        encmot_get_setpoint(this->encmot, &newSP, NULL);
        if (newSP != setPoint) update_SP(this->logQueue, newSP); 
        setPoint = newSP;
        
        // Atualiza os ganhos
        double newkp, newki, newkd;
        encmot_get_pid(this->encmot, &newkp, &newki, &newkd);
        if (newkp != kp) update_KP(this->logQueue, newkp);  
        if (newki != ki) update_KI(this->logQueue, newki);  
        if (newkd != kd) update_KD(this->logQueue, newkd);
        newkp = kp; newki = ki; newkd = kd;


        // Atualiza os valores de entrada do usuário
        if (update == true)
        {
            update = false;
            ESP_LOGI(TAG,">UsrEncoder: %4.4f\r\n", counter);
            ESP_LOGI(TAG,">Increment: %4.4f|t\r\n", increment);

            update_increment(this->logQueue, increment);
            update_newValue(this->logQueue, counter);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
            
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

                if (selected == &kp)
                {
                    update_KP(this->logQueue, counter);   
                }
                else if(selected == &ki)
                {
                    update_KI(this->logQueue, counter);
                }
                else if(selected == &kd)
                {
                    update_KD(this->logQueue, counter);
                }
                else if(selected == &setPoint)
                {
                    update_SP(this->logQueue, counter);
                }


                butonsClear = false;
            }

            if (!gpio_get_level(BUTTON_3))
            {
                // aumenta o incremento
                increment = increment * 10;
                update = true;
                butonsClear = false;

                update_increment(this->logQueue, increment);
            }

            if (!gpio_get_level(BUTTON_2))
            {
                // diminui o incremento
                increment = increment / 10;
                update = true;
                butonsClear = false;

                update_increment(this->logQueue, increment);
            }

            // Seleciona o valor a ser modificado
            if (!gpio_get_level(BUTTON_4))
            {
                ESP_LOGD(TAG,"Select");

                if (selected == &setPoint) 
                {
                    ESP_LOGI(TAG,">Selected: kp|t\r\n");
                    selected = &kp;
                    update_SEL(this->logQueue, "kp");
                }
                else if (selected == &kp)
                {
                    ESP_LOGI(TAG,">Selected: ki|t\r\n");
                    selected = &ki;
                    update_SEL(this->logQueue, "ki");
                } 
                else if (selected == &ki) 
                {
                    ESP_LOGI(TAG,">Selected: kd|t\r\n");
                    selected = &kd;
                    update_SEL(this->logQueue, "kd");
                }
                else if (selected == &kd) 
                {
                    ESP_LOGI(TAG,">Selected: setPoint|t\r\n");
                    selected = &setPoint;
                    update_SEL(this->logQueue, "setPoint");
                }

                counter = *selected;
                update = true;
                butonsClear = false;
            }

        }
        else if (gpio_get_level(BUTTON_1) && gpio_get_level(BUTTON_2) && gpio_get_level(BUTTON_3) && gpio_get_level(BUTTON_4))
        {
            butonsClear = true;
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
    };
    tskInputArgs->encoder = encoder_attach(config);
    xTaskCreatePinnedToCore(tsk_input, "input", /* Stack Size = */ 10*2048 , tskInputArgs, prioridade, NULL, xCoreID);
}