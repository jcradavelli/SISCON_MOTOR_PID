
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "hal/pcnt_ll.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/uart.h"


#include "Encoder.h"
#include <string.h>
#include <math.h>

static const char *TAG = "encoder";



typedef struct encoder_{
    pcnt_unit_handle_t      pcnt_unit;
    pcnt_channel_handle_t   pcnt_chan_a;
    pcnt_channel_handle_t   pcnt_chan_b;
    // int gear_ratio_numerator;               //!< numerador da fração ratio (PULSOS/voltas)
    // int gear_ration_denominator;            //!< Denominador da fração ration (pulsos/VOLTAS)
    int running_counter;
    encoder_sample_t last_sample;

    double encoderGain;                         //!< Ganho que converte a contagem de pulsos em deslocamento (radianos)
}encoder_t;

static encoder_t __encoder_attach 
(
    int gpio_encoder_a, 
    int gpio_encoder_b, 
    int max_glitch_ns, 
    int pcnt_high_limit,
    int pcnt_low_limit,
    int watchpoint_inferior,
    int watchpoint_targett, 
    int watchpoint_superior,
    pcnt_event_on_reach_callback_h example_pcnt_on_reach,
    void *example_pcnt_on_reach_user_data,
    int pulses_per_revolution
    // int gear_ratio_numerator,           //!< numerador da fração ratio (VOLTAS/pulsos)
    // int gear_ration_denominator    //!< Denominador da fração ration (voltas/PULSOS)
)
{
    encoder_t retval = {0};

    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = pcnt_high_limit,
        .low_limit = pcnt_low_limit,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = max_glitch_ns,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = gpio_encoder_a,
        .level_gpio_num = gpio_encoder_b,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL; //preenchido por 'pcnt_new_channel'
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = gpio_encoder_b,
        .level_gpio_num = gpio_encoder_a,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL; //preenchido por 'pcnt_new_channel'
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    if (example_pcnt_on_reach != NULL)
    {
        ESP_LOGI(TAG, "add watch points and register callbacks");
        int watch_points[] = {pcnt_low_limit, watchpoint_inferior, watchpoint_targett, watchpoint_superior, pcnt_high_limit};
        for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
            ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
        }
        pcnt_event_callbacks_t cbs = {
            .on_reach = example_pcnt_on_reach,
        };
        ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, example_pcnt_on_reach_user_data));
    }

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(gpio_encoder_a, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif


    retval.pcnt_unit = pcnt_unit;
    retval.pcnt_chan_a = pcnt_chan_a;
    retval.pcnt_chan_b = pcnt_chan_b;

    if (pulses_per_revolution < 0)
        retval.encoderGain = 1;
    else
        retval.encoderGain = pulses_per_revolution*M_PI/180.0;

    // retval.gear_ratio_numerator = gear_ratio_numerator;
    // retval.gear_ration_denominator = gear_ration_denominator;

    return(retval);
}



encoder_h encoder_attach (encoder_config_t config)
{
    encoder_t aux;
    encoder_h object = NULL;
    
    // Aloca memoria para o objeto EcnMot
    object = malloc(sizeof(encoder_t));
    if (object == NULL)
    {
        ESP_LOGE(TAG,"Não foi possivel criar o objeto.");
        return(NULL);
    }

    // Configura os periféricos e o objeto encoder
    aux = __encoder_attach (
        config.gpio_encoder_a, 
        config.gpio_encoder_b, 
        config.max_glitch_ns, 
        config.pcnt_high_limit,
        config.pcnt_low_limit,
        config.watchpoint_inferior,
        config.watchpoint_targett, 
        config.watchpoint_superior,
        config.example_pcnt_on_reach,
        config.example_pcnt_on_reach_user_data,
        config.pulses_per_revolution
        // config.gear_ratio_numerator,
        // config.gear_ration_denominator
    );
    memcpy(object, &aux, sizeof(aux));

    return(object);
}

int encoder_get_enconderCount_raw(encoder_h handler)
{
    encoder_t *object = handler;
    int pulse_count;

    assert(handler!=NULL);
    ESP_ERROR_CHECK(pcnt_unit_get_count(object->pcnt_unit, &pulse_count));

    return (pulse_count);
}

void encoder_clear_encoderCount (encoder_h handler)
{
    encoder_t *object = handler;

    assert(handler!=NULL);
    ESP_ERROR_CHECK(pcnt_unit_clear_count(object->pcnt_unit));
}

// static float __encoder_get_turns (encoder_h handler)
// {
//     encoder_t *object = handler;
//     float value;

//     assert(handler!=NULL);
//     value = encoder_get_enconderCount_raw(handler);
//     value = (object->gear_ratio_numerator * value)/(object->gear_ration_denominator);
//     ESP_LOGD(TAG, "\n>Turns: %f", value);

    
//     return(value);
// }

// float encoder_get_encoderPosition_grad(encoder_h handler)
// {
//     float value;

//     assert(handler!=NULL);
//     value = __encoder_get_turns(handler) * 360;

//     return(value);
// }

// float encoder_get_encoderPosition_rad(encoder_h handler)
// {
//     float value;

//     assert(handler!=NULL);
//     value = (__encoder_get_turns(handler) * 2 )/ M_PI;

//     return(value);
// }

void encoder_job (encoder_h handler)
{
    encoder_t *object = handler;
    static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;
    double count;
    float speed=0;
    float acele;
    float jerk;
    int64_t time;
    int dtime;

    assert(handler!=NULL);

    // Faz a aquisição da amostra
    taskENTER_CRITICAL(&my_spinlock);
    count   = encoder_get_enconderCount_raw(handler) * 15707.96327;
    time    = esp_timer_get_time();
    taskEXIT_CRITICAL(&my_spinlock);



    // Calcula as derivadas
    dtime = (time - object->last_sample.time);
    ESP_LOGE(TAG,"\n>debug:%f\r\n",(count - object->last_sample.count));
    ESP_LOGE(TAG,"\n>Limiar:%d\r\n",(SHRT_MAX/2));
    if (abs(count - object->last_sample.count) > (SHRT_MAX/2)* 15707.96327) // Ocorreu overflow ou underflow na contágem?
    {
        ESP_LOGE(TAG,"\n>count:%f\r\n",count);
        ESP_LOGE(TAG,"\n>object->last_sample.count:%f\r\n",object->last_sample.count);
        ESP_LOGE(TAG,"\n>SHRT_MIN:%d\r\n",SHRT_MIN);
        ESP_LOGE(TAG,"\n>SHRT_MAX:%d\r\n",SHRT_MAX);

        if (count > object->last_sample.count) // overflow?
        {
            object->last_sample.count += SHRT_MAX* 15707.96327;
            speed = (float)(count - object->last_sample.count)/(dtime);
        }
        else //underflow!
        {
            object->last_sample.count += SHRT_MIN* 15707.96327; //ok
            speed = (float)(count - object->last_sample.count)/(dtime);
        }
        ESP_LOGE(TAG,"\n>considerado:%f\r\n",(count - object->last_sample.count));
    }
    else
    {
        ESP_LOGE(TAG,"\n>considerado:%f\r\n",(count - object->last_sample.count));
        speed = (float)(count - object->last_sample.count)/(dtime);
    }
    acele = (speed - object->last_sample.speed)/(dtime);
    jerk  = (acele - object->last_sample.acele)/(dtime);

    // Debug dos dados de encoder
    ESP_LOGI(TAG,"\n>time:%d us\r\n",dtime);
    ESP_LOGI(TAG,"\n>count:%f °\r\n",count);
    ESP_LOGI(TAG,"\n>Speed:%0.20e °/s\r\n",speed);
    ESP_LOGI(TAG,"\n>acele:%0.20e °/s²\r\n",acele);
    ESP_LOGI(TAG,"\n>jerk:%0.20e °/s³\r\n",jerk);

    // Atualiza os valores
    object->last_sample.time    = time;
    object->last_sample.count   = count;
    object->last_sample.speed   = speed;
    object->last_sample.acele   = acele;
    object->running_counter++;
}

encoder_sample_t encoder_get_lastSample (encoder_h handler)
{
    encoder_t *object = handler;
    assert(handler!=NULL);

    return(object->last_sample);
}

