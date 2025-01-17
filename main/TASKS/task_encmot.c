


#include "task_encmot.h"
#define TAG "tskEncmot"

static void tsk_encmot (void *args)
{
    tskEncmot_args_t *this;
    encmotDebugStream_t encmotStream;
    this = args;
    assert (args != NULL);
    assert (this->logQueue != NULL);

    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        encmot_job (this->encmot, &encmotStream);

        if (xQueueSend(this->logQueue, &encmotStream, 0) != pdPASS)
        {
            ESP_LOGE(TAG, "LogQueue FULL!");
        }
    }
}


void create_tsk_encmot (tskEncmot_args_t* tskInputArgs, UBaseType_t prioridade ,const BaseType_t xCoreID)
{
    xTaskCreatePinnedToCore(tsk_encmot, "encmot", /* Stack Size = */ 4*2048 , tskInputArgs, prioridade, NULL, xCoreID);

}