#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"



#include "console_encmot.h"


#define TAG "console_encmot"
#include "EncMot.h"

void* registered_encmot_instances[MAX_ENC_MOT_INSTANCES];
static int registered_instances_count = 0;


// =============================================================

static struct {
    struct arg_int *instance;
    struct arg_dbl *kp;
    struct arg_dbl *ki;
    struct arg_dbl *kd;
    struct arg_end *end;
} tunePID;

static int console_encmot_tune_pid (int argc, char **argv)
{
    int errors = arg_parse(argc, argv, (void **) &tunePID);
    if (errors != 0) {
        arg_print_errors(stderr, tunePID.end, argv[0]);
        return 1;
    }
    assert(tunePID.instance->count == 1);
    assert(tunePID.kp->count == 1);
    assert(tunePID.ki->count == 1);
    assert(tunePID.kd->count == 1);

    const int instance = tunePID.instance->ival[0];
    const double kp = tunePID.kp->dval[0];  
    const double ki = tunePID.ki->dval[0];  
    const double kd = tunePID.kd->dval[0];
    
    // chama a função
    encmot_tune_pid(registered_encmot_instances[instance], kp, ki, kd);

    return 0;
}

void registered_encmot_tunePID () 
{
    tunePID.instance = arg_int1(NULL, NULL, "<n>", "Numero da instancia do objeto.");
    tunePID.kp = arg_dbl1("p", "kp", "<dval>", "Valor do ganho proporcional (double).");
    tunePID.ki = arg_dbl1("i", "ki", "<dval>", "Valor do ganho integrativo (double).");
    tunePID.kd = arg_dbl1("d", "kd", "<dval>", "Valor do ganho derivativo (double).");
    tunePID.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "tune_pid",
        .help = "Ajusta os ganhos kp ki e kd respectivamente do controlador PID",
        .hint = NULL,
        .func = &console_encmot_tune_pid,
        .argtable = &tunePID
    };
    esp_console_cmd_register(&cmd);
}




// ===========================================

static struct {
    struct arg_int *instance;
    struct arg_dbl *speed;
    struct arg_end *end;
} setSpeed;

static int console_encmot_set_speed(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &setSpeed);
    if (nerrors != 0) {
        arg_print_errors(stderr, setSpeed.end, argv[0]);
        return 1;
    }
    assert(setSpeed.instance->count == 1);
    assert(setSpeed.speed->count == 1);
    const int instance = setSpeed.instance->ival[0];
    const double speed = setSpeed.speed->dval[0];

    // chama a funcao
    encmot_set_speed(registered_encmot_instances[instance], speed);
    
    return 0;
}

void registered_encmot_setSpeed () 
{
    setSpeed.instance = arg_int1(NULL, NULL, "<n>", "Numero da instancia do objeto.");
    setSpeed.speed = arg_dbl1("v", "speed", "<dval>", "Valor da velocidade em radianos por segundo (double).");
    setSpeed.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "set_speed",
        .help = "Configura a velocidade do motor.",
        .hint = NULL,
        .func = &console_encmot_set_speed,
        .argtable = &setSpeed
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
};



// ===========================================

static struct {
    struct arg_int *instance;
    struct arg_dbl *position;
    struct arg_end *end;
} setPosition;

static int console_encmot_set_position(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &setPosition);
    if (nerrors != 0) {
        arg_print_errors(stderr, setPosition.end, argv[0]);
        return 1;
    }
    assert(setPosition.instance->count == 1);
    assert(setPosition.position->count == 1);
    const int instance = setPosition.instance->ival[0];
    const double position = setPosition.position->dval[0];

    // chama a funcao
    encmot_set_position(registered_encmot_instances[instance], position);
    
    return 0;
}

void registered_encmot_setposition () 
{
    setPosition.instance = arg_int1(NULL, NULL, "<n>", "Numero da instancia do objeto.");
    setPosition.position = arg_dbl1("v", "position", "<dval>", "Valor da velocidade em radianos por segundo (double).");
    setPosition.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "set_position",
        .help = "Configura a velocidade do motor.",
        .hint = NULL,
        .func = &console_encmot_set_position,
        .argtable = &setPosition
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
};



///////////////////////////////////////////////////////////

int register_encmot_newInstance (void* handler)
{
    assert(handler != NULL);
    assert(MAX_ENC_MOT_INSTANCES > registered_instances_count);
    registered_encmot_instances[registered_instances_count] = handler;

    registered_encmot_setSpeed (handler);
    registered_encmot_setposition (handler);
    registered_encmot_tunePID (handler);

    return (registered_instances_count++);
}

