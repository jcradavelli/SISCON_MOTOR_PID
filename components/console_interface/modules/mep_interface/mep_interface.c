
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

#include "mep_interface.h"


void assert_console_interface_mep(console_interface_mep_t* interface_mep)
{
    assert(interface_mep != NULL);
    assert(interface_mep->handler != NULL);
    assert(interface_mep->setNormal != NULL);
    assert(interface_mep->release_motors != NULL);
}



static struct {
    struct arg_dbl *angle_0;
    struct arg_dbl *angle_1;
    struct arg_end *end;
} setNormal;


int __set_position (console_interface_mep_t* interface_mep, int argc, char **argv)
{
    assert_console_interface_mep(interface_mep);

    int errors = arg_parse(argc, argv, (void **) &setNormal);
    if (errors != 0) {
        arg_print_errors(stderr, setNormal.end, argv[0]);
        return 1;
    }

    if((setNormal.angle_0->count != 1) || (setNormal.angle_1->count != 1)){
        printf("Argumentos invalidos\n");
        return(0);
    }
    
    // chama a função
    interface_mep->setNormal(interface_mep->handler, *setNormal.angle_0->dval, *setNormal.angle_1->dval);



    return 0;
}



static struct {
    struct arg_end *end;
} release_motors;

int __release_motors (console_interface_mep_t* interface_mep, int argc, char **argv)
{
    assert_console_interface_mep(interface_mep);

    int errors = arg_parse(argc, argv, (void **) &release_motors);
    if (errors != 0) {
        arg_print_errors(stderr, release_motors.end, argv[0]);
        return 1;
    }
    
    // chama a função
    interface_mep->release_motors(interface_mep->handler);
    
    return 0;
}


static struct {
    struct arg_end *end;
} home_motors;

int __home_position (console_interface_mep_t* interface_mep, int argc, char **argv)
{
    assert_console_interface_mep(interface_mep);

    int errors = arg_parse(argc, argv, (void **) &release_motors);
    if (errors != 0) {
        arg_print_errors(stderr, release_motors.end, argv[0]);
        return 1;
    }
    
    // chama a função
    interface_mep->home_position(interface_mep->handler);
    
    return 0;
}





int register_mep (console_interface_mep_t* interface_mep)
{
    assert_console_interface_mep(interface_mep);
    
    setNormal.angle_0 = arg_dbl1("a", "azimute", "<dval>", "Angulo de azimute da normal (double).");
    setNormal.angle_1 = arg_dbl1("p", "polar", "<dval>", "Angulo de polar da normal (double).");
    setNormal.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "mep_setNormal",
        .help = "Posiciona o mep na posicao normal passada como argumento.",
        .hint = NULL,
        .func_w_context = &__set_position,
        .context = interface_mep,
        .argtable = &setNormal,

    };
    esp_console_cmd_register(&cmd);

    release_motors.end = arg_end(0);
    const esp_console_cmd_t cmd2 = {
        .command = "mep_release_motors",
        .help = "Libera os motores do mep.",
        .hint = NULL,
        .func_w_context = &__release_motors,
        .context = interface_mep,
        .argtable = &release_motors,

    };
    esp_console_cmd_register(&cmd2);

    home_motors.end = arg_end(0);
    const esp_console_cmd_t cmd3 = {
        .command = "mep_home_motors",
        .help = "Posiciona os motores do mep na posicao inicial.",
        .hint = NULL,
        .func_w_context = &__home_position,
        .context = interface_mep,
        .argtable = &home_motors,

    };
    esp_console_cmd_register(&cmd3);






    return(0);


}