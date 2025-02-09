
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
}



static struct {
    struct arg_dbl *angle_0;
    struct arg_dbl *angle_1;
    struct arg_dbl *angle_2;
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

    if((setNormal.angle_0->count != 1) || (setNormal.angle_1->count != 1) || (setNormal.angle_2->count != 1)){
        printf("Argumentos invalidos\n");
        return(0);
    }
    
    // TODO: chama a função
    double angles[3] = {*setNormal.angle_0->dval, *setNormal.angle_1->dval, *setNormal.angle_2->dval};
    interface_mep->setNormal(interface_mep->handler, angles);



    return 0;
}





int register_mep (console_interface_mep_t* interface_mep)
{
    assert_console_interface_mep(interface_mep);
    
    setNormal.angle_0 = arg_dbl1("x", "x", "<dval>", "Componente X do vetor normal (double).");
    setNormal.angle_1 = arg_dbl1("y", "y", "<dval>", "Componente Y do vetor normal (double).");
    setNormal.angle_2 = arg_dbl1("z", "z", "<dval>", "Componente Z do vetor normal (double).");
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


    return(0);


}