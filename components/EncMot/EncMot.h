/**
 * @file EncMot.h
 * 
 * @author Júlio César Radavelli (julio.radavelli@tecnoflex-rs.com.br or jc.radavelli@hotmail.com)
 * 
 * @brief Controla motor e mede sua posição através de encoder.
 * Essa biblioteca não implementa controles do tipo PID apenas acionamento e desacionamento.
 * 
 * @version 0.1
 * @date 2023-12-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "Encoder.h"
#include "motor.h"
#include "PIDcontroller.h"


/**
 * @brief Handler para o objeto encmot.
 */
typedef void* encmot_h;

/**
 * @brief Definição do tipo de função de callback para evento "on reach" gerado pelo encoder.
 * 
 */
typedef bool (*pcnt_event_on_reach_callback_h) (pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) ;

/**
 * @brief Estrutura que configura o funcionamento de EncMot.
 * 
 * Essa estrutura recebe os parâmetros para a configuração do componente EncMot
 * 
 * Problemas:
 *  Essa estrutura está espondo todas as variáveis de configuração possíveis, deve-se 
 * verificar quais os elementos realmente úteis para a aplicação
 *  Estruturar as configurações para que seja possivel utilizar diferentes tipos de
 * motores e encoders
 * 
 */
typedef struct encmot_config_{

    /**
     * @brief Configurações do encoder definidas pela aplicação
     * 
     */
    struct {
        int gpio_encoder_a;                                     //!< Número do GPIO do encoder canal A
        int gpio_encoder_b;                                     //!< Número do GPIO do encoder canal B
        int max_glitch_ns;                                      //!< Filtro de 'glitch' em ns. Pulsos com duração menor que esse valor são ignorados
        int gear_ratio_numerator;                               //!< numerador da fração ratio (PULSOS/voltas)
        int gear_ration_denominator;                            //!< Denominador da fração ration (pulsos/VOLTAS)
    }encoder_config;

    /**
     * @brief Configurações do motor definidas pela aplicação
     * 
     */
    struct{
        int motor_control_a;
        int motor_control_b;
    }motor_config;

    /**
     * @brief Configurações do controlador PID definidas pela aplicação
     * 
     */
    struct{
        float kp;
        float ki;
        float kd;
        int samplerate_ms;
        bool* isSatured;
    }pid_config;

}encmot_config_t;

/**
 * @brief Função que conecta o motor ao sistema.
 * 
 * Modo Motor DC com encoder de quadratura:
 *  Único modo suportado até o momento. O encoder é lido utilizando o periférico PCNT, que
 * realiza as contagens por hardware. Implementa rotinas de compensação para acumular
 * as contagens na eventualidade de um overflow. São geradas interrupções apenas ao atingirem
 * valores de contágens pré-definidos. O motor é controlado por dois sinais do tipo A e B. 
 * Para o controle implementado A = B mantém o motor em repouso, A != B coloca o motor em
 * rotação horária ou anti horária, dependendo da combinação.
 * 
 * Compatibilidade:
 *  Suporta atualmente apenas motor DC controlado por ponte H integrada como o
 * circuito integrado L9110S e com encoder incremental de quadratura.
 * 
 * Problemas:
 *  Sabe-se que o algorítmo implementado possui instabilidade se o controlador for
 * colocado em modo sleep. Ver <a href="https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html#power-management">documentação espressif</a> 
 * 
 * @param config Estrutura que configura o funcionamento de EncMot.
 * @return ::encmot_h - Handler para o objeto EncMotor (dinamicamente alocado)
 */
encmot_h encmot_attach (encmot_config_t config);

/**
 * @brief Obtem a contágem crua do valor do encoder
 * 
 * @param handler handler para o objeto encmot, inicializado pela função ::encmot_attach
 * @return int valor contado
 */
int encmot_get_enconderCount_raw(encmot_h handler);

/**
 * @brief executa a rotina do encomt, deve ser chamada regularmente com o período definido na configuração
 * 
 * @param handler 
 */
void encmot_job (encmot_h handler);

/**
 * @brief configura o setpoint a ser atingido
 * TODO: Criar uma função específica para cada tipo de controle a ser utilizado
 * 
 * @param handler handler para o objeto encmot, inicializado pela função ::encmot_attach
 * @param setpoint valor a ser perseguido pelo motor
 */
void encmot_set_speed (encmot_h handler, double setpoint);

/**
 * @brief ajusta os parâmetros de PID do controlador do motor
 * 
 * @param handler handler para o objeto encmot, inicializado pela função ::encmot_attach
 * @param kp ganho proporcional
 * @param ki ganho itegrativo
 * @param kd ganho derivativo
 */
void encmot_tune_pid (encmot_h handler, float kp, float ki, float kd);


// TODO: Funções para expandir a biblioteca
// float encmot_get_encoderPosition_grad(encmot_h handler);
// float encmot_get_encoderPosition_rad(encmot_h handler);
// void encmot_stop (encmot_h handler);
// void encmot_continue (encmot_h handler);

#ifdef __cplusplus
}
#endif
