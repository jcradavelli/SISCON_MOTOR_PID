/**
 * @file encoder.h
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

#include "stdint.h"
#include "hal/pcnt_ll.h"
#include "driver/pulse_cnt.h"



/**
 * @brief Handler para o objeto encoder.
 */
typedef void* encoder_h;

/**
 * @brief Definição do tipo de função de callback para evento "on reach" gerado pelo encoder.
 * 
 */
typedef bool (*pcnt_event_on_reach_callback_h) (pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) ;

/**
 * @brief Estrutura que configura o funcionamento de encoder.
 * 
 * Essa estrutura recebe os parâmetros para a configuração do componente encoder
 * 
 * Problemas:
 *  Essa estrutura está espondo todas as variáveis de configuração possíveis, deve-se 
 * verificar quais os elementos realmente úteis para a aplicação
 *  Estruturar as configurações para que seja possivel utilizar diferentes tipos de
 * motores e encoders
 * 
 */
typedef struct encoder_config_{
    int gpio_encoder_a;         //!< Número do GPIO do encoder canal A
    int gpio_encoder_b;         //!< Número do GPIO do encoder canal B
    int max_glitch_ns;          //!< Filtro de 'glitch' em ns. Pulsos com duração menor que esse valor são ignorados
    int pcnt_high_limit;        //!< Limite máximo de contágem do hardware. Deve ser > 0. @TODO: Determinar qual o valor máximo possivel
    int pcnt_low_limit;         //!< Limite mínimo de contagem do hardware. Deve ser < 0. @TODO: Determinar qual o valor máximo possivel
    int watchpoint_inferior;    //!< Valor de contágem que gera uma chamada de callback
    int watchpoint_targett;     //!< Valor de contágem que gera uma chamada de callback
    int watchpoint_superior;    //!< Valor de contágem que gera uma chamada de callback
    pcnt_event_on_reach_callback_h example_pcnt_on_reach; //!< Endereço de função de calback ver ::pcnt_event_on_reach_callback_h
    void *example_pcnt_on_reach_user_data;  //!< Parâmetro enviado para a callback
    int pulses_per_revolution;           //!< numero de PULSOS/volta - use -1 para usar os valores em contagens
}encoder_config_t;

/**
 * @brief estrutura de dados obtidos com a leitura do encoder
 * 
 */
typedef struct encoder_sample_{
    int64_t time;
    double  count;
    double  angle;
    double  speed;
    double  acele;
}encoder_sample_t;

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
 * @param config Estrutura que configura o funcionamento de encoder.
 * @return ::encoder_h - Handler para o objeto encoderor (dinamicamente alocado)
 */
encoder_h encoder_attach (encoder_config_t config);

/**
 * @brief Obtem a contágem crua do valor do encoder
 * 
 * @param handler handler para o objeto encoder, inicializado pela função ::encoder_attach
 * @return int valor contado
 */
int encoder_get_enconderCount_raw(encoder_h handler);

/**
 * @brief Zera o contador
 * 
 * @param handler handler para o objeto encoder, inicializado pela função ::encoder_attach
 */
void encoder_clear_encoderCount (encoder_h handler);

/**
 * @brief executa a rotina do encoder, deve ser chamada regularmente com o período definido na configuração
 * 
 * @param handler handler para o objeto encoder, inicializado pela função ::encoder_attach
 */
void encoder_job (encoder_h handler);

/**
 * @brief retorna os ultimos dados amostrados pelo enconder
 * 
 * @param handler handler para o objeto encoder, inicializado pela função ::encoder_attach
 * @return encoder_sample_t ultima amostra coletada do encoder
 */
encoder_sample_t encoder_get_lastSample (encoder_h handler);



#ifdef __cplusplus
}
#endif
