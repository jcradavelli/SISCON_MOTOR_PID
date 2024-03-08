/**
 * @file PIDcontroller.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


typedef void* PIDController_h;


/**
 * @brief Construtor do objeto controlador PID
 * 
 * @param pid 
 */
PIDController_h PIDController_create(void);

/**
 * @brief Destructor do objeto PID
 * 
 * @param pid 
 */
int PIDController_delete(PIDController_h pid);

/**
 * @brief inicializa o objeto PID
 * 
 * @param pid 	handler do objeto pid
 * @param Kp 	ganho proporcional
 * @param Ki 	ganho integrativo
 * @param Kd 	ganho derivativo
 * @param tau 		constante de filtro derivativo
 * @param limMin 	limite de saida mínimo
 * @param limMax 	limite de saída máximo
 * @param limMinInt limite de integrador mínimo (anti-windup)
 * @param limMaxInt limite de integrador máximo (anti-windup)
 * @param T período de execução da função update
 */
void PIDController_Init (PIDController_h pid, double Kp, double Ki, double Kd, double tau, 
						int limMin, int limMax, int limMinInt, int limMaxInt, double T);

/**
 * @brief Executa o algorítmo de controle uma vez, essa função deve sewr chamada recorrentemente
 * 
 * @param pid handler do objeto pid
 * @param setpoint Valor a ser atingido
 * @param measurement Valor medido (realimentado)
 * @return double saída do bloco de controle
 */
double PIDController_Update(PIDController_h pid, double setpoint, double measurement);


/**
 * @brief Ajusta os ganhos de PID
 * 
 * @param pid handler do objeto pid
 * @param Kp 	ganho proporcional
 * @param Ki 	ganho integrativo
 * @param Kd 	ganho derivativo
 */
void PIDController_tune_pid (PIDController_h pid, double kp, double ki, double kd);


#ifdef __cplusplus
}
#endif
