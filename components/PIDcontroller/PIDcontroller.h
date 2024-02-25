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


#include <stdbool.h>
#include <stdint.h>

/**
 * Defines if the controler is direct or reverse
 */
typedef enum {
	E_PID_DIRECT,
	E_PID_REVERSE,
}pid_control_directions_t;



typedef struct pid_sample_{
    float input;
    int64_t time_ms;
}pid_sample_t;


typedef struct pid_config_ {
	pid_sample_t *sample;
	float* out;
	float* set; 
	float kp;
	float ki;
	float kd;
	int samplerate_ms;
}pid_config_t;

typedef void* pid_controller_h;

/**
 * @brief Creates a new PID controller
 *
 * Creates a new pid controller and initializes it�s input, output and internal
 * variables. Also we set the tuning parameters
 *
 * @param pid A pointer to a pid_controller structure

 * @param out Poiter to put the controller output value
 * @param set Pointer float with the process setpoint value
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Diferential gain
 *
 * @return returns a pid_controller_t controller handle
 */
pid_controller_h pid_attach(pid_config_t config);

/**
 * @brief Check if PID loop needs to run
 *
 * Determines if the PID control algorithm should compute a new output value,
 * if this returs true, the user should read process feedback (sensors) and
 * place the reading in the input variable, then call the pid_compute() function.
 *
 * @return return Return true if PID control algorithm is required to run
 */
bool pid_need_compute(pid_controller_h pid);

/**
 * @brief Computes the output of the PID control
 *
 * This function computes the PID output based on the parameters, setpoint and
 * current system input.
 *
 * @param pid The PID controller instance which will be used for computation
 * 
 * @return true - o controlaor rodou e existe novo parâmetro de saída
 * @return false - não houve alteração no valor da saída
 */
bool pid_compute(pid_controller_h pid);

/**
 * @brief Sets new PID tuning parameters
 *
 * Sets the gain for the Proportional (Kp), Integral (Ki) and Derivative (Kd)
 * terms.
 *
 * @param pid The PID controller instance to modify
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void pid_tune(pid_controller_h pid, float kp, float ki, float kd);

/**
 * @brief Sets the pid algorithm period
 *
 * Changes the between PID control loop computations.
 *
 * @param pid The PID controller instance to modify
 * @param time The time in milliseconds between computations
 */
void pid_sample(pid_controller_h pid, uint32_t time);

/**
 * @brief Sets the limits for the PID controller output
 *
 * @param pid The PID controller instance to modify
 * @param min The minimum output value for the PID controller
 * @param max The maximum output value for the PID controller
 */
void pid_limits(pid_controller_h pid, float min, float max);

/**
 * @brief Enables automatic control using PID
 *
 * Enables the PID control loop. If manual output adjustment is needed you can
 * disable the PID control loop using pid_manual(). This function enables PID
 * automatic control at program start or after calling pid_manual()
 *
 * @param pid The PID controller instance to enable
 */
void pid_auto(pid_controller_h pid);

/**
 * @brief Disables automatic process control
 *
 * Disables the PID control loop. User can modify the value of the output
 * variable and the controller will not overwrite it.
 *
 * @param pid The PID controller instance to disable
 */
void pid_manual(pid_controller_h pid);

/**
 * @brief Configures the PID controller direction
 *
 * Sets the direction of the PID controller. The direction is "DIRECT" when a
 * increase of the output will cause a increase on the measured value and
 * "REVERSE" when a increase on the controller output will cause a decrease on
 * the measured value.
 *
 * @param pid The PID controller instance to modify
 * @param direction The new direction of the PID controller
 */
void pid_direction(pid_controller_h pid, pid_control_directions_t dir);


bool pid_IsSatured (pid_controller_h pid);

#ifdef __cplusplus
}
#endif
