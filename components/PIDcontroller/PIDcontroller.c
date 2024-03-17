/**
 * @file PIDcontroller.c
 * @author 
 * @brief 
 * @version 0.1
 * @date 2024-03-04
 * 
 * @copyright Copyright (c) 2024
 * 
 * Adaptado de: https://github.com/pms67/PID
 * 
 */

#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "PIDcontroller.h"

#ifndef PC_BUILD
#include "esp_log.h"
#endif


static const char *TAG = "PID_CONTR";


typedef struct 
{
	/* Controller gains */
	double Kp;
	double Ki;
	double Kd;
	/* Derivative low-pass filter time constant */
	double tau;
	/* Output limits */
	int limMin;
	int limMax;
	/* Integrator limits */
	int limMinInt;
	int limMaxInt;
	/* Sample time (in seconds) */
	double T;
	/* Controller "memory" */
	double integrator;
	double prevError;			/* Required for integrator */
	double differentiator;
	double prevMeasurement;		/* Required for differentiator */
	double oldSetpoint;
	/* Controller output */
	int out;
} PIDController_t;


PIDController_h PIDController_create(void){
	PIDController_h this = malloc (sizeof(PIDController_t));

	if (this != NULL)
		memset(this, 0, sizeof(PIDController_t));
	else
		ESP_LOGE(TAG, "Não foi possivel criar o objeto PIDController_t");

	return (this);
}

int PIDController_delete (PIDController_h pid)
{
	if (pid != NULL)
	{
		free(pid);
		pid = NULL;
		return(0);
	}

	ESP_LOGE(TAG, "Não foi possivel deletar o objeto NULL");
	return(1);
}


void PIDController_Init
(
	PIDController_h pid,
	double Kp,
	double Ki,
	double Kd,
	double tau,
	int limMin,
	int limMax,
	int limMinInt,
	int limMaxInt,
	double T
) 
{
	// asserts
	assert(pid != 0); // PID deve ser um endereço válido

	// Variáveis internas
	PIDController_t *this = pid;

	// Inicializa parâmetros de usuário
	this->Kp		=	Kp;
	this->Ki		=	Ki;
	this->Kd		=	Kd;
	this->tau		=	tau;
	this->limMin	=	limMin;
	this->limMax	=	limMax;
	this->limMinInt	=	limMinInt;
	this->limMaxInt	=	limMaxInt;
	this->T			=	T;

	/* Clear controller variables */
	this->integrator = 0.0f;
	this->prevError  = 0.0f;

	this->differentiator  = 0.0f;
	this->prevMeasurement = 0.0f;

	this->out = 0.0f;

}

double proportional;
double error;
double PIDController_Update(PIDController_h pid, double setpoint, double measurement, PIDControllerDebugStream_t *debugOut) 
{
	// asserts
	assert(pid != 0); // PID deve ser um endereço válido

	// Variáveis internas
	PIDController_t *this = pid;

	if(this->oldSetpoint != setpoint)
	{
		this->integrator = 0; // Reseta o integrador ao modificar o setpoint
		this->oldSetpoint = setpoint;
	}

	/*
	* Error signal
	*/
    error = setpoint - measurement;

	/*
	* Proportional
	*/
    proportional = this->Kp * error;

	/*
	* Integral
	*/
    this->integrator = this->integrator +  0.5f * this->Ki * this->T * (error + this->prevError);

	/* Anti-wind-up via integrator clamping */
    if (this->integrator > this->limMaxInt) {
        this->integrator = this->limMaxInt;
    } else if (this->integrator < this->limMinInt) {
        this->integrator = this->limMinInt;
    }

	/*
	* Derivative (band-limited differentiator)
	*/
    this->differentiator = -(2.0f * this->Kd * (measurement - this->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * this->tau - this->T) * this->differentiator)
                        / (2.0f * this->tau + this->T);

	/*
	* Compute output and apply limits
	*/
    this->out = proportional + this->integrator + this->differentiator;
    if (this->out > this->limMax) {
        this->out = this->limMax;
    } else if (this->out < this->limMin) {
        this->out = this->limMin;
    }

	/* Store error and measurement for later use */
    this->prevError       = error;
    this->prevMeasurement = measurement;

	/* Debug */
	if (debugOut != NULL)
	{
		debugOut->measurement		=	measurement;
		debugOut->setpoint			=	setpoint;
		debugOut->error				=	error;
		debugOut->Kp				=	this->Kp;
		debugOut->Ki				=	this->Ki;
		debugOut->Kd				=	this->Kd;
		debugOut->proportional		=	proportional;
		debugOut->integrator		=	this->integrator;
		debugOut->differentiator	=	this->differentiator;
		debugOut->out				=	this->out;
	}
	// ESP_LOGD(
	// 	TAG,
	// 	"\n---------- PID controller job ----------------\n"
	// 	">measurement:\t\t%e\n"
	// 	">setpoint:\t%e\n"
	// 	">error:\t\t%e\n"
	// 	">kp: %e\n"
	// 	">ki: %e\n"
	// 	">kd: %e\n"
	// 	">proportional:\t%e\n"
	// 	">integrator:\t\t%e\n"
	// 	">differentiator:\t\t%e\n"
	// 	">out:\t\t%d\n"
	// 	"\n----------------------------------------------\n",
	// 	measurement,
	// 	setpoint,
	// 	error,
	// 	this->Kp,
	// 	this->Ki,
	// 	this->Kd,
	// 	proportional,
	// 	this->integrator,
	// 	this->differentiator,
	// 	this->out
	// );


	/* Return controller output */
    return this->out;

}



void PIDController_tune_pid (PIDController_h pid, double kp, double ki, double kd)
{
	// asserts
	assert(pid != 0); // PID deve ser um endereço válido

	// Variáveis internas
	PIDController_t *this = pid;

	PIDController_Init(pid, kp, ki, kd, this->tau, this->limMin, this->limMax, this->limMinInt, this->limMaxInt, this->T);
}

void PIDController_reset (PIDController_h pid)
{
	// asserts
	assert(pid != 0); // PID deve ser um endereço válido

	// Variáveis internas
	PIDController_t *this 	= pid;

	/* Clear controller variables */
	this->integrator 		= 0.0f;
	this->prevError  		= 0.0f;
	this->differentiator  	= 0.0f;
	this->prevMeasurement 	= 0.0f;

	this->out 				= 0.0f;
}
