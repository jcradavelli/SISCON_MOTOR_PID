#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "PIDcontroller.h"

#ifndef PC_BUILD
#include "esp_log.h"
#endif



static const char *TAG = "PID_CONTR";


/**
 * Structure that holds PID all the PID controller data, multiple instances are
 * posible using different structures for each controller
 */
typedef struct pid_ {
	// Input, output and setpoint
	pid_sample_t *sample;
	float * output; //!< Corrective Output from PID Controller
	float * setpoint; //!< Controller Setpoint
	// Tuning parameters
	float Kp; //!< Stores the gain for the Proportional term
	float Ki; //!< Stores the gain for the Integral term
	float Kd; //!< Stores the gain for the Derivative term
	// Output minimum and maximum values
	float omin; //!< Maximum value allowed at the output
	float omax; //!< Minimum value allowed at the output
	// Variables for PID algorithm
	float iterm; //!< Accumulator for integral term
	float lastin; //!< Last input value for differential term
	// Time related
	uint32_t lasttime_ms; //!< Stores the time when the control loop ran last time
	uint32_t samplerate_ms; //!< Defines the PID sample time
	// Operation mode
	uint8_t automode; //!< Defines if the PID controller is enabled or disabled
	pid_control_directions_t direction;
	bool isSatured;

}pid_controller_t;



pid_controller_t __pid_attach 
(
	pid_sample_t *sample, 
	float* out, 
	float* set, 
	float kp, 
	float ki, 
	float kd, 
	int samplerate_ms
)
{
	pid_controller_t pid = {0};

	pid.sample = sample;
	pid.output = out;
	pid.setpoint = set;
	pid.automode = false;
	pid.isSatured = false;

	pid_limits(&pid, 0, 255);

	pid.samplerate_ms = samplerate_ms;

	pid_direction(&pid, E_PID_DIRECT);
	pid_tune(&pid, kp, ki, kd);

	pid.lasttime_ms = pid.sample->time_ms;//tick_get() - handler->samplerate_ms;

	return pid;
}

pid_controller_h pid_attach (pid_config_t config)
{
	pid_controller_t aux;
    pid_controller_h object = NULL;
    
    // Aloca memoria para o objeto EcnMot
    object = malloc(sizeof(pid_controller_t));
    if (object == NULL)
    {
        ESP_LOGE(TAG,"Não foi possivel criar o objeto.");
        return(NULL);
    }

	aux = __pid_attach(
		config.sample,
		config.out,
		config.set,
		config.kp,
		config.ki,
		config.kd,
		config.samplerate_ms
	);
	memcpy(object, &aux, sizeof(pid_controller_t));

	ESP_LOGI(TAG, "PIDcontroller conectado!");
	ESP_LOGD(TAG, 
			"\n---------- PID controller config -------------\n"
			"sample->input:\t\t%f\n"
			"sample->time_ms:\t%lld\n"
			"out (adr):\t\t%f\t(%p)\n"
			"samplerate_ms:\t\t%d\n"
			"set (adr):\t\t%f\t(%p)\n"
			"kp:\t\t\t%f\n"
			"ki:\t\t\t%f\n"
			"kd:\t\t\t%f\n"
			"----------------------------------------------", 
			config.sample->input,
			config.sample->time_ms,
			*config.out, config.out,
			config.samplerate_ms,
			*config.set, config.set,
			config.kp,
			config.ki,
			config.kd
	);

    return(object);
}



bool pid_need_compute(pid_controller_h pid)
{
	pid_controller_t *handler = (pid_controller_t* )pid;
	assert(handler != NULL);

	int delta_time = handler->sample->time_ms - handler->lasttime_ms;
	bool result = (delta_time >= handler->samplerate_ms) ? true : false;


	// ESP_LOGD(
	// 	TAG,
	// 	"\n-------- need compute job ---------\n"
	// 	"sample->time_ms\t%lld\n"
	// 	"lasttime_ms\t%ld\n"
	// 	"samplerate_ms\t%ld\n"
	// 	"delta_time\t%d\n"
	// 	"result\t\t%d\n"
	// 	"-----------------------------------",
	// 	handler->sample->time_ms,
	// 	handler->lasttime_ms,
	// 	handler->samplerate_ms,
	// 	delta_time,
	// 	(int)result
	// );

	// Check if the PID period has elapsed
	return(result);
}

bool pid_IsSatured (pid_controller_h pid)
{
	pid_controller_t *handler = (pid_controller_t* )pid;
	return(handler->isSatured);
}

bool pid_compute(pid_controller_h pid)
{
	pid_controller_t *handler = (pid_controller_t* )pid;
	pid_sample_t sample;
	float error;
	float dinput;
	float out;
	assert(handler != NULL);

	// Check if control is enabled
	if (handler->automode == false)
	{
		ESP_LOGD(TAG,"PID auto mode disabled!");
		return (false);
	}

	if (pid_need_compute(pid) == false)
	{
		//O PID não necessita ser calculado ainda
		return (false);
	}
	
	// Copy sample
	sample = *handler->sample;

	// Compute error
	error = (*(handler->setpoint)) - sample.input;

	
	//Anti-windup
	// 		A integração é interrompida quando o controlador satura e 
	//      o sinal de erro tem mesmo sinal que a saída do controlador

	if (handler->isSatured == true)
	{
		//Se chegou aqui, o sinal de saida está saturado
		if ((error > 0 && *handler->output < 0) || (error < 0 && *handler->output > 0))
		{
			//Se chegou aqui, o erro e a saída tem sinais opostos
			// Compute integral
			handler->iterm += (handler->Ki * error);
		}
	}
	else
	{
		// Computa a integral normalmente se não saturado
		handler->iterm += (handler->Ki * error);
	}

	
	// Compute differential on input
	dinput = sample.input - handler->lastin;

	// Compute PID output
	out = (handler->Kp * error) + (handler->iterm) - (handler->Kd * dinput);


	// Output to pointed variable
	if ((out >= handler->omax) || (out <= handler->omin))
	{
		// Saída do controlador está saturada
		handler->isSatured = true;

		// Apply limit to output value
		if (out > handler->omax)
			(*handler->output) = handler->omax;
		else
			(*handler->output) = handler->omin;
	}
	else
	{
		handler->isSatured = false;

		// Propaga a saída do controlador para o processo
		(*handler->output) = out;
		
	}

	

	ESP_LOGD(
		TAG,
		"\n---------- PID controller job ----------------\n"
		">in:\t\t%e\n"
		">setpoint:\t%e\n"
		">error:\t\t%e\n"
		">kp: %e\n"
		">ki: %e\n"
		">kd: %e\n"
		">handler->iterm:\t%e\n"
		">dinput:\t\t%e\n"
		">out:\t\t%e\n"
		"\n----------------------------------------------\n",
		sample.input,
		*(handler->setpoint),
		error,
		handler->Kp,
		handler->Ki,
		handler->Kd,
		handler->iterm,
		dinput,
		out
	);
	
	// Keep track of some variables for next execution
	handler->lastin = sample.input;
	handler->lasttime_ms = handler->sample->time_ms;

	return (true);
}

void pid_tune(pid_controller_h pid, float kp, float ki, float kd)
{
	pid_controller_t *handler = (pid_controller_t* )pid;
	assert(handler != NULL);
	
	// Check for validity
	if (kp < 0 || ki < 0 || kd < 0)
		return;
	
	//Compute sample time in seconds
	//float ssec = ((float) handler->samplerate_ms) / ((float) 1000); //TODO: check if can use real time

	handler->Kp = kp;
	handler->Ki = ki;// * ssec;
	handler->Kd = kd;// / ssec;

}

void pid_limits(pid_controller_h pid, float min, float max)
{
	pid_controller_t *handler = (pid_controller_t* )pid;
	assert(handler != NULL);

	if (min >= max) return;
	handler->omin = min;
	handler->omax = max;
	//Adjust output to new limits
	if (handler->automode) {
		if (*(handler->output) > handler->omax)
			*(handler->output) = handler->omax;
		else if (*(handler->output) < handler->omin)
			*(handler->output) = handler->omin;

		if (handler->iterm > handler->omax)
			handler->iterm = handler->omax;
		else if (handler->iterm < handler->omin)
			handler->iterm = handler->omin;
	}
}

void pid_auto(pid_controller_h pid)
{
	pid_controller_t *handler = (pid_controller_t* )pid;
	assert(handler != NULL);

	// If going from manual to auto
	if (!handler->automode) {
		// handler->iterm = *(handler->output);
		handler->lastin = handler->sample->input;
		// if (handler->iterm > handler->omax)
		// 	handler->iterm = handler->omax;
		// else if (handler->iterm < handler->omin)
		// 	handler->iterm = handler->omin;
		handler->iterm = 0;
		handler->automode = true;
	}
}

void pid_manual(pid_controller_h pid)
{
	pid_controller_t *handler = (pid_controller_t* )pid;
	assert(handler != NULL);

	handler->automode = false;
}

void pid_direction(pid_controller_h pid, pid_control_directions_t dir)
{
	pid_controller_t *handler = (pid_controller_t* )pid;
	assert(handler != NULL);
	
	if (handler->automode && handler->direction != dir) {
		handler->Kp = (0 - handler->Kp);
		handler->Ki = (0 - handler->Ki);
		handler->Kd = (0 - handler->Kd);
	}
	handler->direction = dir;
}
