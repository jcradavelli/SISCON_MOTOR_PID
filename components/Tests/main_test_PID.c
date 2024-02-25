
/**
 * @file main_test_PID.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-02-24
 *
 * @copyright Copyright (c) 2024
 *
 */

// Includes gerais
#include <stdio.h>
#include <assert.h>
#include <locale.h>

// Includes para setup de ambiente
#include "wrappers.h"

// Includes dos fontes das bibliotecas compiladas
#include "../PIDcontroller/PIDcontroller.c"



// Modelo de um motor DC
// https://www.youtube.com/watch?v=ramy3Ucn-WU&ab_channel=Facilita%2Cprofessor%21
// https://www.scielo.org.mx/scielo.php?script=sci_arttext&pid=S1665-64232022000600718
// https://www.scilab.org/discrete-time-pid-controller-implementation
//
// Equação elétrica do motor:
// Va = L_a * i_a(t)d/dt + R_a*i_a(t) + e_m
//
// e_m = k_e*W(t)
// Va = Tensão elétrica aplicada
// i_a = Corrente de armadura (A)
// R_a = Resistência de armadura (Ohms)
// La = Indutância de armadura (H)
// e_m = F.E.M (V)
// ke = constante de força eletromotriz (V*s/rad)
// W(t) Velocidade angular em rad/s
//
//
// Acoplamento eletromecânico:
//  Tm = km*ia(t)
//
// rotor:
//  tm = j*W(t)d/dt + B*W(t) + F
//
// km = constante de torque (N*m/A)
// B = constante de atrito viscoso (N*m*s/rad)
// J = Momento de inercia (kg*m^2)
// F = atrito estático
//
//
double simulation_run (double dTime, double Va, double W, double Ia, double Theta, double *out_w, double *out_theta, double *out_ia)
{
    double dIa, dW;

    const double Ke = 0.064, Ra = 0.837, La = 0.0008, Km = 0.0065, b = -4.1210*0.0000001, F = 0.0005/1000, j =  3.87/10000000.0;
    dIa = ((Va - (Ke*W) - Ia - Ra) / La);
    dW = (((Km * Ia) - (b * W) - F )/ j);


    *out_ia     = Ia + dIa * dTime;
    *out_w      = W + dW * dTime;
    *out_theta  = Theta + *out_w * dTime;

    return(*out_theta);


}

//Simulation variables
double W = 0;
double Ia = 0;
double Theta = 0;
const double time = 0.000001;
int main (void)
{

    float pid_in = 0;
    float pid_out = 0;
    float pid_setpoint = 0;
    float kd = 0;
    float ki = 0;
    float kp = 0.7;
    int samplerate_ms = 0;
    pid_controller_h PIDcontroller;

    pid_config_t pid_config = {
        .kd                                 =   kd,
        .ki                                 =   ki,
        .kp                                 =   kp,
        .out                                =   &pid_out,
        .sample                             =   &pid_in,
        .samplerate_ms                      =   samplerate_ms,
        .set                                =   &pid_setpoint,
    };
    PIDcontroller = pid_attach(pid_config);
    pid_limits(PIDcontroller, -400, 400);
    pid_auto(PIDcontroller);
    pid_setpoint = 12;
    pid_in = 0;
    pid_out = 0;

    setlocale(LC_NUMERIC, ""); //virgulas como decial
    FILE *graph = fopen("graph_test.txt", "w");
    assert(graph!=NULL);

    fprintf(graph, "pid_setpoint\tpid_in\tkp\tki\tkd\tpid_out\tW\tIa\tTheta\n");


    double calc_W = 0;
    double calc_Ia = 0;
    double calc_Theta = 0;

    for (double i =0; i<100; i++) //Janela de 10 ms
    {
        
        fprintf(graph,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",pid_setpoint,pid_in, kp, ki, kd, pid_out, W, Ia, Theta);


        for (double j = 0; j < 0.0001; j+=0.000001) // prints de 0.1 ms
        {
            simulation_run(time, pid_out, W, Ia, Theta, &calc_W, &calc_Theta, &calc_Ia);
            W = calc_W;
            Ia = calc_Ia;
            Theta = calc_Theta;
        }
        pid_in = Theta;

        if (pid_compute(PIDcontroller));

    }

    fclose(graph);
}