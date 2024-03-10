
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
#include "../PIDcontroller/PIDcontroller.h"








// ===========================================================================
// Controller parameters
// ===========================================================================
    // #define PID_KP  1.0f // Ganho crítico 3
    // #define PID_KI  0.0f
    // #define PID_KD  0.000f
// Ganho obtidos pelo segundo metodo de ZIEGLER-NICHOLS para controle de posição
//  Aparentemente existe um erro de offset a ser eliminado
    #define PID_CONTROL_VARIABLE *out_theta
    #define PID_KP  1.8f
    #define PID_KI  40.0f//0.025f
    #define PID_KD  0.00625f
// #define PID_CONTROL_VARIABLE    *out_w
// #define PID_KP                  0.6f*0.04f
// #define PID_KI                  1.0f/(0.5f*0.02032f)
// #define PID_KD                  0.125f*0.02032f

#define PID_TAU                 0.02f
#define PID_LIM_MIN             -12.0f
#define PID_LIM_MAX             12.0f
#define PID_LIM_MIN_INT         -5.0f
#define PID_LIM_MAX_INT         5.0f
#define SAMPLE_TIME_S           0.01f





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

    const double Ke = /*0.064*/0.01326, Ra = /*0.837*/11.3, La = /*0.0008*/0.0010, Km = /*0.0065*/0.27785, b = 4.1210*0.0001, F = 0.0005/100000, j =  3.87/100000.0;

    //TODO: Discretizar a variável de entrada Va - simular o PWM
    //TODO: Limitar a corrente máxima (reduzindo Va)

    dIa = ((Va - (Ke*W) - Ia - Ra) / La);
    dW = (((Km * Ia) - (b * W) - F )/ j);


    *out_ia     = Ia + dIa * dTime;
    *out_w      = W + dW * dTime;
    *out_theta  = Theta + *out_w * dTime;

    //TODO: Discretizar a variável de controle
    return(PID_CONTROL_VARIABLE);
}






//Simulation variables
double W = 0;
double Ia = 0;
double Theta = 0;
double setpoint = 0;
double measure = 0;
double pid_out = 0;
const double tick_simulation = 0.000001;
int main (void)
{
    PIDController_h myPID = PIDController_create();
    PIDController_t *dut = myPID;
    assert(myPID != NULL);

    PIDController_Init(
        myPID,
        PID_KP,
        PID_KI,
        PID_KD,
        PID_TAU,
        PID_LIM_MIN,
        PID_LIM_MAX,
        PID_LIM_MIN_INT,
        PID_LIM_MAX_INT,
        SAMPLE_TIME_S
    );


    // Cria o arquivo de saída da simulação
    setlocale(LC_NUMERIC, ""); //virgulas como decial
    FILE *graph = fopen("graph_test.txt", "w");
    assert(graph!=NULL);

    /* Simulate response using test system */
    fprintf(graph, "setpoint\tpid_in\tkp\tki\tkd\tpid_out\tW\tIa\tTheta\n");

    double calc_W = 0;
    double calc_Ia = 0;
    double calc_Theta = 0;

    for (double i =0; i<1000; i+=10) //Janela de 10000 ms
    {
        
        fprintf(graph,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",setpoint,measure, dut->Kp, dut->Ki, dut->Kd, dut->out, W, Ia, Theta);
        for (double j = 0; j < 0.01; j+=0.000001) // integração de 1us, prints e PID de 10 ms
        {
            measure = simulation_run(tick_simulation, pid_out, W, Ia, Theta, &calc_W, &calc_Theta, &calc_Ia);
            W = calc_W;
            Ia = calc_Ia;
            Theta = calc_Theta;
        }


        pid_out = PIDController_Update(myPID, setpoint, measure);

    }

    fclose(graph);
}