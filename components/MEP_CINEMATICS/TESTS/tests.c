
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdbool.h>


#include "esp_log.h"

#include "../MEP_CINEMATICS.h"

const char *TAG = "TESTS_MEP_CINEMATICS";

int testCounter = 0;


typedef struct test_inverseKinematics_args_
{
    const char* test_title;
    mep_cinematics_t this;
    double expected_theta[3];
    double Vx[3];
    double Vy[3];
    double Vz[3];
}test_inverseKinematics_args_t;


typedef struct test_getVectorsFromNormal_{
    const char* test_title;
    double normal[3];
    double v1_esperado[3];
    double v2_esperado[3];
    double v3_esperado[3];
} test_getVectorsFromNormal_t;









bool test_inverseCinematics(const char* test_title, mep_cinematics_t this, double expected_theta[3], double Vx[3], double Vy[3], double Vz[3])
{
    double theta[3];
    const double MAX_ERROR_ABS = 0.01;

    ESP_LOGD(TAG, "Running %s", test_title);
    
    getAnglesFromVectors(this, theta, Vx, Vy, Vz);

    // check error
    for(int i = 0; i<3; i++)
    {
        theta[i] = fabs(theta[i] - expected_theta[i]);
        ESP_LOGD(TAG, "theta[%d] error = %0.5f", i, theta[i]);
    }

    // return result
    return(theta[0] < MAX_ERROR_ABS && theta[1] < MAX_ERROR_ABS && theta[2] < MAX_ERROR_ABS);
}


bool test_getVectorsFromNormal(const char* test_title, double normal[3], double v1_esperado[3], double v2_esperado[3], double v3_esperado[3])
{
    double v1[3], v2[3], v3[3];
    const double MAX_ERROR_ABS = 0.001;

    ESP_LOGD(TAG, "Running %s", test_title);

    getVectorsFromNormal(normal, v1, v2, v3);

    double erro_v1 = sqrt(pow(v1[0] - v1_esperado[0], 2) + pow(v1[1] - v1_esperado[1], 2) + pow(v1[2] - v1_esperado[2], 2));
    double erro_v2 = sqrt(pow(v2[0] - v2_esperado[0], 2) + pow(v2[1] - v2_esperado[1], 2) + pow(v2[2] - v2_esperado[2], 2));
    double erro_v3 = sqrt(pow(v3[0] - v3_esperado[0], 2) + pow(v3[1] - v3_esperado[1], 2) + pow(v3[2] - v3_esperado[2], 2));


    if (erro_v1 > MAX_ERROR_ABS || erro_v2 > MAX_ERROR_ABS || erro_v3 > MAX_ERROR_ABS) {
        ESP_LOGE("TESTE", "Normal: (%f, %f, %f)", normal[0], normal[1], normal[2]);
        ESP_LOGE("TESTE", "v1 esperado: (%f, %f, %f)", v1_esperado[0], v1_esperado[1], v1_esperado[2]);
        ESP_LOGE("TESTE", "v1 obtido: (%f, %f, %f)", v1[0], v1[1], v1[2]);
        ESP_LOGE("TESTE", "v2 esperado: (%f, %f, %f)", v2_esperado[0], v2_esperado[1], v2_esperado[2]);
        ESP_LOGE("TESTE", "v2 obtido: (%f, %f, %f)", v2[0], v2[1], v2[2]);
        ESP_LOGE("TESTE", "v3 esperado: (%f, %f, %f)", v3_esperado[0], v3_esperado[1], v3_esperado[2]);
        ESP_LOGE("TESTE", "v3 obtido: (%f, %f, %f)", v3[0], v3[1], v3[2]);

        return false;
    } else {
        return true;
    }
}


int main(char* argc, char** argv)
{

    test_inverseKinematics_args_t args[] =
    {
        {
            .test_title = "Teste com parametros construtivos do prototipo",
            .this = {
                .alpha          = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
                .gamma          = DEG_TO_RAD(0),
                .mounting_mode  = MEP_CINEMATICS_MODE_LLL,
            },
            .Vx = {1.0000e+00f, -5.0000e-01f, -5.0000e-01f},
            .Vy = {9.6692e-08f, -8.1213e-01f, 8.1213e-01f},
            .Vz = {9.6692e-08f, 3.0074e-01f, -3.0074e-01f},
            .expected_theta =  {DEG_TO_RAD(0), DEG_TO_RAD(-18.96945), DEG_TO_RAD(18.96945)}
        },
        {
            .test_title = "Teste com parametros de Bai, Hansen, Angeles (2009)",
            .this = {
                .alpha          = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
                .gamma          = DEG_TO_RAD(0),
                .mounting_mode  = MEP_CINEMATICS_MODE_LLL,
            },
            .Vx = {1.0000e+00f, -5.0000e-01f, -5.0000e-01f},
            .Vy = {9.6692e-08f, -8.1213e-01f, 8.1213e-01f},
            .Vz = {9.6692e-08f, 3.0074e-01f, -3.0074e-01f},
            .expected_theta =  {DEG_TO_RAD(0), DEG_TO_RAD(-18.96945), DEG_TO_RAD(18.96945)}
        },
        {
        .test_title = "Teste com parametros do artigo de Bai, Hansen e Angeles (2009) - displacement no 1",
        .this = {
            .alpha          = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
            .gamma          = DEG_TO_RAD(0),
            .mounting_mode  = MEP_CINEMATICS_MODE_RLR,
        },
        .Vx = {-0.6494,    0.28874,    0.76948,    },
        .Vy = {-0.5377,    -0.73259,   -0.06107,   },
        .Vz = {-0.5377,    0.61635,    -0.63575,   },
        .expected_theta =  {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)}
        },
        {
        .test_title = "Teste com parametros do artigo de Bai, Hansen e Angeles (2009) - displacement no 2",
        .this = {
            .alpha          = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
            .gamma          = DEG_TO_RAD(0),
            .mounting_mode  = MEP_CINEMATICS_MODE_RRR,
        },
        .Vx = {-0.70711,   0.78656,    -0.079461,  },
        .Vy = {0.5,        0.36233,    -0.86235,   },
        .Vz = {0.5,        0.50003,    0.49999,    },
        .expected_theta =  {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)}
        },
        {
        .test_title = "Teste com parametros do artigo de Bai, Hansen e Angeles (2009) - displacement no 3",
        .this = {
            .alpha          = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
            .gamma          = DEG_TO_RAD(0),
            .mounting_mode  = MEP_CINEMATICS_MODE_RLL,
        },
        .Vx = {-0.4901,    -0.79038,   0.33185,    },
        .Vy = {0.61635,    -0.29356,   0.69686,    },
        .Vz = {0.61635,    -0.53772,   -0.6358,    },
        .expected_theta =  {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)}
        },
        {
        .test_title = "Teste com parametros do artigo de Bai, Hansen e Angeles (2009) - displacement no 4",
        .this = {
            .alpha          = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
            .gamma          = DEG_TO_RAD(0),
            .mounting_mode  = MEP_CINEMATICS_MODE_LRR,
        },
        .Vx = {0.4901,     -0.33183,   0.79038,    },
        .Vy = {0.61635,    0.69686,    -0.29356,   },
        .Vz = {0.61635,    -0.6358,    -0.53772,   },
        .expected_theta =  {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)}
        },
        {
        .test_title = "Teste com parametros do artigo de Bai, Hansen e Angeles (2009) - displacement no 5",
        .this = {
            .alpha          = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
            .gamma          = DEG_TO_RAD(0),
            .mounting_mode  = MEP_CINEMATICS_MODE_RRL,
        },
        .Vx = {-0.43757,   -0.14094,   -0.77886,   },
        .Vy = {-0.6358,    0.83129,    0.11627,    },
        .Vz = {-0.6358,    -0.53769,   0.61633,    },
        .expected_theta =  {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)}
        },
        {
        .test_title = "Teste com parametros do artigo de Bai, Hansen e Angeles (2009) - displacement no 6",
        .this = {
            .alpha          = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
            .gamma          = DEG_TO_RAD(0),
            .mounting_mode  = MEP_CINEMATICS_MODE_LRL,
        },
        .Vx = {0.43758,    0.77885,    0.1409,     },
        .Vy = {-0.6358,    0.11627,    0.83127,    },
        .Vz = {-0.6358,    0.61633,    -0.53771,   },  
        .expected_theta =  {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)}
        },
        {
        .test_title = "Teste com parametros do artigo de Bai, Hansen e Angeles (2009) - displacement no 7",
        .this = {
            .alpha          = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
            .gamma          = DEG_TO_RAD(0),
            .mounting_mode  = MEP_CINEMATICS_MODE_LLR,
        },
        .Vx = {0.64949,    -0.76941,   -0.28873,   },
        .Vy = {-0.53765,   -0.06112,   -0.73261,   },
        .Vz = {-0.53765,   -0.6358,    0.61633,    },
        .expected_theta =  {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)}
        },
        {
        .test_title = "Teste com parametros do artigo de Bai, Hansen e Angeles (2009) - displacement no 8",
        .this = {
            .alpha          = {DEG_TO_RAD(45), DEG_TO_RAD(90)},
            .gamma          = DEG_TO_RAD(0),
            .mounting_mode  = MEP_CINEMATICS_MODE_LLL,
        },
        .Vx = {0.7071,     0.079402,   -0.78658,   },
        .Vy = {0.5,        -0.86236,   0.36239,    },
        .Vz = {0.5,        0.50008,    0.49997,    },
        .expected_theta =  {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)}
        }        
    };

    for (int i=0; i<sizeof(args)/sizeof(args[0]); i++)
    {
        bool result;
        ESP_LOGI(TAG, "Starting TESTE %d", ++testCounter);
        
        result = test_inverseCinematics(args[i].test_title, args[i].this,args[i].expected_theta,args[i].Vx,args[i].Vy,args[i].Vz); 

        if (result) 
        {
            ESP_LOGI(TAG, "TESTE %d result SUCESS", testCounter);
        }  
        else
        {
            ESP_LOGE(TAG, "TESTE %d result FAILL", testCounter);
        }

    }

    test_getVectorsFromNormal_t testes[] = {
        {
            .test_title = "Teste com parametros do artigo de Tursynbek, Shintemirov (2021)",
            .normal = {-0.274, -0.555, 0.786},
            .v1_esperado = {-0.8967, 0.4427, 0.0},
            .v2_esperado = {0.1470, -0.8317, -0.5360},
            .v3_esperado = {0.7497, 0.3890, 0.5360}
        },
        // Adicione mais testes aqui...
    };

    for (int i = 0; i < sizeof(testes) / sizeof(testes[0]); i++) {

        ESP_LOGI(TAG, "Starting TESTE %d", ++testCounter);
        bool result;

        result = test_getVectorsFromNormal(testes[i].test_title, testes[i].normal, testes[i].v1_esperado, testes[i].v2_esperado, testes[i].v3_esperado);


        if (result) 
        {
            ESP_LOGI(TAG, "TESTE %d result SUCESS", testCounter);
        }  
        else
        {
            ESP_LOGE(TAG, "TESTE %d result FAILL", testCounter);
        }
    }





    return 0;
}