
#include "esp_log.h"

#include "MEP_CINEMATICS.h"


#define TEST_MEP_CINEMATICS true

static const char *TAG = "MEP_CINEMATICS";

#define n(_idx_) (2*((_idx_+1)-1)*M_PI/3)

static void cross(const double a[3], const double b[3], double out_result[3]) {
    out_result[0] = a[1] * b[2] - a[2] * b[1];
    out_result[1] = a[2] * b[0] - a[0] * b[2];
    out_result[2] = a[0] * b[1] - a[1] * b[0];
}

static double dot(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * @brief Get the Vectors From Normal object
 * 
 * Fonte: Iliyas Tursynbek, Almas Shintemirov (2021)
 * 
 * @param normal 
 * @param out_v1 coordenadas do vetor v1 na sewquencia x, y e z
 * @param out_v2 coordenadas do vetor v2 na sewquencia x, y e z
 * @param out_v3 coordenadas do vetor v3 na sewquencia x, y e z 
 */
void getVectorsFromNormal(const double normal[3], double out_v1[3], double out_v2[3], double out_v3[3]) {
    // Calcular out_v1
    double cross_product[3];
    double vx;
    double vy;
    double vz;
    double angle;
    double norm_cross_product;


    cross(normal, (double[3]){0, 0, 1}, cross_product);
    norm_cross_product = sqrt(cross_product[0] * cross_product[0] + cross_product[1] * cross_product[1] + cross_product[2] * cross_product[2]);
    out_v1[0] = cross_product[0] / norm_cross_product;
    out_v1[1] = cross_product[1] / norm_cross_product;
    out_v1[2] = cross_product[2] / norm_cross_product;

    cross(normal, out_v1, cross_product);

    // Calcular out_v2
    angle = 4 * M_PI / 3;
    vx = out_v1[0] * cos(angle) + cross_product[0] * sin(angle) + normal[0] * dot(normal, out_v1) * (1 - cos(angle));
    vy = out_v1[1] * cos(angle) + cross_product[1] * sin(angle) + normal[1] * dot(normal, out_v1) * (1 - cos(angle));
    vz = out_v1[2] * cos(angle) + cross_product[2] * sin(angle) + normal[2] * dot(normal, out_v1) * (1 - cos(angle));
    out_v2[0] = vx;
    out_v2[1] = vy;
    out_v2[2] = vz;
    
    // Calcular out_v3
    angle = 2 * M_PI / 3;
    vx = out_v1[0] * cos(angle) + cross_product[0] * sin(angle) + normal[0] * dot(normal, out_v1) * (1 - cos(angle));
    vy = out_v1[1] * cos(angle) + cross_product[1] * sin(angle) + normal[1] * dot(normal, out_v1) * (1 - cos(angle));
    vz = out_v1[2] * cos(angle) + cross_product[2] * sin(angle) + normal[2] * dot(normal, out_v1) * (1 - cos(angle));
    out_v3[0] = vx;
    out_v3[1] = vy;
    out_v3[2] = vz;
}

/**
 * @brief 
 * 
 * Adaptado de:
 * Bai, Hansen e Angeles (2009)
 * Iliyas Tursynbek, Almas Shintemirov (2021)
 * Iliyas Tursynbek, Aibek Niyetkaliyev and Almas Shintemirov () - Computation of Unique Kinematic Solutions of a Spherical Parallel Manipulator with Coaxial Input Shafts
 * Yang Jian, Jorge Angeles (2012)
 * Shaoping Bai, Michael R. Hansen and Jorge Angeles () - A Robust Forward-Displacement Analysis of Spherical Parallel Robots
 * Clément M. Gosselin, Eric Lavoie (1993)
 * Aibek Niyetkaliyev, Almas Shintemirov (2014)
 * 
 * 
 * 
 * @param this 
 * @param out_theta 
 * @param Vx 
 * @param Vy 
 * @param Vz 
 * @return int 
 */
int getAnglesFromVectors (mep_cinematics_t this, double out_theta[3], double Vx[3], double Vy[3], double Vz[3])
{
    // Valores intermediarios
    double A[3], B[3], C[3];                        //<! Coeficientes obtidos pela equação 7
    double T[3];                                    //<! Raizes positivas da equação 6

                    

    // Obtenção dos coeficientes A, B e C
    for (int i = 0; i < 3; i++)
    {   
        A[i] = (-sin(n(i))*sin(this.gamma)*cos(this.alpha[0]) + sin(n(i))*cos(this.gamma)*sin(this.alpha[0])) * (- Vx[i]) + (cos(n(i))*sin(this.gamma)*cos(this.alpha[0]) - cos(n(i))*cos(this.gamma)*sin(this.alpha[0]))* Vy[i] + (cos(this.gamma)*cos(this.alpha[0])-sin(this.gamma)*sin(this.alpha[0]))*Vz[i] - cos(this.alpha[1]);
        B[i] = (cos(n(i))*sin(this.alpha[0]))*(-Vx[i]) + sin(n(i))*sin(this.alpha[0])*Vy[i];
        C[i] = (-sin(n(i))*sin(this.gamma)*cos(this.alpha[0]) - sin(n(i))*cos(this.gamma)*sin(this.alpha[0])) * (- Vx[i]) + (cos(n(i))*sin(this.gamma)*cos(this.alpha[0]) + cos(n(i))*cos(this.gamma)*sin(this.alpha[0]))* (Vy[i]) + (-cos(this.gamma)*cos(this.alpha[0])+sin(this.gamma)*sin(this.alpha[0]))*Vz[i] - cos(this.alpha[1]);

        ESP_LOGD (TAG, "A[%d] = %f", i, A[i]);
        ESP_LOGD (TAG, "B[%d] = %f", i, B[i]);
        ESP_LOGD (TAG, "C[%d] = %f", i, C[i]);
    }

    // Obtenção das raizes positivas de t[i] na equação 6
    // A[i]*t[i]^2 + 2*B[i]*t[i] + C[i] = 0
    switch (this.mounting_mode)
    {
        case MEP_CINEMATICS_MODE_RRR:
            T[0] = +1.0;
            T[1] = +1.0;
            T[2] = +1.0;
            break;
        case MEP_CINEMATICS_MODE_RRL:
            T[0] = +1.0;
            T[1] = +1.0;
            T[2] = -1.0;
            break;
        case MEP_CINEMATICS_MODE_RLR:
            T[0] = +1.0;
            T[1] = -1.0;
            T[2] = +1.0;
            break;
        case MEP_CINEMATICS_MODE_RLL:
            T[0] = +1.0;
            T[1] = -1.0;
            T[2] = -1.0;
            break;
        case MEP_CINEMATICS_MODE_LRR:
            T[0] = -1.0;
            T[1] = +1.0;
            T[2] = +1.0;
            break;
        case MEP_CINEMATICS_MODE_LRL:
            T[0] = -1.0;
            T[1] = +1.0;
            T[2] = -1.0;
            break;
        case MEP_CINEMATICS_MODE_LLR:
            T[0] = -1.0;
            T[1] = -1.0;
            T[2] = +1.0;
            break;
        case MEP_CINEMATICS_MODE_LLL:
            T[0] = -1.0;
            T[1] = -1.0;
            T[2] = -1.0;
            break;
        default:
            T[0] = -1.0;
            T[1] = -1.0;
            T[2] = -1.0;
            ESP_LOGW(TAG, "Invalid mounting mode, assuming LLL");
            break;
    }
    for (int i = 0; i < 3; i++)
    {
        // Coeficientes da equação quadratica
        double a = A[i];
        double b = 2*B[i];
        double c = C[i];

        double delta = (b*b) - (4*a*c);
        ESP_LOGD (TAG, "delta = %f", delta);
        if (delta < 0)
        {
            ESP_LOGE (TAG, "Delta negativo");
        }

        T[i] = (-b + T[i]*sqrt(delta)) / (2*a);

        ESP_LOGD (TAG, "T[%d] = %f", i, T[i]);
    }

    // Obtanção dos angulos das juntas a partir dos coeficientes T[i]
    for (int i = 0; i < 3; i++)
    {
        out_theta[i] = (atan(T[i]))*2;   
        ESP_LOGD (TAG, "theta[%d] = %f degrees", i, RAD_TO_DEG(out_theta[i]));
    }

    return(0);
}





