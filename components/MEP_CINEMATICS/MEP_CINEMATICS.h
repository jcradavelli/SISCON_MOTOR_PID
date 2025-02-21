
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif  

#define DEG_TO_RAD(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define RAD_TO_DEG(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

typedef enum mep_mode_ {
    MEP_CINEMATICS_MODE_RRR,
    MEP_CINEMATICS_MODE_RRL,
    MEP_CINEMATICS_MODE_RLR,
    MEP_CINEMATICS_MODE_RLL,
    MEP_CINEMATICS_MODE_LRR,
    MEP_CINEMATICS_MODE_LRL,
    MEP_CINEMATICS_MODE_LLR,
    MEP_CINEMATICS_MODE_LLL,    
}mep_mode_t;

typedef struct mep_cinematics_
{
    double alpha[2];            //<! Angulo das juntas
    double gamma;               //<! Angulo das arestas ao centro da plataforma
    mep_mode_t mounting_mode;    //<! Modo de montagem - são possiveis 8 modos de montagem a depender do sentido de montagem dos distal links
}mep_cinematics_t;


int getAnglesFromVectors (mep_cinematics_t this, double out_theta[3], double Vx[3], double Vy[3], double Vz[3]);
void getVectorsFromNormal(const double normal[3], double out_v1[3], double out_v2[3], double out_v3[3]);