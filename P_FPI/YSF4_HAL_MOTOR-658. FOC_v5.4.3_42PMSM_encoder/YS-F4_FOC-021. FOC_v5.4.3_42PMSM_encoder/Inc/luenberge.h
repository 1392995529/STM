#include "mc_type.h"


typedef struct
{
  int16_t Iqav;
  int16_t Iq;
  float  Iqavf ;
  float  TL ;
  int16_t  L1;
  int16_t  L2;
  float  J1;  //转动惯量
  float  X;    //观测到的转速
  float K ;   //K = 1.5Pnφ
  int16_t hVqdLowPassFilterBW; 
}Observer_Handle_t;

float Luenberge_GetTl(Observer_Handle_t * pHandle);
extern Observer_Handle_t  Torque_observation ;
void Observer_SetL1( Observer_Handle_t * pHandle, int16_t L1 );
void Observer_SetL2( Observer_Handle_t * pHandle, int16_t L2 );
void Observer_SetK( Observer_Handle_t * pHandle, float K );