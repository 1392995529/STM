#include "luenberge.h"
#include "mc_api.h"
Observer_Handle_t  Torque_observation =
{
        .L1      = 2000,
        .L2      = -1,
        .J1      = 50000,   //1/j
        .TL      = 0,     //转矩观测值
        .K       = 1.05, //1.5*5*phi
        .hVqdLowPassFilterBW = 128
};


float Luenberge_GetTl(Observer_Handle_t * pHandle)
{
   pHandle->Iq =  MC_GetIqdMotor1().q ;
   int32_t wAux;
   int32_t lowPassFilterBW = ( int32_t ) pHandle->hVqdLowPassFilterBW - ( int32_t )1;  //低通滤波器
   wAux = ( int32_t )( pHandle->Iqav ) * lowPassFilterBW;
   wAux += pHandle->Iq;
   
   pHandle->Iqav = ( int16_t )( wAux /
                                      ( int32_t )( pHandle->hVqdLowPassFilterBW ) );

   int16_t wm = MC_GetMecSpeedAverageMotor1(); 
   float wr;
//   wr = (float)wm * 3.1415926/30 ;  //转换为角速度
   wr = (float)wm ;  //转换为角速度   
   float b ;
   pHandle->Iqavf = (float)pHandle->Iqav / 1596.5f ;   //需要进行转换
   float X = pHandle->X;      //观测到的角速度
   float TL = pHandle->TL;
   TL += (wr - X)*pHandle->L2;
   b = (pHandle->Iqavf *pHandle->K - TL )*pHandle->J1 + (wr - X)*pHandle->L1;
   X += b;
   pHandle->X = X;
   pHandle->TL = TL;
   return TL;
}

void Observer_SetL1( Observer_Handle_t * pHandle, int16_t L1 )
{
  pHandle->L1 = L1;
}

void Observer_SetL2( Observer_Handle_t * pHandle, int16_t L2 )
{
  pHandle->L2 = L2;
}
 void Observer_SetK( Observer_Handle_t * pHandle, float K )
{
  pHandle->K = K;
}