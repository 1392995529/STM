/**
  ******************************************************************************
  * @file    mc_position.c
  * @author  Ken An
  * @brief   This file provides firmware functions for position control
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_position.h"
#include "speed_pos_fdbk.h"
#include "mc_tasks.h"
#include "fuzzypi.h"

extern int16_t DB_data[8];
extern FeedForward_Handle_t *FFPosition;
extern Position_Handle_t *pPIDPosition;


/* Define and initial positi0n structure */
Position_Handle_t Position_M1 =   //位置控制句柄
{
        .hTargetAngle            = 0,
        .Position_KpGain         = 8,
	.Postiion_KpDiv          = 2048,   
        .Position_KiGain         = 2,   //以下参数暂未使用
	.Postiion_KiDiv          = 2048,
        .Postiion_KeGain         = 0.2,
        .Postiion_KecGain        = 0.02, 
        .Postiion_KupGain        = 35,
	.Postiion_KuiGain        = 1,
	/* Max 1000ms/(3000RPM*10/60) = 2*/
	.Speed_ACC_Gain        = 1,
	.Speed_ACC_Div         = 3,
	
	.Speed_ACC_Count       = 0,
	.Encoder_Update_Count  = 0,
	.Encoder_Direction     = 0,
	.Encoder_Pre_Direction = 0,
	.Mode_Flag             = P_SPEED_MODE,
	.Torque_First_Flag     = 0,
};
FeedForward_Handle_t FF_M1 =   //前馈控制句柄
{
        .POSAngle          = 0,
        .lastPOSAngle      = 0,
        .perrPOSAngle      = 0,
        .Kvff_Gain     = 0,
        .Kaff_Gain    = 0,
  
};


/**
  * @brief  PI / PID position parameters Motor 1
  */
#define PID_ANGLE_KP_DEFAULT 3000
#define AG_KPDIV 2048
#define AG_KPDIV_LOG 11

#define PID_ANGLE_KI_DEFAULT 2
#define AG_KIDIV 16384
#define AG_KIDIV_LOG 14

#define PID_ANGLE_KD_DEFAULT 0
#define AG_KDDIV 16384
#define AG_KDDIV_LOG 14

/* Define Position Iq position max = 30% IQMAX, in this demo IQMAX = 16000*/
#define IQ_POSITION 4000

PID_Handle_t PIDAngleHandle_M1 =   //位置环PID句柄
{
  .hDefKpGain          = (int16_t)PID_ANGLE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_ANGLE_KI_DEFAULT, 
  .wUpperIntegralLimit = (int32_t)IQ_POSITION * (int32_t)AG_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQ_POSITION * (int32_t)AG_KIDIV,
  .hUpperOutputLimit   = (int16_t)IQ_POSITION, 
  .hLowerOutputLimit   = -(int16_t)IQ_POSITION,
  .hKpDivisor          = (uint16_t)AG_KPDIV,
  .hKiDivisor          = (uint16_t)AG_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)AG_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)AG_KIDIV_LOG,
  .hDefKdGain          = (int16_t)PID_ANGLE_KD_DEFAULT,
  .hKdDivisor          = (uint16_t)AG_KDDIV,
  .hKdDivisorPOW2      = (uint16_t)AG_KDDIV_LOG,
};

/**
  * @brief  It return absolute value of input
  * @param  dat: input data
  * @retval absolute value
  */
uint32_t Abs_Value(int32_t dat)  //取绝对值
{
	uint32_t temp;
	if(dat < 0)
	{
		temp = -dat;
	}
	else
	{
		temp = dat;
	}
	return temp;
}

/**
  * @brief  Return error angle unit in rad*10000
  * @param  angle_ref: Target angle
  * @retval error angle
  */
int32_t Position_GetErrorAngle(void)  // 位置角度误差
{
     	
//        Position_M1.hTargetAngle = (int32_t)angle_ref;
	
	/* Measure angle = count*2PI*10000 + Mec_Angle*2PI*10000/65536 
	*/
     
	Position_M1.hMeasuredAngle = Position_GetMecAngle();   //机械角度	
	
	/* Compute angle error */
        Position_M1.hError_Angle = Position_M1.hTargetAngle - Position_M1.hMeasuredAngle;
	
	/* If enter target range of position, then change to Torque mode */
//	if(Abs_Value(Position_M1.hError_Angle) < CHANGE_LIMIT_LOW)
//	{
//		Position_M1.Mode_Flag = P_TORQUE_MODE;
//	}
//	/* Else if motor run in speed mode */
//	else if(Abs_Value(Position_M1.hError_Angle) > CHANGE_LIMIT_HIGH)
//	{
//		Position_M1.Mode_Flag = P_SPEED_MODE;
//		Position_M1.Torque_First_Flag = 0;
//	}
	
	return  Position_M1.hError_Angle;
}

///////////////////PSD电压信号作为位置环误差输入////////////////////

float Position_GetErrorVoltage(void)  // psd电压误差
{
        float a,b;
        Position_M1.hLastError_Angle = Position_M1.hError_Angle;   //前一拍误差
        a = DB_data[0]*10000/32768;
        b = -a;
        Position_M1.hError_Angle = (int32_t)(8*b);        
        Position_M1.hMeasuredAngle = Position_GetMecAngle();	
        return a;
}

/**
  * @brief  It computes the new values for max speed
  * @param  None
  * @retval Speed referrence
  */
int16_t Position_CalcSpeedReferrence(void)
{
	int32_t hSpeedReference_PI = 0; //PI计算得到速度
	int32_t hSpeedReference_FF = 0; //位置前馈控制计算所得速度
        int32_t hSpeedReference=0;
//	hSpeedReference_PI = Position_M1.hError_Angle * Position_M1.Position_KpGain / Position_M1.Postiion_KpDiv;  //计算转速参考值
//	hSpeedReference_PI = Position_FuzzyPI(pPIDPosition);
        hSpeedReference_PI = Position_CompoundFuzzy(pPIDPosition);
        hSpeedReference_FF = FeedforwardController(&FF_M1);
        hSpeedReference = hSpeedReference_FF+hSpeedReference_PI;
        Position_M1.Encoder_Direction = hSpeedReference_PI;
        Position_M1.Encoder_Pre_Direction = hSpeedReference_FF;

	/* Limit max referrence speed*/
	if(hSpeedReference > MOTOR_MAX_SPEED_RPM/60)
	{
		hSpeedReference = MOTOR_MAX_SPEED_RPM/60;
	}
	else if(hSpeedReference < -MOTOR_MAX_SPEED_RPM/60)
	{
		hSpeedReference = -MOTOR_MAX_SPEED_RPM/60;
	}

  return (int16_t)hSpeedReference;
}

/**
  * @brief  It computes the new values current referrence
  * @param  target angle
  * @retval Iqref
  */
int16_t Position_CalcTorqueReferrence(void)
{
  int16_t hTorqueReference = 0;

  /* Calculate Iqref using PID*/
  hTorqueReference = PI_Controller( &PIDAngleHandle_M1, ( int32_t )Position_M1.hError_Angle );

  return hTorqueReference;
}

/*******前馈控制******/
int16_t FeedforwardController(FeedForward_Handle_t * vFFC)
{
  float result;
  float s0,s1,s2;
  s0 = 0.005493164f*Position_GetMecAngle();
  vFFC->POSAngle = s0;
  s1 = s0-vFFC->lastPOSAngle;
  s2 = vFFC->lastPOSAngle-vFFC->perrPOSAngle;
  result = vFFC->Kvff_Gain*s1+ vFFC->Kaff_Gain*(s1-s2);
  vFFC->perrPOSAngle= vFFC->lastPOSAngle;
  vFFC->lastPOSAngle= s0;
  return (int16_t)result;
}


int32_t Position_GetAngle(void)   //获取电机当前位置，方便传输，需进行换算
{
  return  Position_M1.hMeasuredAngle;
}

void  Position_SetAngle(int32_t hTargetAngle)  //设定目标位置，单位°
{
  Position_M1.hTargetAngle = hTargetAngle*9102;
}



//*****************************模糊PI相关函数*********************************//



//隶属度计算函数 ms0 ms1为三角隶属度韩式左右两部分  qv为论域范围内的具体值  index为索引值用于查表
 void CalcMembership(float *ms,float qv,int * index)
{
 if((qv>=NB)&&(qv<NM))
 {
 index[0]=0;
 index[1]=1;
 ms[0]=-0.5*qv-2.0;  //y=-0.5x-2.0
 ms[1]=0.5*qv+3.0;  //y=0.5x+3.0
 }
 else if((qv>=NM)&&(qv<NS))
 {
 index[0]=1;
 index[1]=2;
 ms[0]=-0.5*qv-1.0;  //y=-0.5x-1.0
 ms[1]=0.5*qv+2.0;  //y=0.5x+2.0
 }
 else if((qv>=NS)&&(qv<ZO))
 {
 index[0]=2;
 index[1]=3;
 ms[0]=-0.5*qv;  //y=-0.5x
 ms[1]=0.5*qv+1.0;  //y=0.5x+1.0
 }
 else if((qv>=ZO)&&(qv<PS))
 {
 index[0]=3;
 index[1]=4;
 ms[0]=-0.5*qv+1.0;  //y=-0.5x+1.0
 ms[1]=0.5*qv;  //y=0.5x
 }
 else if((qv>=PS)&&(qv<PM))
 {
 index[0]=4;
 index[1]=5;
 ms[0]=-0.5*qv+2.0;  //y=-0.5x+2.0
 ms[1]=0.5*qv-1.0;  //y=0.5x-1.0
 }
 else if((qv>=PM)&&(qv<=PB))
 {
 index[0]=5;
 index[1]=6;
 ms[0]=-0.5*qv+3.0;  //y=-0.5x+3.0
 ms[1]=0.5*qv-2.0;  //y=0.5x-2.0
  }
}

//输入值的量化论域(-6->6)
 void LinearQuantization(Position_Handle_t *vPID,float *qValue)
{
 int32_t thisError;
 int32_t deltaError;
 
 thisError=vPID->hError_Angle;   //计算当前偏差e
 deltaError=thisError-vPID->hLastError_Angle;   //计算偏差增量ec
	
//E和EC的量化
 qValue[0]=vPID->Postiion_KeGain*thisError;
 qValue[1]=vPID->Postiion_KecGain*deltaError;
 if (qValue[0] > 6.0F) {
    qValue[0] = 6.0F;
  }
 else {
    if (qValue[0] < -6.0F) {
      qValue[0] = -6.0F;
    }
 }
 
 if (qValue[1] > 6.0F) {
    qValue[1] = 6.0F;
  }
 else {
    if (qValue[1] < -6.0F) {
      qValue[1] = -6.0F;
    }
 }

}


//解模糊PI
 void FuzzyPIComputation (Position_Handle_t *vPID)
{
//量化值
 float qValue[2]={0,0};  

 int indexE[2]={0,0};     //e在规则库中的索引
 float msE[2]={0,0};      //e的隶属度
 
 int indexEC[2]={0,0};    //ec在规则库中的索引
 float msEC[2]={0,0};      //ec的隶属度
 
 //pid增量值
 int16_t pidvalue[2];
 
//量化
 LinearQuantization(vPID,qValue);
//计算e的隶属度和索引
 CalcMembership(msE,qValue[0],indexE);
//计算ec的隶属度和索引
 CalcMembership(msEC,qValue[1],indexEC);
 
 //采用重心法计算pid增量值
 pidvalue[0]= (int16_t)(msE[0]*(msEC[0]*ruleKp[indexE[0]][indexEC[0]]+msEC[1]*ruleKp[indexE[0]][indexEC[1]]) 
            +msE[1]*(msEC[0]*ruleKp[indexE[1]][indexEC[0]]+msEC[1]*ruleKp[indexE[1]][indexEC[1]]));
 pidvalue[1]= (int16_t)(msE[0]*(msEC[0]* ruleKi[indexE[0]][indexEC[0]]+msEC[1]*ruleKi[indexE[0]][indexEC[1]])
            +msE[1]*(msEC[0]*ruleKi[indexE[1]][indexEC[0]]+msEC[1]*ruleKi[indexE[1]][indexEC[1]]));


//pi增量修正
 vPID->deta_Kp=vPID->Postiion_KupGain*pidvalue[0];
 vPID->deta_Ki=vPID->Postiion_KuiGain*pidvalue[1];

}

//解模糊
 void FuzzyComputation (Position_Handle_t *vPID)
{
//量化值
 float qValue[2]={0,0};  

 int indexE[2]={0,0};     //e在规则库中的索引
 float msE[2]={0,0};      //e的隶属度
 
 int indexEC[2]={0,0};    //ec在规则库中的索引
 float msEC[2]={0,0};      //ec的隶属度
 
 //模糊输出值
 int16_t outvalue;
 
//量化
 LinearQuantization(vPID,qValue);
//计算e的隶属度和索引
 CalcMembership(msE,qValue[0],indexE);
//计算ec的隶属度和索引
 CalcMembership(msEC,qValue[1],indexEC);
 
 //采用重心法计算模糊输出
 outvalue= (int16_t)(msE[0]*(msEC[0]*ruleKu[indexEC[0]][indexE[0]]+msEC[1]*ruleKu[indexEC[0]][indexE[1]]) 
            +msE[1]*(msEC[0]*ruleKu[indexEC[1]][indexE[0]]+msEC[1]*ruleKu[indexEC[1]][indexE[1]]));
// outvalue= (int16_t)(msE[0]*(msEC[0]*ruleKu[indexE[0]][indexEC[0]]+msEC[1]*ruleKu[indexE[0]][indexEC[1]]) 
//            +msE[1]*(msEC[0]*ruleKu[indexE[1]][indexEC[0]]+msEC[1]*ruleKu[indexE[1]][indexEC[1]]));
 

//添加比例因子
 vPID->FuzzySpeed=vPID->Postiion_KupGain*outvalue;

}

int16_t Position_FuzzyPI(Position_Handle_t *pHandle)
{
  int32_t wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
  int32_t wDischarge = 0;
  //输出限幅
  int16_t hUpperOutputLimit =6000;
  int16_t hLowerOutputLimit = -6000;
  //积分限幅
  int32_t wUpperIntegralLimit = pHandle->Postiion_KiDiv * hUpperOutputLimit;
  int32_t wLowerIntegralLimit = pHandle->Postiion_KiDiv * hLowerOutputLimit;
  
  //计算模糊增量
  FuzzyPIComputation(pHandle);
  
  //比例项计算
  wProportional_Term = (pHandle->Position_KpGain + pHandle->deta_Kp) * pHandle-> hError_Angle;
 
  //积分项计算 
  if ( pHandle->Position_KiGain == 0 )
  {
    pHandle->wIntegralTerm = 0;
  }
  else
  {
    wIntegral_Term = (pHandle->Position_KiGain + pHandle->deta_Ki)* pHandle-> hError_Angle;
    wIntegral_sum_temp = pHandle->wIntegralTerm + wIntegral_Term;

    if ( wIntegral_sum_temp < 0 )
    {
      if ( pHandle->wIntegralTerm > 0 )
      {
        if ( wIntegral_Term > 0 )
        {
          wIntegral_sum_temp = INT32_MAX;
        }
      }
    }
    else
    {
      if ( pHandle->wIntegralTerm < 0 )
      {
        if ( wIntegral_Term < 0 )
        {
          wIntegral_sum_temp = -INT32_MAX;
        }
      }
    }
    if ( wIntegral_sum_temp > wUpperIntegralLimit )
    {
      pHandle->wIntegralTerm = wUpperIntegralLimit;
    }
    else if ( wIntegral_sum_temp < wLowerIntegralLimit )
    {
      pHandle->wIntegralTerm = wLowerIntegralLimit;
    }
    else
    {
      pHandle->wIntegralTerm = wIntegral_sum_temp;
    }
  }

//#ifdef FULL_MISRA_C_COMPLIANCY
  wOutput_32 = ( wProportional_Term / ( int32_t )pHandle->Postiion_KpDiv ) + ( pHandle->wIntegralTerm /
               ( int32_t )pHandle->Postiion_KiDiv );
//#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/
//  wOutput_32 = ( wProportional_Term >> pHandle->hKpDivisorPOW2 ) + ( pHandle->wIntegralTerm >> pHandle->hKiDivisorPOW2 );
//#endif

  if ( wOutput_32 > hUpperOutputLimit )
  {

    wDischarge = hUpperOutputLimit - wOutput_32;
    wOutput_32 = hUpperOutputLimit;
  }
  else if ( wOutput_32 < hLowerOutputLimit )
  {

    wDischarge = hLowerOutputLimit - wOutput_32;
    wOutput_32 = hLowerOutputLimit;
  }
  else { /* Nothing to do here */ }

  pHandle->wIntegralTerm += wDischarge;

  return ( ( int16_t )( wOutput_32 ) );
}


//*******************************************双模控制********************************************//
int16_t Position_CompoundFuzzy(Position_Handle_t *pHandle)
{
  
  int32_t wOutput_32;
//  if (pHandle->hError_Angle >= 1000)
//  {
    FuzzyComputation(pHandle);   //计算模糊增量
    wOutput_32 = pHandle->FuzzySpeed;
//  }
//  else
//  {
//    wOutput_32 = pHandle->hError_Angle * pHandle->Position_KpGain / 
//     pHandle->Postiion_KpDiv;
//  }
  
  return ( ( int16_t )( wOutput_32 ) );
}

/******************位置环参数更改接口函数************************/

 void Position_SetKP( Position_Handle_t * pHandle, int16_t Position_KpGain )
{
  pHandle->Position_KpGain = Position_KpGain;
}
 void Position_SetKI( Position_Handle_t * pHandle, int16_t Position_KiGain )
{
  pHandle->Position_KiGain = Position_KiGain;
}
 void Position_SetKE( Position_Handle_t * pHandle, float Postiion_KeGain )
{
  pHandle->Postiion_KeGain = Postiion_KeGain;
}
 void Position_SetKEC( Position_Handle_t * pHandle, float Postiion_KecGain )
{
  pHandle->Postiion_KecGain = Postiion_KecGain;
}
 void Position_SetKUP( Position_Handle_t * pHandle, int16_t Postiion_KupGain )
{
  pHandle->Postiion_KupGain = Postiion_KupGain;
}
 void Position_SetKUI( Position_Handle_t * pHandle, int16_t Postiion_KuiGain )
{
  pHandle->Postiion_KuiGain = Postiion_KuiGain;
} 

void Position_SetKvff( FeedForward_Handle_t * pHandle, float Kvff_Gain )
{
  pHandle->Kvff_Gain = Kvff_Gain;
}

void Position_SetKaff( FeedForward_Handle_t * pHandle, float Kaff_Gain )
{
  pHandle->Kaff_Gain = Kaff_Gain;
}
 int16_t Position_GetKP( Position_Handle_t * pHandle )
{
  return ( pHandle->Position_KpGain );
}
 int16_t Position_GetTargrtangle(void)
{
  return ( Position_M1.hTargetAngle/9102);
}

/******END OF FILE****/
