
/**
  ******************************************************************************
  * @file    mc_position.h
  * @author  Ken An
  * @brief   This file provides firmware functions for position control
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pmsm_motor_parameters.h"
#include "pid_regulator.h"


/* Define positon control structure */
typedef struct
{	
	/* HW Settings */
  TIM_TypeDef * TIMx;   /*!< Timer used for ENCODER sensor management.*/
	
	volatile int32_t hMeasuredAngle; /* Real rotor Angle */
	volatile int32_t hTargetAngle;   /* Target angle */
	volatile int32_t hError_Angle;   /* Error angle */
        volatile int32_t hLastError_Angle;   /* Lasr Error angle */
	
	int16_t Control_Speed;           /* Speed control value */
	int16_t Real_Speed;	             /* Real average speed */
	uint16_t Duration_ms;            /* Speed ramp, used for speed accelarate */
	int16_t Set_Speed;               /* Position control set speed */

	int16_t Position_KpGain;          /* Position Kp gain */
	int16_t Postiion_KpDiv;           /* Position Kp divider */
	int16_t Position_KiGain;          /* Position Ki gain */
	int16_t Postiion_KiDiv;           /* Position Ki divider */
        int32_t wIntegralTerm;            /* integral term*/
        float  Postiion_KeGain;
        float  Postiion_KecGain;
        int16_t Postiion_KupGain;
        int16_t Postiion_KuiGain;
        int16_t deta_Kp;
        int16_t deta_Ki;
        int16_t FuzzySpeed;
	int16_t Speed_ACC_Gain;         /* Speed accelarate Kp gain */
	int16_t Speed_ACC_Div;          /* Speed accelarate Kp divider */
	int16_t Speed_ACC_Count;        /* Speed accelarate Kp period time */
	
	int16_t Encoder_Update_Count;    /* Encoder timer update count */
	int32_t Encoder_Direction;          /* Encoder runing direction */
	int32_t Encoder_Pre_Direction;      /* Encoder pre runing direction*/
	
	bool Mode_Flag;                  /* Mode_Flag = 1, under torque mode, Mode_Flag = 0,under speed mode */
	int32_t Torque_First_Flag;          /* First torque mode flag */
} Position_Handle_t;

typedef struct
{  
  float POSAngle;               //电机位置当前值*/
  float lastPOSAngle;           //电机位置前一拍值*/           
  float  perrPOSAngle;           //电机位置前两拍值*/
  float  Kvff_Gain;                  //速度前馈 */
  float  Kaff_Gain;                  //加速度前馈 */
} FeedForward_Handle_t;



/* Define mode */
#define P_TORQUE_MODE 1
#define P_SPEED_MODE  0

/* Do ACC duration calculate rate: (Speed loop time * SPEED_ACC_RATE) */
#define SPEED_ACC_RATE 10

#define _IQ24(A) (long) ((A) * 16777216.0L)
#define IQ24_PI 52707178 // 3.14159265358979323846 * 16777216
#define s16degree 0.0000958737992f            /* 2pi/65536 */
#define Rad_angle 57.295779513     /*180/pi*/
/* PI_MUL = 2*PI*10000 */
#define PI_MUL 62832

/* Define low delta angle and high delta angle, used for control mode change */
#define CHANGE_LIMIT_LOW  628320
#define CHANGE_LIMIT_HIGH 628320*2//62832*2


int32_t Position_GetErrorAngle(void);
float Position_GetErrorVoltage(void);
int16_t Position_CalcSpeedReferrence(void);
int16_t Position_CalcTorqueReferrence(void);
int16_t FeedforwardController(FeedForward_Handle_t * vFFC);
void Position_SetKaff( FeedForward_Handle_t * pHandle, float Kaff_Gain );
void Position_SetKvff( FeedForward_Handle_t * pHandle, float Kvff_Gain );
void Position_SetKP( Position_Handle_t * pHandle, int16_t Position_KpGain );
void Position_SetKI( Position_Handle_t * pHandle, int16_t Position_KiGain );
void Position_SetKE( Position_Handle_t * pHandle, float Postiion_KeGain );
void Position_SetKEC( Position_Handle_t * pHandle, float Postiion_KecGain );
void Position_SetKUP( Position_Handle_t * pHandle, int16_t Postiion_KupGain );
void Position_SetKUI( Position_Handle_t * pHandle, int16_t Postiion_KuiGain );
int16_t Position_GetKP( Position_Handle_t * pHandle );
int16_t Position_GetTargrtangle(void);
//float Position_GetAngle(void);
int32_t Position_GetAngle(void);
void  Position_SetAngle(int32_t hTargetAngle);
uint32_t Abs_Value(int32_t dat);
int16_t Position_FuzzyPI(Position_Handle_t *pHandle);
void FuzzyPIComputation (Position_Handle_t *vPID);
void LinearQuantization(Position_Handle_t *vPID,float *qValue);
void CalcMembership(float *ms,float qv,int * index);
int16_t Position_Fuzzy(Position_Handle_t *pHandle);
int16_t Position_CompoundFuzzy(Position_Handle_t *pHandle);
extern PID_Handle_t PIDAngleHandle_M1;
extern Position_Handle_t Position_M1;
extern FeedForward_Handle_t FF_M1;

/******END OF FILE****/
