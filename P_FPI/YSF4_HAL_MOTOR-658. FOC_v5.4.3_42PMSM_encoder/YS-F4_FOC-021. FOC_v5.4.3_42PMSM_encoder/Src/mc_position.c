/**
  ******************************************************************************
  * @file    mc_position.c
  * @author  Ken An (Modified for PI + Feedforward - Pitch Axis)
  * @brief   This file provides firmware functions for position control
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_position.h"
#include "speed_pos_fdbk.h"
#include "mc_tasks.h"
#include "bsp_key.h"

// 外部变量
extern int16_t DB_data[8];
extern Position_Handle_t *pPIDPosition;
extern FeedForward_Handle_t *FFPosition; // 必须保留，mc_tasks.c 需要用到

/* ================== 1. 参数结构体定义 ================== */

/* 位置环 PI 参数结构体 */
Position_Handle_t Position_M1 =   
{
    .hTargetAngle        = 0,
    .Position_KpGain     = 6,       // 初始 P 参数
    .Postiion_KpDiv      = 2048,    
    .Position_KiGain     = 0,       // 初始 I 参数
    .Postiion_KiDiv      = 2048,
    
    .wIntegralTerm       = 0,       // 积分累加器
    
    // 兼容性保留参数（UI协议需要，但不参与计算）
    .Postiion_KeGain     = 0,
    .Postiion_KecGain    = 0, 
    .Postiion_KupGain    = 0,
    .Postiion_KuiGain    = 0,
    .Speed_ACC_Gain      = 1,
    .Speed_ACC_Div       = 3,
    .Speed_ACC_Count     = 0,
    .Encoder_Update_Count= 0,
    .Encoder_Direction   = 0,
    .Encoder_Pre_Direction= 0,
    .Mode_Flag           = P_SPEED_MODE,
    .Torque_First_Flag   = 0,
};

/* 前馈控制参数结构体 */
FeedForward_Handle_t FF_M1 =   
{
    .POSAngle      = 0,
    .lastPOSAngle  = 0,
    .perrPOSAngle  = 0,
    .Kvff_Gain     = 0, // 速度前馈系数 (调试时在上位机设置)
    .Kaff_Gain     = 0, // 加速度前馈系数
};

// 兼容旧代码的宏定义
#define PID_ANGLE_KP_DEFAULT 3000
#define AG_KPDIV 2048
#define AG_KPDIV_LOG 11
#define PID_ANGLE_KI_DEFAULT 2
#define AG_KIDIV 16384
#define AG_KIDIV_LOG 14
#define PID_ANGLE_KD_DEFAULT 0
#define AG_KDDIV 16384
#define AG_KDDIV_LOG 14
#define IQ_POSITION 4000

// 备用的 PID 句柄
PID_Handle_t PIDAngleHandle_M1 = {
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

/* ================== 2. 辅助函数 ================== */

uint32_t Abs_Value(int32_t dat)
{
    if(dat < 0) return -dat;
    return dat;
}

// 恢复前馈控制器 (计算速度和加速度补偿)
int16_t FeedforwardController(FeedForward_Handle_t * vFFC)
{
    float result;
    float s0,s1,s2;
    
    // 获取当前机械角度并归一化
    s0 = 0.005493164f * Position_GetMecAngle(); 
    vFFC->POSAngle = s0;
    
    // 计算一阶导数 (速度趋势)
    s1 = s0 - vFFC->lastPOSAngle;
    
    // 计算二阶导数 (加速度趋势)
    s2 = vFFC->lastPOSAngle - vFFC->perrPOSAngle;
    
    // 前馈输出 = Kv * 速度 + Ka * 加速度
    result = vFFC->Kvff_Gain * s1 + vFFC->Kaff_Gain * (s1 - s2);
    
    // 更新历史状态
    vFFC->perrPOSAngle = vFFC->lastPOSAngle;
    vFFC->lastPOSAngle = s0;
    
    return (int16_t)result;
}

/* ================== 3. 核心计算函数 ================== */

// 获取编码器误差 (定位模式)
int32_t Position_GetErrorAngle(void) 
{
    Position_M1.hMeasuredAngle = Position_GetMecAngle();   
    Position_M1.hLastError_Angle = Position_M1.hError_Angle;
    Position_M1.hError_Angle = Position_M1.hTargetAngle - Position_M1.hMeasuredAngle;
    return Position_M1.hError_Angle;
}

// 获取PSD误差 (追踪模式)
float Position_GetErrorVoltage(void)  
{
    float a;
    Position_M1.hLastError_Angle = Position_M1.hError_Angle;   
    
    // 俯仰轴获取 Data[0] (请根据硬件接线确认)
    a = DB_data[0] * 10000 / 32768; 
    
    // 8.0 是PSD电压到角度的敏感度系数
    Position_M1.hError_Angle = (int32_t)(8 * a);        
    
    Position_M1.hMeasuredAngle = Position_GetMecAngle();    
    return a;
}

/**
  * @brief  计算最终速度参考值 (PI + Feedforward)
  * @原理   Speed = (Kp*Error + Ki*SumError) + (Kv*Vel + Ka*Accel)
  */
int16_t Position_CalcSpeedReferrence(void)
{
    int32_t wProportional_Term = 0;
    int32_t wIntegral_Term = 0;
    int32_t wOutput_32 = 0;
    int16_t hSpeedReference = 0;
    int32_t hSpeedReference_PI = 0;
    int32_t hSpeedReference_FF = 0;

    // --- 1. PI 计算部分 ---
    
    // P项
    wProportional_Term = (Position_M1.hError_Angle * Position_M1.Position_KpGain) / Position_M1.Postiion_KpDiv;

    // I项
    if (Position_M1.Position_KiGain != 0)
    {
        wIntegral_Term = (Position_M1.hError_Angle * Position_M1.Position_KiGain) / Position_M1.Postiion_KiDiv;
        Position_M1.wIntegralTerm += wIntegral_Term;

        // 积分抗饱和 (根据最大转速限制)
        int32_t limit = 3000; 
        if (Position_M1.wIntegralTerm > limit) Position_M1.wIntegralTerm = limit;
        if (Position_M1.wIntegralTerm < -limit) Position_M1.wIntegralTerm = -limit;
    }
    else
    {
        Position_M1.wIntegralTerm = 0;
    }
    
    hSpeedReference_PI = wProportional_Term + Position_M1.wIntegralTerm;

    // --- 2. 前馈 (Feedforward) 计算部分 ---
    hSpeedReference_FF = FeedforwardController(&FF_M1);

    // --- 3. 合成输出 ---
    wOutput_32 = hSpeedReference_PI + hSpeedReference_FF;

    // --- 4. 总限幅 ---
    // 限制在电机物理允许的最大转速内
    int16_t max_speed = MOTOR_MAX_SPEED_RPM/60; 
    
    if(wOutput_32 > max_speed)
    {
        hSpeedReference = max_speed;
    }
    else if(wOutput_32 < -max_speed)
    {
        hSpeedReference = -max_speed;
    }
    else
    {
        hSpeedReference = (int16_t)wOutput_32;
    }

    // 调试赋值
    Position_M1.Encoder_Direction = hSpeedReference_PI; // 可以在这里看PI输出了多少
    Position_M1.Encoder_Pre_Direction = hSpeedReference_FF; // 可以在这里看前馈输出了多少

    return hSpeedReference;
}

int16_t Position_CalcTorqueReferrence(void)
{
  return PI_Controller( &PIDAngleHandle_M1, ( int32_t )Position_M1.hError_Angle );
}

int32_t Position_GetAngle(void)   
{
  return  Position_M1.hMeasuredAngle;
}

void  Position_SetAngle(int32_t hTargetAngle)  
{
  Position_M1.hTargetAngle = hTargetAngle*9102;
}

/* ================== 4. 参数设置接口 ================== */

void Position_SetKP( Position_Handle_t * pHandle, int16_t Position_KpGain )
{
  pHandle->Position_KpGain = Position_KpGain;
}

void Position_SetKI( Position_Handle_t * pHandle, int16_t Position_KiGain )
{
  pHandle->Position_KiGain = Position_KiGain;
  if(Position_KiGain == 0) {
      pHandle->wIntegralTerm = 0;
  }
}

/* 恢复前馈系数设置功能 */
void Position_SetKvff( FeedForward_Handle_t * pHandle, float Kvff_Gain )
{
  pHandle->Kvff_Gain = Kvff_Gain;
}

void Position_SetKaff( FeedForward_Handle_t * pHandle, float Kaff_Gain )
{
  pHandle->Kaff_Gain = Kaff_Gain;
}

/* 空接口，保留以防 user_interface.c 报错 */
void Position_SetKE( Position_Handle_t * pHandle, float Postiion_KeGain ) {}
void Position_SetKEC( Position_Handle_t * pHandle, float Postiion_KecGain ) {}
void Position_SetKUP( Position_Handle_t * pHandle, int16_t Postiion_KupGain ) {}
void Position_SetKUI( Position_Handle_t * pHandle, int16_t Postiion_KuiGain ) {}
void CalcMembership(float *ms, float qv, int * index) {} // 防止 pid_regulator.o 报错

int16_t Position_GetKP( Position_Handle_t * pHandle )
{
  return ( pHandle->Position_KpGain );
}

int16_t Position_GetTargrtangle(void)
{
  return ( Position_M1.hTargetAngle/9102);
}