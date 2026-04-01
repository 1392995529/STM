/**
  ******************************************************************************
  * @file    mc_position.c
  * @author  Ken An (Modified for PI + Feedforward - Rotary Axis)
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
// 确保 mc_tasks.c 能引用到这个前馈句柄
FeedForward_Handle_t FF_M1; 

/* ==============================================================================
   1. 参数结构体初始化
   ============================================================================== */

Position_Handle_t Position_M1 =   
{
    .hTargetAngle        = 0,
    .Position_KpGain     = 8,       // 回转轴初始 P (保持原值)
    .Postiion_KpDiv      = 2048,    
    .Position_KiGain     = 2,       // 回转轴初始 I (保持原值)
    .Postiion_KiDiv      = 2048,
    
    .wIntegralTerm       = 0,       // 积分累加器
    
    // --- 以下参数为兼容性保留，实际逻辑不使用 ---
    .Postiion_KeGain     = 0.2,
    .Postiion_KecGain    = 0.02, 
    .Postiion_KupGain    = 35,
    .Postiion_KuiGain    = 1,
    .Speed_ACC_Gain      = 1,
    .Speed_ACC_Div       = 3,
    .Speed_ACC_Count     = 0,
    .Encoder_Update_Count= 0,
    .Encoder_Direction   = 0,
    .Encoder_Pre_Direction= 0,
    .Mode_Flag           = P_SPEED_MODE,
    .Torque_First_Flag   = 0,
};

/* 初始化前馈结构体 */
FeedForward_Handle_t FF_M1 =   
{
    .POSAngle      = 0,
    .lastPOSAngle  = 0,
    .perrPOSAngle  = 0,
    .Kvff_Gain     = 0, // 速度前馈系数
    .Kaff_Gain     = 0, // 加速度前馈系数
};

// 宏定义保留
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

// PID句柄保留（防报错）
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

/* ==============================================================================
   2. 辅助与计算函数
   ============================================================================== */

uint32_t Abs_Value(int32_t dat)
{
    if(dat < 0) return -dat;
    return dat;
}

// 1. 获取编码器误差 (定位模式)
int32_t Position_GetErrorAngle(void) 
{
    Position_M1.hMeasuredAngle = Position_GetMecAngle();   
    Position_M1.hLastError_Angle = Position_M1.hError_Angle;
    Position_M1.hError_Angle = Position_M1.hTargetAngle - Position_M1.hMeasuredAngle;
    return Position_M1.hError_Angle;
}

// 2. 获取PSD电压误差 (追踪模式)
float Position_GetErrorVoltage(void)  
{
    float a;
    Position_M1.hLastError_Angle = Position_M1.hError_Angle;   
    
    // 读取数据 (解决 Volatile 警告: 先读入临时变量)
    int16_t raw_data = DB_data[0]; 
    a = raw_data * 10000.0f / 32768.0f; 
    
    // 转换为角度误差 (系数8)
    // 注意：回转轴原程序中有取反操作 (b = -a)，这里保留该逻辑 (-8 * a)
    Position_M1.hError_Angle = (int32_t)(-8 * a);        
    
    Position_M1.hMeasuredAngle = Position_GetMecAngle();    
    return a;
}

// 3. 前馈控制器计算
int16_t FeedforwardController(FeedForward_Handle_t * vFFC)
{
    float result;
    float s0,s1,s2;
    
    s0 = 0.005493164f * Position_GetMecAngle(); // 当前角度
    vFFC->POSAngle = s0;
    
    s1 = s0 - vFFC->lastPOSAngle;               // 速度
    s2 = vFFC->lastPOSAngle - vFFC->perrPOSAngle; // 加速度相关
    
    // 前馈输出 = Kv * 速度 + Ka * 加速度
    result = vFFC->Kvff_Gain * s1 + vFFC->Kaff_Gain * (s1 - s2);
    
    vFFC->perrPOSAngle = vFFC->lastPOSAngle;
    vFFC->lastPOSAngle = s0;
    
    return (int16_t)result;
}

/**
  * @brief  核心计算：PI + 前馈 -> 速度参考
  */
int16_t Position_CalcSpeedReferrence(void)
{
    int32_t wProportional_Term = 0;
    int32_t wIntegral_Term = 0;
    int32_t wOutput_32 = 0;
    int16_t hSpeedReference = 0;
    int32_t hSpeedReference_FF = 0;

    // --- Part A: PI 计算 ---
    // P项
    wProportional_Term = (Position_M1.hError_Angle * Position_M1.Position_KpGain) / Position_M1.Postiion_KpDiv;

    // I项
    if (Position_M1.Position_KiGain != 0)
    {
        wIntegral_Term = (Position_M1.hError_Angle * Position_M1.Position_KiGain) / Position_M1.Postiion_KiDiv;
        Position_M1.wIntegralTerm += wIntegral_Term;

        // 积分抗饱和 (限制最大积分量)
        int32_t limit = 3000; 
        if (Position_M1.wIntegralTerm > limit) Position_M1.wIntegralTerm = limit;
        if (Position_M1.wIntegralTerm < -limit) Position_M1.wIntegralTerm = -limit;
    }
    else
    {
        Position_M1.wIntegralTerm = 0;
    }

    // --- Part B: 前馈 计算 ---
    hSpeedReference_FF = FeedforwardController(&FF_M1);

    // --- Part C: 合成 (PI + FF) ---
    wOutput_32 = wProportional_Term + Position_M1.wIntegralTerm + hSpeedReference_FF;

    // --- Part D: 总输出限幅 ---
    int16_t max_speed = MOTOR_MAX_SPEED_RPM/60; 
    if(wOutput_32 > max_speed)      hSpeedReference = max_speed;
    else if(wOutput_32 < -max_speed) hSpeedReference = -max_speed;
    else                             hSpeedReference = (int16_t)wOutput_32;

    // 调试监视
    Position_M1.Encoder_Direction = (int16_t)(wProportional_Term + Position_M1.wIntegralTerm); // PI部分
    Position_M1.Encoder_Pre_Direction = (int16_t)hSpeedReference_FF; // 前馈部分

    return hSpeedReference;
}

/* ==============================================================================
   3. 接口函数 (含空壳函数以消除 Linker Error)
   ============================================================================== */

void Position_SetKP( Position_Handle_t * pHandle, int16_t Position_KpGain ) {
  pHandle->Position_KpGain = Position_KpGain;
}

void Position_SetKI( Position_Handle_t * pHandle, int16_t Position_KiGain ) {
  pHandle->Position_KiGain = Position_KiGain;
  if(Position_KiGain == 0) pHandle->wIntegralTerm = 0; // 清零积分
}

/* 恢复前馈设置接口 */
void Position_SetKvff( FeedForward_Handle_t * pHandle, float Kvff_Gain ) {
  pHandle->Kvff_Gain = Kvff_Gain;
}
void Position_SetKaff( FeedForward_Handle_t * pHandle, float Kaff_Gain ) {
  pHandle->Kaff_Gain = Kaff_Gain;
}

/* --- 空壳函数 (为了兼容 user_interface.c) --- */
void Position_SetKE( Position_Handle_t * pHandle, float Postiion_KeGain ) {}
void Position_SetKEC( Position_Handle_t * pHandle, float Postiion_KecGain ) {}
void Position_SetKUP( Position_Handle_t * pHandle, int16_t Postiion_KupGain ) {}
void Position_SetKUI( Position_Handle_t * pHandle, int16_t Postiion_KuiGain ) {}
void CalcMembership(float *ms, float qv, int * index) {} // 兼容 pid_regulator.o

int16_t Position_GetKP( Position_Handle_t * pHandle ) {
  return ( pHandle->Position_KpGain );
}
int16_t Position_GetTargrtangle(void) {
  return ( Position_M1.hTargetAngle/9102 );
}
int32_t Position_GetAngle(void) {
  return  Position_M1.hMeasuredAngle;
}
void Position_SetAngle(int32_t hTargetAngle) {
  Position_M1.hTargetAngle = hTargetAngle*9102;
}
int16_t Position_CalcTorqueReferrence(void) {
  return PI_Controller( &PIDAngleHandle_M1, ( int32_t )Position_M1.hError_Angle );
}