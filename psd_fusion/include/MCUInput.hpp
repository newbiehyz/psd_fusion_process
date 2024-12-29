
#pragma once

#ifndef MCUINPUT_H
#define MCUINPUT_H

#include "internalSOCv1_2.hpp"
// 时间戳
typedef struct
{
	unsigned int USS_TimeStamp; 			// 超声波时间戳
	unsigned int CAN_WheelPulse_TimeStamp; 	// 轮速脉冲时间戳
	unsigned int SYS_TimeStamp; 			// 系统时间戳
}App2Alg_ReceiveTimeStamp; 

// 巡航搜索车位的控制参数
typedef struct
{
	float Command_ParkFuncMode_RPPControl_Slope;
	float Command_ParkFuncMode_RPPControl_b;
	float Command_ParkFuncMode_RPPControl_lineType;
}App2Alg_ControlCommand_Parameter;

// 各雷达距离信息
typedef struct
{
	unsigned short USS_DISTANCE_FMR;
	unsigned short USS_DISTANCE_FML;
	unsigned short USS_DISTANCE_FLS;
	unsigned short USS_DISTANCE_FL;
	unsigned short USS_DISTANCE_RLS;
	unsigned short USS_DISTANCE_RL;
	unsigned short USS_DISTANCE_FRS;
	unsigned short USS_DISTANCE_FR;
	unsigned short USS_DISTANCE_RRS;
	unsigned short USS_DISTANCE_RR;
	unsigned short USS_DISTANCE_RMR;
	unsigned short USS_DISTANCE_RML;
}USSDistance_RxTx, USSDistance_Triangle_Value;

/* --------------------------------- 2.CAN信息 --------------------------------- */
typedef struct
{
	unsigned char CanSig_EngineStatus;
	unsigned char CanSig_AcceleratorPedalPosition;
	unsigned char CanSig_GearStatus;
	float CanSig_SteerWheelAngle;
	float CanSig_VehicleSpeed;
	unsigned char CanSig_VehicleStandStill;
	unsigned char CanSig_BreakPedalStatus;
	float CanSig_LateralAcceleration;
	float CanSig_LongitAcceleration;
	float CanSig_YawRate;
	unsigned short CanSig_FlWheelPulse;
	unsigned short CanSig_FrWheelPulse;
	unsigned short CanSig_RlWheelPulse;
	unsigned short CanSig_RrWheelPulse;
	float CanSig_FlWheelSpeed;
	float CanSig_FrWheelSpeed;
	float CanSig_RlWheelSpeed;
	float CanSig_RrWheelSpeed;
	unsigned char CanSig_FLWheelDirection;
	unsigned char CanSig_FRWheelDirection;
	unsigned char CanSig_RLWheelDirection;
	unsigned char CanSig_RRWheelDirection;
	unsigned char CanSig_EpbStatus;
	short CanSig_Ramp;
	unsigned char CanSig_PepsPowerModeStatus;
	unsigned char CanSig_RainFallLevel;
	float CanSig_AmbientTemperature;
}App2Alg_Can;

// 影像目标车位
typedef struct
{
	float TargetSlot_Ax;
	float TargetSlot_Ay;
	float TargetSlot_Bx;
	float TargetSlot_By;
	float TargetSlot_Cx;
	float TargetSlot_Cy;
	float TargetSlot_Dx;
	float TargetSlot_Dy;
}App2Alg_ParkingPlace_TargetSlot;

// 超声波车位
typedef struct
{
	float UssSlot_Ax;
	float UssSlot_Ay;
	float UssSlot_Bx;
	float UssSlot_By;
	float UssSlot_Cx;
	float UssSlot_Cy;
	float UssSlot_Dx;
	float UssSlot_Dy;
}App2Alg_ParkingPlace_UssSlot;

// 重定位(侧方水平泊车)
typedef struct
{
	unsigned char AvmSlotRelocation_Top_Validity;
	unsigned char AvmSlotRelocation_Down_Validity;
	float AvmSlotRelocation_Top_Theta;
	float AvmSlotRelocation_Down_Theta;
	short AvmSlotRelocation_Top_Distance;
	short AvmSlotRelocation_Down_Distance;
}App2Alg_ParkingPlace_AvmSlotTopLowRelocation;

// 重定位(垂直&斜车位泊车)
typedef struct
{
	unsigned char AvmSlotRelocation_Left_Validity;
	unsigned char AvmSlotRelocation_Right_Validity;
	float AvmSlotRelocation_Left_Theta;
	float AvmSlotRelocation_Right_Theta;
	short AvmSlotRelocation_Left_Distance;
	short AvmSlotRelocation_Right_Distance;
}App2Alg_ParkingPlace_AvmSlotLeRiRelocation;

// 空间信息
typedef struct
{
	float AvmSpaceDetect_OppositeSpace;
	unsigned short AvmSpaceDetect_NeighboringSlotStatus;
	unsigned char AvmSpaceDetect_TargetSlotType;
}App2Alg_ParkingPlace_AvmSpaceDetect;

// 控制命令
typedef struct
{
	unsigned char Command_Dr_Initialize; // No Use
	unsigned char Command_Dr_Calculate;  // No Use
	unsigned char Command_ParkFuncSelectMode; // 泊车类型
	unsigned char Command_ParkFuncMode_InterruptStatus; // 通知系统当前状态
	App2Alg_ControlCommand_Parameter ControlCommand_Parameter; // 直进直出状态
}App2Alg_ControlCommand;

/* --------------------------------- 1.超声波距离信息 --------------------------------- */
typedef struct
{
	USSDistance_RxTx Uss_Distance_RxTx;
	USSDistance_Triangle_Value Uss_Distance_Triangle_Value;
}App2Alg_UssInfo;

/* --------------------------------- 3.车位信息 --------------------------------- */
// 车位信息(搜索和泊车阶段)
typedef struct
{
	App2Alg_ParkingPlace_TargetSlot TargetSlotInformation;// 影像目标车位
	App2Alg_ParkingPlace_UssSlot UssSlotInformation; 	  // 超声波车位
	App2Alg_ParkingPlace_AvmSlotTopLowRelocation AVMSlotTopLowRelocationInformation;// 重定位校正(侧方)
	App2Alg_ParkingPlace_AvmSlotLeRiRelocation AVMSlotLeRiRelocationInformation; 	// 重定位校正(垂直&斜向)
	App2Alg_ParkingPlace_AvmSpaceDetect AvmSpaceDetectInformation; // 空间信息
}App2Alg_ParkingPlace;

/* --------------------------------- 控制输入 --------------------------------- */
// 所有信息复合结构体
typedef struct
{
	App2Alg_UssInfo UssInformation; // 超声波距离信息
	App2Alg_Can CanInformation; 	// CAN信息
	App2Alg_ParkingPlace ParkingPlaceInformation; // 车位信息
	App2Alg_ReceiveTimeStamp ReceiverTimeStamp;   // 时间戳信息
	App2Alg_ControlCommand ControlCommandInformation; // 控制命令
	App2Alg_padRealTimeLocation padRealTimeLocation;  // 泛亚结构体
}App2Alg;

#endif
