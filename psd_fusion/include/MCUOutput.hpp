
#ifndef MCUOUTPUT_H
#define MCUOUTPUT_H

/* --------------------------------- 1.DR信息 --------------------------------- */
// 航位推算
typedef struct
{
	float CoordinateDr_X; 	// 单位mm
	float CoordinateDr_Y; 	// 单位mm
	float CoordinateDr_Yaw; // 单位degree
}Alg2App_Coordinate_DR;

// 航位预测
typedef struct
{
	float PredictLocation_X; 	// 单位mm
	float PredictLocation_Y; 	// 单位mm
	float PredictLocation_Yaw; 	// 单位degree
}Alg2App_Coordinate_PredictLocation;

// DR信息
typedef struct
{
	Alg2App_Coordinate_DR CoordinateDrInfo; // 航位推算
	Alg2App_Coordinate_PredictLocation PredictLocationInformation; // 航位预测
}Alg2App_Coordinate;


/* --------------------------------- 2.HMI显示信息 --------------------------------- */
typedef struct
{
	unsigned char Display_TargetTotalStep; 		 // 总步数
	unsigned char Display_TargetCurrentStep; 	 // 当前处于第几步
	unsigned char Display_TargetStepProgressBar; // 进度（0-100%）
}Alg2App_Display;


/* --------------------------------- 3.控制接口转发 --------------------------------- */
typedef struct
{
	unsigned char Control_TargetGear; 	// 目标挡位
	float Control_TargetSteeringAngle; 	// 目标方向盘转角
	float Control_TargetVehicleSpeed; 	// 目标车速
	float Control_TargetAcceleration; 	// 目标加速度
	unsigned short Control_TargetStopDistance; 	// 目标stop distance
	unsigned char Control_TargetDriveoff; 		// 目标启动
	unsigned char Control_TargetStandstillReq; 	// 目标静止
}Alg2App_Can_Control;


/* --------------------------------- 4.控制命令反馈 --------------------------------- */
typedef struct
{
	unsigned char CommandResp_Error; 				// 算法内部错误
	unsigned char CommandResp_Dr_InitializeStatus; 	// DR初始化状态
	unsigned char CommandResp_Dr_CalculateStatus; 	// DR的计算过程状态
	unsigned char CommandResp_Control_Status; 		// 控制状态反馈
}Alg2App_CommandResp;

/* --------------------------------- 控制输出 --------------------------------- */
// 所有信息复合结构体
typedef struct
{
	Alg2App_Coordinate CoordinateInformation; 	// DR信息
	Alg2App_Display DisplayInformation; 		// HMI显示信息
	Alg2App_Can_Control CanControlInformation; 	// 控制接口转发
	Alg2App_CommandResp CommandRespInformaion; 	// 控制命令反馈
}Alg2App;

#endif
