
#ifndef APA_INTERNALSOCV1_2_HPP
#define APA_INTERNALSOCV1_2_HPP

// #include "externalSOC.hpp"
/*
typedef unsigned char padImg; 		// 处理与拼接完成准备检测的图像 //zhguoi:no use
typedef unsigned char padProcImg; 	// 处理完未拼接的单张图像
typedef unsigned char padRawImg; 	// 未处理的原始鱼眼单张图像
typedef unsigned char padVdrImg; 	// 处理后的单张图像

#define RAWHEIGHT 960; 	// 原始图像高
#define RAWWIDTH 1280; 	// 原始图像宽
#define PROCHEIGHT 720; // 处理后图像高
#define PROCWIDTH 1280; // 处理后图像宽
#define IMGVDRHEIGHT 0; // 单张拼接图高
#define IMGVDRWIDTH 0; 	// 单张拼接图宽
#define PADDING 60; 	// 中间补0条
#define IMGHEIGHT 448; 	// 单张拼接图高
#define IMGWIDTH 448; 	// 单张拼接图宽
#define NCHANNEL 3; 	// 图像通道数
#define SCALE 0.035; 	// 单个像素相对的距离(m)
*/
#define MAXNSLOTS 10; 	// 单帧最大车位检测数量
#define MAXNFEATURES 30 // 单帧最大车位特征点检测数量

#define RIGHT 0;  // 车位在右侧
#define LEFT 1;	  // 车位在左侧

#define VISION 0; // 纯影像车位
#define USS 1;	  // 纯超声波车位
#define FUSION 2; // 融合车位

#define VERTICAL 0; // 垂直车位
#define ANGLED 1; 	// 斜向车位
#define PARALLEL 2; // 侧方车位

#define NORMAL_ 0; // 通用场景 // modify to fix conflit in opencv
#define SPECIAL 1; // 特殊场景

/* 图像处理和视觉检测 */
// 图像坐标系下的坐标点
typedef struct
{
	unsigned short x; //pixel
	unsigned short y; //pixel
}padPixelPoint;
// 车位特征点定义
typedef struct
{
	padPixelPoint featureLocation;
	unsigned char featureClass; // 车位特征点分类信息 0x00为通用场景 0x01为特殊场景
}padCornerFeature;
// 单帧图片车位特征点定义
typedef struct
{
	unsigned short	 nFeatures; //车位信息点
	padCornerFeature features[MAXNFEATURES]; //车位特征点列表
}padSlotFeature;
// 真实坐标系下的坐标点
typedef struct
{
    int x; //mm
	int y;
    // float x; //mm
	// float y;

}padPoint;
// 真实坐标系下目标车位角点坐标信息(视觉)
typedef struct
{
	padPoint a; // vision: 下入口点坐标
	padPoint b; // 上入口点坐标
	padPoint c; // 上底点坐标
	padPoint d; // 下底点坐标
	padPoint o; // 原点，车位左上角点坐标
	unsigned char slotSide; // 车位的左右方位信息 0x00为右侧车位 0x01为左侧车位
	unsigned char bayType;	// 车位类型 0x00为垂直泊车位(包含斜向车位) 0x01为水平泊车位 0x02为斜向泊车位
	int occupy;
	int material; // 地面材质
}padVisionSlotCoord;

// 重定位检测结果，车辆与车位的偏差 // 可参考对外接口(SOC External中的重定位接口定义)
typedef struct
{
	float leftTheta;  // 车辆与左侧车位线夹角
	float rightTheta; // 车辆与右侧车位线夹角
	int leftDist;	  // 车辆左后角点与左侧车位线距离
	int rightDist;	  // 车辆右后角点与右侧车位线距离
}padSlotRedress;

/* 航位推算(DR)模块 */
// 车辆在真实坐标系下的位姿定义结构体
typedef struct
{
	padPoint coord; // mm
	float yaw;		// 车辆航向角(degree) (-180degree - 180degree)
	unsigned char status;
	unsigned long long timestamps;
}padVehiclePose;

/*
typedef enum 
{
	ERELOCATE_DIST_INVALID = 0,
	ERELOCATE_DIST_X = 1,
	ERELOCATE_DIST_Y = 2,
	ERELOCATE_DIST_X_L = 3,
	ERELOCATE_DIST_X_R = 4,
	ERELOCATE_DIST_Y_L = 5,
	ERELOCATE_DIST_Y_R = 6
} ERELOCATE_DIST_TYPE;

// 泊车阶段综合结构体，输出给HMI和规控算法
struct LocateOUT
{
	// 垂直车位重定位信息，给HMI用
	ERELOCATE_DIST_TYPE eDistType_L;
	ERELOCATE_DIST_TYPE eDistType_R;
	bool isRelocation_L;
    bool isRelocation_R;

	float fTheta_L;
	float fTheta_R;
	float fDist_L;
	float fDist_R;
	// 水平车位重定位信息，给HMI用
	ERELOCATE_DIST_TYPE eDistType_Top;
	ERELOCATE_DIST_TYPE eDistType_Down;
	float fTheta_Top;
	float fTheta_Down;
	float fDist_Top;
	float fDist_Down;

	App2Alg_padRealTimeLocation pdaRealTimeLocation;//泛亚结构体，给泊车算法用
};
*/

/* 状态管理：apaStatus */
/*
typedef struct
{
	padPoint a;	// 下入口点坐标
	padPoint b;	// 上入口点坐标
	padPoint c;	// 上底点坐标
	padPoint d;	// 下底点坐标
	unsigned short slotId; 	// 车位ID
}padTargetSlotCoord; // 真实坐标系下目标车位角点坐标信息(车位相对于车辆后轴中心)
*/
// 当前APA指令和状态
typedef struct
{
	unsigned char parkingStatus;	
}apaStatus;


/* CAN信息 */
typedef struct
{
	// MCU30->A72(bridge)->dds 
	// 50 Hz
	float vehicleSteerWheelAngle; 	// 方向盘转角
	float vehicleWheelAngle; 		// 前轮转角
	unsigned short vehicleTurnLeftSignal; 	// 左转转向灯信号
	unsigned short vehicleTurnRightSignal;  // 右转转向灯信号
	unsigned long long vehicleSteerTimestamp;
	// 100 Hz
	float vehicleFrontLeftWheelSpeed; 	// positive == forward, negative == backward
	float vehicleFrontRightWheelSpeed; 	// positive == forward, negative == backward
	float vehicleRearLeftWheelSpeed; 	// positive == forward, negative == backward
	float vehicleRearRightWheelSpeed; 	// positive == forward, negative == backward
	unsigned short vehicleFrontLeftWheelPulse; 	// 左前轮速脉冲
	unsigned short vehicleFrontRightWheelPulse; // 右前轮速脉冲
	unsigned short vehicleRearLeftWheelPulse; 	// 左后轮速脉冲
	unsigned short vehicleRearRightWheelPulse; 	// 右后轮速脉冲
	unsigned short vehicleSpeed; // 速度
	unsigned long long vehicleWheelSpeedTimestamp;
	// 50 Hz
	float vehicleYawRate; 	// yaw rate
	float vehicleAccX; 		// acceleration x axis
	float vehicleAccY; 		// acceleration y axis
	unsigned long long vehicleYawTimestamp;
	// 10 Hz
	unsigned short vehicleDriverDoor; // 0: no request, 1: close, 2:open
	unsigned short vehiclePsngrDoor;  // 0: no request, 1: close, 2:open
	unsigned short trunkDoor; 		  // 0: no request, 1: close, 2: open
	unsigned short vehicleStandStill; // 0: active, 1: standstill
	unsigned long long vehicleStatusTimestamp;
	signed int gearStatus; // add by krc
}padCanInfo;
#ifdef USE_DESAY_LIB
#include "LocationResp.h"
#else
// 泛亚结构体，输出实时车位和相对于的车辆位姿
typedef struct
{
	unsigned char parkingStatus; // 泊车状态
	unsigned char slotType; 	 // 车位类型
	unsigned short slotWidth; 	 // 车位宽度
	unsigned short slotDepth; 	 // 车位深度
	int x;
	int y;
	float yaw;
	unsigned long long timeStamp;
	unsigned char ressrve;
}App2Alg_padRealTimeLocation;
#endif

#endif
