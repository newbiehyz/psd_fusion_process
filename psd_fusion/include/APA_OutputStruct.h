
#ifndef _APA_OUTPUT_STRUCT_
#define _APA_OUTPUT_STRUCT_
#include <vector>
using namespace std;
#include "internalSOCv1_2.hpp"
#define RECTPointNum			4		//车位框点数
#define APS_DEFAULT_DIST		99999	//APS算法中默认距离值
#define APS_INVAILD_LABEL		-999	//APS算法中无效LABEL

namespace APA_SPACE{

typedef enum 
{
	PARALLEL_PARKING = 0,	//水平车位
	VERTICAL_PARKING,		//垂直车位
	DIAGONAL_PARKING_1,		//斜列式60
	DIAGONAL_PARKING_2,		//斜列式120
	DIAGONAL_PARKING_3,		//斜列式45
	DIAGONAL_PARKING_4,		//斜列式135
	DIAGONAL_PARKING_5,		//直角斜列式
	VERTICAL_PARKING_P , 	//97度强平行垂直车位
	CIRCLE_PARKING_1,		//弯道车位
	CIRCLE_PARKING_2,
	I_PARKING,				//I型车位
	VP_PARKING,				//只检测水平和垂直车位  //2021-05-12 新增
	ALLTYPE_PARKING,		//自动模式0626
}PStype;

#define CIRCLE_PARKING 			APA_SPACE::CIRCLE_PARKING_1
#define DIAGONAL_PARKING 		APA_SPACE::DIAGONAL_PARKING_1
typedef struct SApaPoint_I
{
    int x;
    int y;
	SApaPoint_I() : x(0), y(0) {}
	SApaPoint_I(double x, double y) : x(x), y(y) {}
}SApaPoint_I;

typedef struct SApaPoint_F
{
    float x;
    float y;
	SApaPoint_F() : x(0), y(0) {}
	SApaPoint_F(double x, double y) : x(x), y(y) {}
}SApaPoint_F;

typedef enum
{
	E_SCENE_DEFAULT = 0,//默认，未知状态
	E_SCENE_NORMAL,		//正常场景
	E_SCENE_FLOORTILE	//地砖地
}ERECT_SCENE_TYPE;

enum ERECT_SOD_TYPE //车位sod类型
{
	RECTSOD_DEFAULT = -999,			//默认值
	RECTSOD_Empty = 0,				//空车位
	RECTSOD_Obstacle = 1,			//障碍物车位
	RECTSOD_UNConfirm = -1,			//不确认类型车位
	RECTSOD_INIT = 2,				//初始化车位
	RECTSOD_EXCLUDEINIT = 3,		//算法内部用
	RECTSOD_PARLLE_HAS_ROADEDGE = 4 //水平车位有路沿
};

enum E_RECT_TYPE
{
	E_RECT_VISION = 0,	//视觉车位
	E_RECT_ULTRA,		//超声波车位
	E_RECT_COMPLEX,		//超声波复合车位
	E_RECT_COMPLEX_REC,	//超声波推荐复合车位
	E_RECT_FUSE			//融合车位（视觉和超声波融合后的车位，暂时无用，待定）
};

enum ERECT_EDGE_SOD_TYPE //车位上下空间的SOD类型
{
	E_EDGESOD_DEFAULT = 0,			//默认，上下空间都未知情况
	E_EDGESOD_U_UNKNOW_D_EMPTY,		//上部空间未知，下部分为空
	E_EDGESOD_U_UNKNOW_D_SOD,		//上部空间未知，下部分为SOD
	E_EDGESOD_U_EMPTY_D_UNKNOW,		//上部空间为空，下部分为未知
	E_EDGESOD_U_EMPTY_D_EMPTY,		//上部空间为空，下部分为空
	E_EDGESOD_U_EMPTY_D_SOD,		//上部空间为空，下部分为SOD
	E_EDGESOD_U_SOD_D_UNKNOW,		//上部空间为SOD，下部分为未知
	E_EDGESOD_U_SOD_D_EMPTY,		//上部空间为SOD，下部分为空
	E_EDGESOD_U_SOD_D_SOD			//上部空间为SOD，下部分为SOD
};

enum //车位上下突出空间的SOD类型
{
    E_EXTRUDED_DEFAULT = 0,         //默认，上下空间都未知情况
    E_EXTRUDED_U_UNKNOW_D_EMPTY,    //上部空间未知，下部分为空
    E_EXTRUDED_U_UNKNOW_D_SOD,      //上部空间未知，下部分为SOD
    E_EXTRUDED_U_EMPTY_D_UNKNOW,    //上部空间为空，下部分为未知
    E_EXTRUDED_U_EMPTY_D_EMPTY,     //上部空间为空，下部分为空
    E_EXTRUDED_U_EMPTY_D_SOD,       //上部空间为空，下部分为SOD
    E_EXTRUDED_U_SOD_D_UNKNOW,      //上部空间为SOD，下部分为未知
    E_EXTRUDED_U_SOD_D_EMPTY,       //上部空间为SOD，下部分为空
    E_EXTRUDED_U_SOD_D_SOD          //上部空间为SOD，下部分为SOD
};

enum SOD_IN_SLOT_LOCATION//障碍物在车位内的位置
{
	SOD_LOCATION_DEFAULT = -1, //默认
	SOD_LOCATION_NO = 0, //障碍物不在车位内
	SOD_LOCATION_AB = 1, //障碍物离AB边更近
	SOD_LOCATION_BC = 2, //障碍物离BC边更近
	SOD_LOCATION_CD = 3, //障碍物离CD边更近
	SOD_LOCATION_DA = 4, //障碍物离DA边更近
};

struct LineEquParam //直线方程系数 ax+by+c=0
{
	float a;
	float b;
	float c;
	int linetype;	 //线的类型 -1:未检测到  1:车道线  0:车位线
	float confidents;//巡航输出使用置信度
};

struct SApaPSRect
{
	SApaPoint_I pt[RECTPointNum];//坐标点，单位:mm
	int PStype;					 //车位形状类型，对应枚举PStype，0垂直，1水平，2斜列
	int label;					 //车位标号，唯一值
	int iSodType;				 //车位内障碍物状态，对应枚举ERECT_SOD_TYPE
	int iRectType;				 //车位类型，对应枚举E_RECT_TYPE
	int iOtherSideSOD; 			 //对象空间是否有障碍物，-1未知情况，1有障碍物，0无障碍物
	bool isDriveSpaceEnough;	 //泊车行驶空间是否足够，1为空间足够；0为不足
	int iDownSlotSOD;			 //车位上、下方障碍物存在情况，对应枚举ERECT_EDGE_SOD_TYPE
	int level;					 //跟踪值还是检测值 2为检测值，1为跟踪值
	int iMinOtherSideDist;		 //对向泊车空间障碍物最近距离,单位mm，默认99999,车位坐标距离障碍物的距离
	// unsigned char iRoadEdgeType; //视觉水平车位底部障碍物类型，0没有障碍物，1路沿，2大墙，99other
	int iRoadEdgeDist;			 //水平车位AB点距离路沿距离，单位mm，默认99999
	int iSceneType;				 //场景类型，对应枚举ERECT_SCENE_TYPE
	int iHasGL;					 //是否有地锁，-1为默认值，1为有地锁，0为没地锁
	int flag;					 //desay算法内部用，
    int iExtrudeSOD;			 //车位相邻障碍物突出

	float StopperDistance; 	 	 //限位器到入口边的距离
	int StopperLocation;		 //用于车位内限位器位置，0为NO，1为AB，2为BC，3为CD，4为DA
	int StopperInSlot;			 //车位内是否有限位器
	int StopperCount;			 //车位内有多少限位器
	int StopperID[2];			 //车位内限位器的ID
	int StopperX[2]; 			 //车位内限位器的X坐标
	int StopperY[2];			 //车位内限位器的Y坐标
	int LockInSlot;			 	 //车位内是否有地锁
	int OBSInSlot;			 	 //车位内是否有障碍物
	
	int iMaterial;				 //车位下的地面材质，0其他，1草砖，2机械

	int NotToRelease; 			 //不应该被释放的车位（入口边过窄）,1为不应该被释放，0为被释放

	int ParkInSlot;				 //要泊入的车位，0不是，1是当前泊入的

	bool is_singleframe_calibrated; //是否被单帧校准过

	SApaPSRect()
	{
		iSceneType			= E_SCENE_DEFAULT;
		iHasGL				= -1;
		// iRoadEdgeType       = 99;
		iRoadEdgeDist 		= APS_DEFAULT_DIST;
		iMinOtherSideDist 	= APS_DEFAULT_DIST;
		level 				= 0;
		iDownSlotSOD 		= E_EDGESOD_DEFAULT;
		iSodType 			= RECTSOD_UNConfirm;
		iRectType 			= E_RECT_VISION;
    	iExtrudeSOD         = E_EXTRUDED_DEFAULT;
		PStype				= 1;
		label 				= 0;
		iOtherSideSOD 		= 0;
		isDriveSpaceEnough 	= false;

		StopperCount 		= 0;
		StopperDistance		= 0.0;
		StopperLocation	    = 0;
		StopperInSlot 		= 0;
		LockInSlot			= 0;
		OBSInSlot			= 0;

		iMaterial	 		= 0;
		NotToRelease		= 0;
		
		ParkInSlot			= 0;

		is_singleframe_calibrated = false;

		for(int i = 0; i < RECTPointNum; i++)
		{
			pt[i].x = 0;
			pt[i].y = 0;
		}

		for(int i = 0; i < 2; i++)
		{
			StopperX[i] = 0;
			StopperY[i] = 0;
			StopperID[i] = -1;
		}
	}
};

struct SApaPSInfo
{ 
    vector<SApaPSRect> WorldoutRect;	//表示车位的点集
    unsigned int   AbsoluteDeltaX;		//当前帧获取的dr值（原始的，从UartData2中获取）
    unsigned int   AbsoluteDeltaY;		//当前帧获取的dr值（原始的，从UartData2中获取）
    unsigned short AbsoluteDeltaAngle;	//当前帧获取的dr值（原始的，从UartData2中获取）
	unsigned long long ullFrameId; 		//当前帧的摄像头帧号 //2021-05-12 新增
    //unsigned short UartCount; 			//当前帧的uart count号（原始的，从UartData2中获取） //2021-05-12 删除
    LineEquParam Out_LineParam[4];		//车道线车位线信息
	unsigned long long ulAlgStartTime; 	//算法开始时间  //2021-05-12 新增
    unsigned long long ulAlgEndTime; 	//算法结束时间  //2021-05-12 新增

	App2Alg_padRealTimeLocation padRealTimeLocation;//泛亚结构体，给泊车算法用

};

}

#endif
