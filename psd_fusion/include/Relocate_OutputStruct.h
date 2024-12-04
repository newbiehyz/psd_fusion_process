
#ifndef _APA_RELOCATE_OUTPUT_STRUCT_H_
#define _APA_RELOCATE_OUTPUT_STRUCT_H_
#include "APA_OutputStruct.h"
using namespace APA_SPACE;
namespace RELOCATE_SPACE
{

typedef enum 
{
	ERELOCATE_DIST_INVALID = 0, //0，无效值
	ERELOCATE_DIST_X = 1,		//1，给出的是X方向的距离
	ERELOCATE_DIST_Y = 2,		//2，给出的是Y方向的距离
	ERELOCATE_DIST_X_L = 3,		//3，给出的是水平车位短线的距离车外接矩形左边角的X方向距离
	ERELOCATE_DIST_X_R = 4,		//4，给出的是水平车位短线的距离车外接矩形右边角的X方向距离
	ERELOCATE_DIST_Y_L = 5,		//5，给出的是水平车位短线的距离车外接矩形左边角的Y方向距离
	ERELOCATE_DIST_Y_R = 6,		//6，给出的是水平车位短线的距离车外接矩形右边角的Y方向距离
}ERELOCATE_DIST_TYPE;


typedef enum 
{
	ERELOCATE_SOURCE_DEFAULT = 0,	//默认值
	ERELOCATE_SOURCE_LR,			//左右路出的结果
	ERELOCATE_SOURCE_RF,			//前后路出的结果，目前只有后路
	ERELOCATE_SOURCE_FUSE			//融合出的结果
	
}ERELOCATE_RES_SOURCE;
struct LocateOUT
{
	unsigned long long 		ullCameraFrameNo;//图像frame号              //2021-05-12 新增
	unsigned long 			ulAlgStartTime;	 //算法开始时间，单位毫秒	   //2021-05-12 新增
	unsigned long 			ulAlgEndTime;	 //算法结束时间，单位毫秒	   //2021-05-12 新增
	ERELOCATE_DIST_TYPE eDistType_L;	//左侧输出值类型，对应枚举ERELOCATE_DIST_TYPE
	ERELOCATE_DIST_TYPE eDistType_R;	//右侧输出值类型，对应枚举ERELOCATE_DIST_TYPE
	ERELOCATE_DIST_TYPE eDistType_E;	//Entrance 输出值类型，对应枚举ERELOCATE_DIST_TYPE
	float				fTheta_L;		//左侧线跟Y轴的夹角，在Y轴右侧为正
	float				fTheta_R;		//右侧线跟Y轴的夹角，在Y轴右侧为正
	float				fTheta_E;		//Entrance line 跟Y轴的夹角，在Y轴右侧为正
	float				fDist_L;		//车尾左顶点到左边线的距离，单位mm
	float				fDist_R; 		//车尾右顶点到右边线的距离，单位mm
	float				fDist_E; 		//车尾左顶点到Entrance line的距离，单位mm
	APA_SPACE::SApaPoint_I			Point_L_1;
	APA_SPACE::SApaPoint_I			Point_L_2;
	APA_SPACE::SApaPoint_I			Point_R_1;
	APA_SPACE::SApaPoint_I			Point_R_2;
	APA_SPACE::SApaPoint_I			Point_E_1;
	APA_SPACE::SApaPoint_I			Point_E_2;
	int					iResSource_L;	//结果为单边还是后路
	int					iResSource_R;	//结果为单边还是后路

	//20200110,新增水平车位检测上下俩条短横线
	ERELOCATE_DIST_TYPE eDistType_Top;	//值可能为0、3、4、5、6其中之一
	ERELOCATE_DIST_TYPE eDistType_Down;	//值可能为0、3、4、5、6其中之一
	float				fTheta_Top;		//车位上线跟Y轴的夹角，在Y轴右侧为正
	float				fTheta_Down;	//车位下线跟Y轴的夹角，在Y轴右侧为正
	float				fDist_Top;		//车位上线跟Y轴的夹角，在Y轴右侧为正
	float				fDist_Down; 	//车位下线跟Y轴的夹角，在Y轴右侧为正

	APA_SPACE::SApaPoint_I			rectPoint[4];	//根据重定位线重构的坐标
	bool				isRectPtVaild;	//坐标点是否有效

	unsigned int		AbsoluteDeltaX;	//当前帧dr信息
	unsigned int		AbsoluteDeltaY;
	unsigned short		AbsoluteDeltaAngle;
	unsigned short		UartCount;		//uart传输的帧号	
	bool				isOutputBaseCarCenter;//输出距离是否以车中心为中心 //2021-05-12 新增
	LocateOUT()
	{
		ResetAllData();
	}
	void ResetAllData()
	{
		iResSource_L		= ERELOCATE_SOURCE_DEFAULT;
		iResSource_R		= ERELOCATE_SOURCE_DEFAULT;
		eDistType_L			= ERELOCATE_DIST_INVALID;
		eDistType_R			= ERELOCATE_DIST_INVALID;
		fTheta_L			= 0.0f;
		fTheta_R			= 0.0f;
		fDist_L				= 0.0f;
		fDist_R				= 0.0f;

		eDistType_Top		= ERELOCATE_DIST_INVALID;
		eDistType_Down		= ERELOCATE_DIST_INVALID;
		fTheta_Top			= 0.0f;
		fTheta_Down			= 0.0f;
		fDist_Top			= 0.0f;
		fDist_Down			= 0.0f;

		isRectPtVaild		= false;
		AbsoluteDeltaX 		= 0;
		AbsoluteDeltaY 		= 0;
		AbsoluteDeltaAngle 	= 0;
		UartCount			= 0;
		isOutputBaseCarCenter   = 0; //2021-05-12 新增

	}
};

}
#endif
