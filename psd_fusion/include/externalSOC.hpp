
#ifndef APA_EXTERNALSOC_HPP
#define APA_EXTERNALSOC_HPP

#define RECTPointNum    4

struct UartData1
{
    unsigned int       YawRate;
    unsigned short     VehicleSpeed;
    unsigned char      GearStatus;
    unsigned char      WheelDirection;
    unsigned short     WheelAngle;
    unsigned short     LateralAcce;
    unsigned short     LongitAcce;
    unsigned short     FLWheelSpeedInkph;
    unsigned short     FRwheelSpeedInkph;
    unsigned short     RLwheelSpeedInkph;
    unsigned short     RRwheelSpeedInkph;
    unsigned short     FLWheelSpeedPulse;
    unsigned short     FRWheelSpeedPulse;
    unsigned short     RLWheelSpeedPulse;
    unsigned short     RRWheelSpeedPulse;
    unsigned char      YawRateDirection;
    unsigned char      UartDataValidFlag;
};	
struct UartData2
{
    unsigned int       AbsoluteDeltaX;
    unsigned int       AbsoluteDeltaY;
    unsigned short     AbsoluteDeltaAngle;
    unsigned short     RelativeDeltaX;
    unsigned short     RelativeDeltaY;
    unsigned short     RelativeDeltaAngle;
    unsigned short     LeftPasRelativeAX;
    unsigned short     LeftPasRelativeAY;
    unsigned short     LeftPasRelativeBX;
    unsigned short     LeftPasRelativeBY;
    unsigned short     LeftPasRelativeCX;
    unsigned short     LeftPasRelativeCY;
    unsigned short     LeftPasDepth;
    unsigned char      ResetAbsoluteCoordinate;
    unsigned char      DistanceRange[12];
    unsigned char      BrakePedalSwitch;
    unsigned short     BrakePower;
    unsigned char      HazardStatus;
    unsigned char      WiperStatus;
    unsigned int  Time;
    unsigned char tmp9;
    unsigned char tmp8;
    unsigned char tmp7;
    unsigned char tmp6;	

    unsigned char tmp5;
    unsigned char tmp4;
    unsigned char tmp3;
    unsigned char tmp2;
    unsigned char tmp1;
    unsigned char tmp0;
    unsigned short UartCount;
    unsigned short UartSum;
    unsigned short VsdkCount;
    unsigned short VsdkSum;
    unsigned short   LeftPasRelativeDX;
    unsigned short   LeftPasRelativeDY;
    unsigned short   RightPasRelativeAX;
    unsigned short   RightPasRelativeAY;
    unsigned short   RightPasRelativeBX;
    unsigned short   RightPasRelativeBY;
    unsigned short   RightPasRelativeCX;
    unsigned short   RightPasRelativeCY;
    unsigned short   RightPasRelativeDX;
    unsigned short   RightPasRelativeDY;
    unsigned short   RightPasDepth;
};


#if USE_DESAY_LIB
#include "StaticParamsInterface.h"
#else
typedef struct	
{	
    double car_width;   //车宽	
    double car_length;  //车长	
    double car_height;  //车高	
    double wheel_base;  //轴距	
    double front_track; //前轮轮距	
    double rear_track;  //后轮轮距	
    double front_overhang;   //前悬值 前轮中心到车头的距离	
    double rear_overhang;    //后悬值 后轮中心到车尾的距离	
    double mirror_head_dist; //后视镜到车头的距离	
}CarParams;	
	
typedef struct	
{	
    double image_width;	      //camera buffer width
    double image_height;	  //camera buffer  height
    double distortion_width;  //该值可忽略不需要用
    double distortion_height; //该值可忽略不需要用
    double dx;	              //sensor每个象元的横向尺寸
    double dy;	              //sensor每个象元的纵向尺寸
    double center_x;	//光学中心横向坐标
    double center_y; 	//光学中心纵向坐标
    double scale_x; 	//camera horizon scale
    double scale_y;	    //camera vertical scale
    double EFL;   	    //effective focal length
}CameraParams;

typedef struct	
{	
    double fx;	//横向等效焦距
    double fy;	//纵向等效焦距
    double cx;	//图像中心点横向坐标
    double cy;	//图像中心点纵向坐标
    double k1;	//k1-k4是鱼眼镜头的畸变系数
    double k2;
    double k3;
    double k4;	
    double k5;	
    double k6;
    double front_rvec1; //front_rvec1 - front_rvec3:前相机的外参 - 旋转向量
    double front_rvec2;	
    double front_rvec3;	
    double front_tvec1; //front_tvec1 - front_tvec3:前相机的外参 - 平移向量
    double front_tvec2;
    double front_tvec3;
        
    double rear_rvec1; //rear_rvec1 - rear_rvec3: 后相机的外参 - 旋转向量
    double rear_rvec2;
    double rear_rvec3;
    double rear_tvec1; //rear_tvec1 - rear_tvec3: 后相机的外参 - 平移向量
    double rear_tvec2;	
    double rear_tvec3;
        
    double left_rvec1; //left_rvec1 - left_rvec3: 左相机的外参 - 旋转向量
    double left_rvec2;
    double left_rvec3;
    double left_tvec1; //left_tvec1 - left_tvec3: 左相机的外参 - 平移向量
    double left_tvec2;	
    double left_tvec3;
        
    double right_rvec1;//right_rvec1 - right_rvec3: 右相机的外参 - 旋转向量
    double right_rvec2;
    double right_rvec3;
    double right_tvec1;//right_tvec1 - right_tvec3: 右相机的外参 - 平移向量
    double right_tvec2;
    double right_tvec3;
}CalibrationParams;

typedef struct	
{	
    double fx;	//横向等效焦距
    double fy;	//纵向等效焦距
    double cx;	//图像中心点横向坐标
    double cy;	//图像中心点纵向坐标
    double k1;  //k1-k4是鱼眼镜头的畸变系数
    double k2;	
    double k3;	
    double k4;	
    double k5;	
    double k6;
    double front_rvec1;//front_rvec1 - front_rvec3:前相机的外参 - 旋转向量
    double front_rvec2;
    double front_rvec3;
    double front_tvec1;//front_tvec1 - front_tvec3: 前相机的外参 - 平移向量
    double front_tvec2;
    double front_tvec3;
        
    double rear_rvec1;//rear_rvec1 - rear_rvec3: 后相机的外参 - 旋转向量
    double rear_rvec2;
    double rear_rvec3;
    double rear_tvec1;//rear_tvec1 - rear_tvec3: 后相机的外参 - 平移向量
    double rear_tvec2;
    double rear_tvec3;
        
    double left_rvec1;//left_rvec1 - left_rvec3: 左相机的外参 - 旋转向量
    double left_rvec2;
    double left_rvec3;
    double left_tvec1;//left_tvec1 - left_tvec3: 左相机的外参 - 平移向量
    double left_tvec2;
    double left_tvec3;
        
    double right_rvec1;//right_rvec1 - right_rvec3: 右相机的外参 - 旋转向量
    double right_rvec2;
    double right_rvec3;
    double right_tvec1;//right_tvec1 - right_tvec3: 右相机的外参 - 平移向量
    double right_tvec2;
    double right_tvec3;
}CalibrationParams_6k;	
#endif

#if USE_DESAY_LIB
#include "CameraIn.h"
#else
enum sirfbsoc_pixel_fmt 
{
	SIRFBSOC_PIXEL_FMT_GRAY,
	SIRFBSOC_PIXEL_FMT_RGB888,
	SIRFBSOC_PIXEL_FMT_ARGB888,
	SIRFBSOC_PIXEL_FMT_VYUY,
	SIRFBSOC_PIXEL_FMT_RGB565,
	SIRFBSOC_PIXEL_FMT_NV12,
	SIRFBSOC_PIXEL_FMT_NR,
};
typedef enum _ZM_IMG_LAYOUT
{
	ZM_IMG_LAYOUT_PROGRESSIVE,
	ZM_IMG_LAYOUT_FIELD_INTERLACED,
	ZM_IMG_LAYOUT_LINE_DISORDER,
	ZM_IMG_LAYOUT_MULTI_PLANES,
	ZM_IMG_LAYOUT_NR
} ZM_IMG_LAYOUT, *PZM_IMG_LAYOUT;
#define CHANNEL_MAX 4

typedef struct _ZM_IMG_PROP 
{
	int m_iWidth;
	int m_iHeight;
	int m_iWidth_stride; 

	enum sirfbsoc_pixel_fmt m_ePixel_fmt;
	ZM_IMG_LAYOUT            m_eLayout;
} ZM_IMG_PROP, *PZM_IMG_PROP;

typedef struct _ZM_IMG 
{
	ZM_IMG_PROP m_sProp;
	void*      m_pBuf;
} ZM_IMG, *PZM_IMG;

typedef struct _ZM_RAW_IMG 
{
	ZM_IMG      m_sChannels[CHANNEL_MAX];
	unsigned long long   m_uIndex;
	unsigned long long   m_uTime_us;
} ZM_RAW_IMG, *PZM_RAW_IMG;
#endif

#endif