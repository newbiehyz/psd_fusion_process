#ifndef APA_DEFINE_H
#define APA_DEFINE_H

#include "internalSOCv1_2.hpp"
#include "APA_OutputStruct.h"
#include "Relocate_OutputStruct.h"

#define IMAGE_CACHE_SIZE 5
#define CAMERA_BIRD_LEFT_RIGHT_SHM_FILE_PATH "/camera.bird.left_right"
#define CAMERA_BIRD_LR_LEFT_SHM_FILE_PATH "/camera.bird.left_right.left"
#define CAMERA_BIRD_LR_RIGHT_SHM_FILE_PATH "/camera.bird.left_right.right"
#define CAMERA_BIRD_REAR_SHM_FILE_PATH "/camera.bird.rear"
#define CAMERA_RAW_ALL_SHM_FILE_PATH "/camera.raw.all"

// #define USER_SELECT_SLOT_MAX_NUM 6
using namespace APA_SPACE;
using namespace RELOCATE_SPACE;

typedef APA_SPACE::SApaPoint_I POINT_I;

/* 当前APA指令和状态 */
typedef enum
{
	APA_USER_ACTION_SEARCH = 1,
	APA_USER_ACTION_PARKING,
	APA_USER_ACTION_SELECT_SLOT,
	APA_USER_ACTION_CANCEL,
    APA_USER_ACTION_STOP,
	APA_USER_ACTION_EXIT,
	APA_USER_ACTION_INVALID
}apaUserAction;
typedef enum
{
    APA_STATUS_INIT 		= 0x10,
	APA_STATUS_STANDBY 		= 0x20,
    APA_STATUS_SEARCHING 	= 0x30,
    APA_STATUS_SONAR_PARKING 		= 0x40,
    APA_STATUS_PARKING 		= 0x50,
    APA_STATUS_PARKING_OUT 		= 0x60,
    APA_STATUS_PARKING_SELECT_SLOT = 0x70,

	APA_STATUS_PARKING_SHAKEHANDS = 0x80,
	APA_STATUS_PARKING_SHAKEHANDS_OK = 0x90,
	APA_STATUS_PARKING_SHAKEHANDS_FAIL = 0xA0,

    APA_STATUS_PARKING_PAUSE = 0xB0,
    APA_STATUS_PARKING_FAILURE = 0xC0,
    APA_STATUS_PARKING_SUCCESS = 0xD0,

	APA_STATUS_EXIT,
    APA_STATUS_EXIT_FINISH,
} APA_STATUS;

typedef enum
{
    APA_INNER_STATUS_INIT_ING = 0,
    APA_INNER_STATUS_INIT_OK,
    APA_INNER_STATUS_INIT_FAILED,
    APA_INNER_STATUS_START_ING,
    APA_INNER_STATUS_START_OK,
    APA_INNER_STATUS_START_FAILED,
    APA_INNER_STATUS_STOP_ING,
	APA_INNER_STATUS_STOP_OK,
    APA_INNER_STATUS_STOP_FAILED,
    APA_INNER_STATUS_DESTORY_ING,
	APA_INNER_STATUS_DESTORY_OK,
    APA_INNER_STATUS_DESTORY_FAILED,
} APA_INNER_STATUS;

struct apaCanInfo
{
    padCanInfo canInfo;
    
    //rtk info
    float angleHeading; //航向角
	float anglePitch;   //俯仰角
	float angleRoll;    //横滚角
	double posLat;
	double posLon;
};

struct apaSlotInfo
{
    APA_SPACE::SApaPSRect rectInfo;
    POINT_I sonar_pt[RECTPointNum]; //sonar坐标点,单位:mm
    int detect_frame_count;
    int detect_as_occupy_count;
    bool is_reliable;
    apaSlotInfo()
    : detect_frame_count(0)
    , detect_as_occupy_count(0)
    , is_reliable(false)
    {

    }
};

struct apaSlotListInfo
{
    vector<apaSlotInfo>     WorldoutRect;       //表示车位的点集
    unsigned int            AbsoluteDeltaX;     //当前帧获取的dr值(原始的，从UartData2中获取)
    unsigned int            AbsoluteDeltaY;     //当前帧获取的dr值(原始的，从UartData2中获取)
    unsigned short          AbsoluteDeltaAngle; //当前帧获取的dr值(原始的，从UartData2中获取)
    unsigned short          UartCount;          //当前帧的uart count号(原始的，从UartData2中获取)
    unsigned long long      ullFrameId; 		//当前帧的摄像头帧号 //2021-05-12 新增
	unsigned long long      ulAlgStartTime; 	//算法开始时间      //2021-05-12 新增
    unsigned long long      ulAlgEndTime; 	    //算法结束时间      //2021-05-12 新增
    vector<apaSlotInfo>     slots_in_cur_frame; //for desay,car center coordinate 

    App2Alg_padRealTimeLocation padRealTimeLocation; //泛亚结构体
    
    SApaPSInfo convert2ApaPsInfo()
    {
        SApaPSInfo psinfo;
        psinfo.AbsoluteDeltaAngle = this->AbsoluteDeltaAngle;
        psinfo.AbsoluteDeltaX = this->AbsoluteDeltaX;
        psinfo.AbsoluteDeltaY = this->AbsoluteDeltaY;
        psinfo.padRealTimeLocation = this->padRealTimeLocation;
        psinfo.WorldoutRect.clear();
        for(auto rect : this->WorldoutRect) {
            psinfo.WorldoutRect.push_back(rect.rectInfo);
        }
	    psinfo.ullFrameId = this->ullFrameId; 		    
	    psinfo.ulAlgStartTime = this->ulAlgStartTime; 	
        psinfo.ulAlgEndTime = this->ulAlgEndTime; 	    
        return psinfo;
    }
};

#include <stdint.h>
#define IMG_RAW_HEIGHT 480
#define IMG_RAW_WIDTH 640

// #define VISION_OD_HEIGHT 300
// #define VISION_OD_WIDTH 300
// #define VISION_FS_HEIGHT 384
// #define VISION_FS_WIDTH 768

// #define _USE_704_704_
#ifdef _USE_704_704_
    #define BIRD_VIEW_HEIGHT 704
    #define BIRD_VIEW_WIDTH 704
    #define REAR_BIRD_VIEW_HEIGHT 704 
    #define REAR_BIRD_VIEW_WIDTH 704
#else
    #define BIRD_VIEW_HEIGHT 448 //448
    #define BIRD_VIEW_WIDTH 448 //448
    #define REAR_BIRD_VIEW_HEIGHT 448 //448
    #define REAR_BIRD_VIEW_WIDTH 448 //448
#endif

#define LR_BIRD_PIXECL_2_WORLD (15000.0 / BIRD_VIEW_HEIGHT)  //15000.0
#define REAR_BIRD_PIXECL_2_WORLD (15000.0 / REAR_BIRD_VIEW_WIDTH) //15000.0

/* 原始鱼眼输出 */ //输出鱼眼单色图采用m_*[IMG_RAW_HEIGHT * IMG_RAW_WIDTH]，输出鱼眼三色图采用m_*[IMG_RAW_HEIGHT * IMG_RAW_WIDTH * 3]
typedef struct _ZM_RAW_IMG_4
{	
    uint8_t m_left[IMG_RAW_HEIGHT * IMG_RAW_WIDTH * 3]; //m_left[IMG_RAW_HEIGHT * IMG_RAW_WIDTH * 3];
    uint8_t m_right[IMG_RAW_HEIGHT * IMG_RAW_WIDTH * 3];//m_right[IMG_RAW_HEIGHT * IMG_RAW_WIDTH * 3]
    uint8_t m_front[IMG_RAW_HEIGHT * IMG_RAW_WIDTH * 3];//m_front[IMG_RAW_HEIGHT * IMG_RAW_WIDTH * 3]
    uint8_t m_rear[IMG_RAW_HEIGHT * IMG_RAW_WIDTH * 3]; //m_rear[IMG_RAW_HEIGHT * IMG_RAW_WIDTH * 3]
} ZM_RAW_IMG_4;

typedef struct _ZM_BIRDVIEW_LEFT_RIGHT_IMG 	
{	
    uint8_t m_left_right[BIRD_VIEW_HEIGHT * BIRD_VIEW_WIDTH];

} ZM_BIRDVIEW_LEFT_RIGHT_IMG;

typedef struct _ZM_BIRDVIEW_REAR_IMG 	
{	
    uint8_t m_rear[REAR_BIRD_VIEW_HEIGHT * REAR_BIRD_VIEW_WIDTH];

} ZM_BIRDVIEW_REAR_IMG;

typedef struct {
    float x;
    float y;
}mark_point;

typedef struct {
    mark_point p1; //Entry point
    mark_point p2; //Entry point
    mark_point p3; 
    mark_point p4;
    int occupy;
    int type;     //0:hor 1:per 2:slant
    float confidence;
    float angle;
}ParkingSlot;

typedef struct {
    int32_t box[4];
    int32_t cls;
    float score;
    int16_t num_obj;
}ObjectBox;

typedef struct {
    int x;
    int y;
}PointFs;

typedef struct {
    vector<PointFs> road;
    vector<PointFs> person;
    vector<PointFs> car;
}FreeSpaceOutput;

typedef struct {
    float APAFRS_Distance;
    float APAFR_Distance;
    float APAFRM_Distance;
    float APAFLM_Distance;
    float APAFL_Distance;
    float APAFLS_Distance;
    float APARRS_Distance;
    float APARR_Distance;
    float APARRM_Distance;
    float APARLM_Distance;
    float APARL_Distance;
    float APARLS_Distance;
    float USSlot_Length;
    float USSlot_Depth;
    float USSlot_PtA_X;
    float USSlot_PtA_Y;
    float USSlot_PtB_X;
    float USSlot_PtB_Y;
    float RearSysBumpTrig_PtA_X;
    float RearSysBumpTrig_PtA_Y;
    float FrontSysBumpTrig_PtB_X;
    float FrontSysBumpTrig_PtB_Y;
    unsigned long long timestamp;
}UssInfo;

enum SONAR_DETECTION_MODULE
{
    IAUTO = 1,
    TTE = 2,
};


struct sRelocate_Point2f
{
    float x;
    float y;

    sRelocate_Point2f()
    : x(0.0f)
    , y(0.0f)
    {

    }

    void clear()
    {
        x = 0.0f;
        y = 0.0f;
    }
};
/**
 * data for DigitalMap module
 */
struct sDigitalMap_Line
{
    int iLine_ID;          // Line's ID (line valid: the ID value is increased from "0". line invalid: the ID value is "-1".) note: 0:right line 1:left line 2:entrance line from Relocate module
    float fLine_Confidence;// Line's confidence (value range: 0.0 ~ 1.0)
    /**
     * Line angle
     *
     * note: the coordinate system is not an image coordinate system, but a coordinate system with the center of vehicle body as the origin.
     * The right side of car is the positive direction of the X axis, and the front of car is the positive direction of the Y axis.
     * The angle is the included angle with the Y axis, the right of Y axis is positive, and the left of Y axis is negative.
     */
    float fLine_Angle;               // Line's angle (value range: -90 degree to +90 degree, unit: degree)
    float fLine_Intercept;           // Line's intercept
    // float fLine_LineWidth;        // Line's width(reserved)
    sRelocate_Point2f fLine_StartPt; // Line's start point
    sRelocate_Point2f fLine_EndPt;   // Line's end point

    sDigitalMap_Line()
    : fLine_StartPt()
    , fLine_EndPt()
    , iLine_ID(-1)
    , fLine_Confidence(0.0f)
    , fLine_Angle(0.0f)
    , fLine_Intercept(0.0f)
    {

    }

    void clear()
    {
        iLine_ID = -1;
        fLine_Confidence = 0.0f;
        fLine_Angle = 0.0f;
        fLine_Intercept = 0.0f;
        fLine_StartPt.clear();
        fLine_EndPt.clear();
    }
};

/**
 * Output data of the relocate module
 * @note contains 2 parts data:
 * part 1: "LocateOUT"
 * part 2: "vector Digital Map Data" for DigitalMap module
 */
class PSD_RelocateOUTData
{
public:
    PSD_RelocateOUTData()
    : sLocateOUT()
    , bStatus(false)
    {

    }
    ~PSD_RelocateOUTData(){}

    // Data member
    sDigitalMap_Line m_line_vertical[4][2];	 //目标车位的两根垂线 for DigitalMap module
    sDigitalMap_Line m_line_horizontal[4][2];//目标车位的两根横线 for DigitalMap module

    LocateOUT sLocateOUT;
    bool bStatus;                            // OUT Data valid status. (0: invalid 1: valid)
};

#endif
