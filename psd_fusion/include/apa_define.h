#ifndef APA_DEFINE_H
#define APA_DEFINE_H

#include <memory>
#include "Eigen/Core"

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
typedef APA_SPACE::SApaPoint_F POINT_F;

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

static inline bool FloatNumEqual(float l, float r) {
    constexpr float EPSILON = 1e-7;
    return fabsf(l - r) < EPSILON;
}

/**
 * @brief 模型给过来的4个角点可获得的所有信息 *
 */
struct QuadInfo {
    Eigen::Matrix<float, 3, 4> quads;  // 模型给出的4个顶点的坐标
    Eigen::Matrix<float, 3, 4> corners_ego;    // 转移到自车系的坐标
    Eigen::Matrix<float, 3, 4> corners_world;  // 转移到世界系的坐标
    Eigen::Matrix<float, 3, 4> corners_extended;  // 补全并转移到世界系的坐标

    std::array<Eigen::Matrix2f, 4> cov_ego;       // 自车系的协方差
    std::array<Eigen::Matrix2f, 4> cov_world;     // 世界系的协方差
    std::array<Eigen::Matrix2f, 4> cov_extended;  // 补全后的协方差
    std::array<bool, 4> near_edge;                // 是否靠近IPM图边缘

    Eigen::Vector3f center_pixel = Eigen::Vector3f::Zero();
    Eigen::Vector3f center_ego = Eigen::Vector3f::Zero();
    Eigen::Vector3f center_world = Eigen::Vector3f::Zero();

    Eigen::Vector2f long_dir_pixel = Eigen::Vector2f::Zero();
    Eigen::Vector2f long_dir_world = Eigen::Vector2f::Zero();
    Eigen::Vector2f wide_dir_pixel = Eigen::Vector2f::Zero();
    Eigen::Vector2f wide_dir_world = Eigen::Vector2f::Zero();

    float length_world, width_world;
    uint32_t occupy;
    uint32_t material;
};
typedef std::shared_ptr<QuadInfo> QuadInfoPtr;

struct IPMParameters {
    float focal_length = 29.8f;
    // float focal_length = 44.8f;
    float ipm_width = 896.0f;
    float ipm_height = 896.0f;

    float cx_ratio = 0.5f;
    float cy_ratio = 0.5f;

    float edge_thr = 10.0f;

    //限制车位在图像中心的左右
    // // 10m 352x352 (4,6) (2.5,7.5)
    // float min_u = 141.f;  // width dir
    // float max_u = 210.f;
    // float min_v = 87.f;  // height dir
    // float max_v = 264.f;
    // float cam_v = 120.f;

    // // 15m 352x352 (6.5,8.5) (5,10)
    //  float min_u = 153.f;  // width dir
    //  float max_u = 200.f;
    //  float min_v = 118.f;  // height dir
    //  float max_v = 235.f;
    //  float cam_v = 120.f;
     
    // // 20m 896x896 (9,11) (7.5,12.5)
    // float min_u = 158.f;  // width dir
    // float max_u = 193.f;
    // float min_v = 132.f;  // height dir
    // float max_v = 219.f;
    // float cam_v = 120.f;

    // 20m 896x896 (9,11) (7.5,12.5)
    float min_u = 403.f;  // width dir
    float max_u = 494.f;
    float min_v = 336.f;  // height dir
    float max_v = 561.f;
    float cam_v = 120.f;
};

struct ParkingSlotManagerParameters {
    // 车位有效阈值
    float valid_slot_conf_thr = 10.0f;
    // 像素误差与到相机距离的比例系数
    float sigma_1_ratio = 0.05f;
    float sigma_2_ratio = 0.02f;
    // 超出此范围的像素，sigma增加
    float pixel_dist_thr = 100.f;
    float sigma_enlarge_coeff = 50.0f;
    // 保留车位的范围
    float neighborhood_range = 10000.0f;
    // 检查同一车位的范围
    float check_same_slot_range = 3000.0f;
};
static IPMParameters ipmp_;
static ParkingSlotManagerParameters psmp_;

typedef struct ParkingSlotResult {
    Eigen::Vector2f tl, tr, bl, br;
    Eigen::Vector2f ori_tl, ori_tr, ori_bl, ori_br;
    float confidence;
    float width;
    float length;
    Eigen::Vector2f center;
    Eigen::Vector2f wide_direction;
    Eigen::Vector2f long_direction;
    bool is_occupied;
    uint8_t type;
} ParkingSlotResult;
typedef std::shared_ptr<ParkingSlotResult> ParkingSlotResultPtr;

/* Describe approx_box's point */
struct ApproxBoxPoints {
    Eigen::Vector2f p;
    float border_dist = 0.F;
    float point_score = 0.F;
    // clockwise line, with next point
    float line_len = 0.F;
    float line_score = 0.F;
    bool has_border_point = false;
};

struct ParkingSlotRect {
    ParkingSlotRect(int left, int top, int w, int h)
        : l_(left), t_(top), w_(w), h_(h) {
        r_ = l_ + w;
        b_ = t_ + h;
    }
    int area() const { return w_ * h_; }
    int left() const { return l_; }
    int right() const { return r_; }
    int top() const { return t_; }
    int bottom() const { return b_; }

 private:
    int l_, t_, r_, b_, w_, h_;
};

/* Quad describes everything for a parkingslot */
struct ParkingSlotQuad {
    Eigen::Vector2f tl, tr, bl, br;
    Eigen::Vector2f ori_tl, ori_tr, ori_bl, ori_br;
    float confidence;
    uint32_t label;
    bool filtered = false;
    int slot_type = -1;
    int map_slot_type = -1;

    float IOU(const ParkingSlotQuad &rhs) const;
    float IOUBoundingBox(const ParkingSlotQuad &rhs) const;

    ParkingSlotRect BoundingRect() const {
        int left = std::min(tl(0), std::min(tr(0), std::min(bl(0), br(0))));
        int right = std::max(tl(0), std::max(tr(0), std::max(bl(0), br(0))));
        int top = std::min(tl(1), std::min(tr(1), std::min(bl(1), br(1))));
        int bottom = std::max(tl(1), std::max(tr(1), std::max(bl(1), br(1))));
        return ParkingSlotRect(left, top, right - left + 1, bottom - top + 1);
    }

    bool IsValidQuad() const {
        bool invalid =
            (FloatNumEqual(tl(0), tr(0)) && FloatNumEqual(tl(1), tr(1))) ||
            (FloatNumEqual(br(0), tr(0)) && FloatNumEqual(br(1), tr(1))) ||
            (FloatNumEqual(tl(0), bl(0)) && FloatNumEqual(tl(1), bl(1))) ||
            (FloatNumEqual(br(0), bl(0)) && FloatNumEqual(br(1), bl(1)));
        return !invalid;
    }

    // score for each point
    float s_tl, s_tr, s_bl, s_br;
    // attribute for each point
    std::shared_ptr<ApproxBoxPoints> p_tl = nullptr;
    std::shared_ptr<ApproxBoxPoints> p_tr = nullptr;
    std::shared_ptr<ApproxBoxPoints> p_bl = nullptr;
    std::shared_ptr<ApproxBoxPoints> p_br = nullptr;

    // addition info for this parkingslot
    Eigen::Vector2f dir_in;
    Eigen::Vector2f dir_width;
    Eigen::Vector2f dir_length;
    Eigen::Vector2f center;
    bool opp_modify = false;
    bool is_complete = false;
    float width = 0;
    float length = 0;
    float slant_length = 0;
    bool is_visited = false;
    bool valid_slant = false;
};
typedef std::shared_ptr<ParkingSlotQuad> ParkingSlotQuadPtr;

struct ParkingSlotRange {
        ParkingSlotRange() {}
        ParkingSlotRange(float a, float b) { Set(a, b); }

        float low, high;

        inline void Set(float a, float b) {
            low = std::min(a, b);
            high = std::max(a, b);
        }
        inline bool InRange(const float v) { return low < v && v < high; }
};

struct ParkingSlotSizeController {
        // *******10m
        // std::vector<std::pair<float, float>> vp_slot_sizes = {
        //     {70, 175}, {82, 180}, {86, 184}, {90, 188}};
        // std::vector<std::pair<float, float>> v_slot_sizes = {
        //     {70, 175}, {82, 180}, {86, 184}, {90, 188}};
        // std::vector<std::pair<float, float>> p_slot_sizes = {
        //     {82, 210}, {86, 220}, {95, 246}};
        // *******15m
        // std::vector<std::pair<float, float>> vp_slot_sizes = {
        //     {59, 148}, {69, 152}, {72, 156}, {76, 159}};
        // std::vector<std::pair<float, float>> v_slot_sizes = {
        //     {59, 148}, {69, 152}, {72, 156}, {76, 159}};
        // std::vector<std::pair<float, float>> p_slot_sizes = {
        //     {68, 178}, {73, 186}, {81, 209}};
        // *******20m
        std::vector<std::pair<float, float>> vp_slot_sizes = {
            {88, 223}, {104, 230}, {108, 235}, {113, 240}};
        std::vector<std::pair<float, float>> v_slot_sizes = {
            {88, 223}, {104, 230}, {108, 235}, {113, 240}};
        std::vector<std::pair<float, float>> p_slot_sizes = {
            {103, 268}, {108, 280}, {119, 314}};
        std::vector<std::pair<float, float>> slant_slot_size = {{92, 230},
                                                                {110, 240}};
        bool Adjust(float &length,
                    float &width,
                    bool base_on_length,
                    std::vector<std::pair<float, float>> ps_sizes,
                    bool adjust_both = false);
};

struct ParkingSlotParam {
        uint32_t topk = 100;
        float confidence_threshold = 0.25;
        float corner_conf_threshold = 0.5;
        float corner_dis_threshold = 1000;
        float iou_threshold = 0.3;
        uint32_t input_w = 896, input_h = 896;
        uint32_t image_w = 600, image_h = 600;
        // 20m & 896*896 1pixel = 0.02232m
        // 15m & 448*448 1pixel = 0.03348m
        // 10m & 352*352 1pixel = 0.0284m
        // *******20m
        ParkingSlotRange ps_score_range{0.58, 0.9397};
        ParkingSlotRange ps_length_range{200, 405};
        ParkingSlotRange ps_width_range{75, 140};
        ParkingSlotRange ps_width_slant_range{110, 200};
        ParkingSlotRange ps_length_complete_range{230, 408};
        ParkingSlotRange ps_length_slant_complete_range{240, 408};
        ParkingSlotRange ps_length_2_range{200, 240};
        ParkingSlotRange ps_length_slant_2_range{200, 290};
        // *******15m
        // ParkingSlotRange ps_score_range{0.58, 0.9397};
        // ParkingSlotRange ps_length_range{136, 271};
        // ParkingSlotRange ps_width_range{53, 92};
        // ParkingSlotRange ps_width_slant_range{76, 131};
        // ParkingSlotRange ps_length_complete_range{156, 271};
        // ParkingSlotRange ps_length_slant_complete_range{161, 271};
        // ParkingSlotRange ps_length_2_range{136, 156};
        // ParkingSlotRange ps_length_slant_2_range{136, 187};
        // *******10m
        // ParkingSlotRange ps_score_range{0.58, 0.8};  
        // ParkingSlotRange ps_length_range{160, 320};                // 4.54m - 9.0m 
        // ParkingSlotRange ps_width_range{63, 108};                  // 1.78m - 3.0m 
        // ParkingSlotRange ps_width_slant_range{90, 155};            // 2.55m - 4.4m 
        // ParkingSlotRange ps_length_complete_range{184, 320};       // 5.22m - 9.08m 
        // ParkingSlotRange ps_length_slant_complete_range{190, 320}; // 5.40m - 9.08m 
        // ParkingSlotRange ps_length_2_range{160, 184};              // 4.54m - 5.22m 
        // ParkingSlotRange ps_length_slant_2_range{160, 220};        // 4.54m - 6.24m 
        float vertical_threshold = 0.15;
        float ps_ratio = 2.4;
        ParkingSlotSizeController ps_size_controller;

        float point_border_dist_thres = 5;
        float line_border_dist_thres = 5;
        float point_border_dist_complete_h2 = 3;
        float direction_score_thr1 = 0.5F;
        float direction_score_thr2 = 0.5F;
        // *******20m
        float vp_MaxW = 153;
        float vp_MaxH = 292;
        float v_MaxH = 292;
        float p_MaxH = 292;
        float slant_MaxH = 305;
        float p_MinH = 268;
        float v_MinH = 224;
        float slant_MinH = 268;
        // // *******15m
        // float vp_MaxW = 102;
        // float vp_MaxH = 195;
        // float v_MaxH = 195;
        // float p_MaxH = 195;
        // float slant_MaxH = 203;
        // float p_MinH = 178;
        // float v_MinH = 148;
        // float slant_MinH = 178;
        // *******10m
        // float vp_MaxW = 120;    // 3.4m
        // float vp_MaxH = 230;    // 6.5m
        // float v_MaxH = 230;     // 6.5m
        // float p_MaxH = 230;     // 6.5m
        // float slant_MaxH = 240; // 6.8m
        // float p_MinH = 210;     // 5.96m
        // float v_MinH = 175;     // 4.97m
        // float slant_MinH = 210; // 5.96m
        float supplement_corner_dis_threshold = 50.F;
        float slant_cos_up = 0.9397;
        float slant_cos_low = 0.31;
        float slant_cos_para = 0.174;
        float border_point_dis_thr = 5;
        // *******10m
        // ParkingSlotRange car_length_range = {88.F, 264.F}; //{2.5m, 7.5m}
        // ParkingSlotRange car_width_range = {140.F, 213.F}; //{4m, 6m}
        // float point_border_dis_thres_for_score_modify = 2.F;
        // std::vector<Eigen::Vector2f> car_contour = {
        //     {140.F, 88.F}, {213.F, 88.F}, {213.F, 264.F}, {140.F, 264.F}};
        // *******20m
        ParkingSlotRange car_length_range = {110.F, 338.F}; //{2.5m, 7.5m}
        ParkingSlotRange car_width_range = {178.F, 270.F}; //{4m, 6m}
        float point_border_dis_thres_for_score_modify = 2.F;
        std::vector<Eigen::Vector2f> car_contour = {
            {178.F, 110.F}, {270.F, 110.F}, {270.F, 338.F}, {178.F, 338.F}};

        bool post_output_parking_slot = true;
    };

template <typename _Tp>
class PSMask {
 public:
    PSMask() = default;
    PSMask(uint32_t width, uint32_t height, const _Tp value = 0)
        : width(width), height(height) {
        step = width;
        data.resize(height * step, value);
    }
    PSMask(uint32_t width, uint32_t col, const std::vector<_Tp> &vec)
        : width(width), height(height), data(vec) {
        step = width;
        this->data.resize(height * step);
    }

    // Delete unrecognized type
    template <typename T>
    _Tp &At(T x, T y) const = delete;
    template <typename T>
    _Tp At(T x, T y, _Tp safe_v) const = delete;

    _Tp &At(uint32_t x, uint32_t y) { return data[y * step + x]; }
    _Tp At(uint32_t x, uint32_t y, _Tp safe_v) const {
        return x < width && y < height ? data[y * step + x] : safe_v;
    }
    _Tp &At(float x, float y) {
        return At(static_cast<uint32_t>(x + 0.5F),
                  static_cast<uint32_t>(y + 0.5F));
    }
    _Tp At(float x, float y, _Tp safe_v) const {
        return At(static_cast<uint32_t>(x + 0.5F),
                  static_cast<uint32_t>(y + 0.5F), safe_v);
    }

 public:
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t step = 0;
    std::vector<_Tp> data;
};
typedef PSMask<uint8_t> PSMaskU8;

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
    #define BIRD_VIEW_HEIGHT 896 //896
    #define BIRD_VIEW_WIDTH 896 //896
    #define REAR_BIRD_VIEW_HEIGHT 896 //896
    #define REAR_BIRD_VIEW_WIDTH 896 //896
#endif

#define LR_BIRD_PIXECL_2_WORLD (20000.0 / BIRD_VIEW_HEIGHT)  //20000.0
#define REAR_BIRD_PIXECL_2_WORLD (20000.0 / REAR_BIRD_VIEW_WIDTH) //20000.0

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
