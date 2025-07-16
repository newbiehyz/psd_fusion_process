#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "state_client.hpp"
#include <iostream>
#include <typeinfo>
#include <float.h>
#include "math.hpp"

// ***************************标定量
#define VEHICLE_LENGTH 5259.9 
#define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7
#define MM_TO_M 1000.0

// ***************************配置文件修改的参数
bool DEBUG = false; 
bool FARAWAY_FILTER = false; //距离范围限制车位释放功能开关
float FARAWAY_SLOTS_LEFT[2] = {-99999.0,0}; 
float FARAWAY_SLOTS_RIGHT[2] = {0,99999.0};
float FARAWAY_SLOTS_REAR = -99999.0;
float FARAWAY_SLOTS_FRONT = 99999.0;
bool ANGEL_FILTER = false;
float ANGEL_FILTER_LIMIT = -99999.0;
bool VCU_TOO_SMALL_FILTER = false;
float VCU_TOO_SMALL = -99999.0;
bool PARALLEL_VECTOR_FILTER = false;
float PARALLEL_VECTOR_LIMIT = -99999.0;
float NARROWSLOT_THRESHOLD = -99999.0;

// ***************************输入的全局变量
int apa_status = 0;
int park_request = 0;
const int search_interrupt = 0; //@TODO VC7 RELEASE
int is_Still;
Loc::App2emap_DR previous_dr_pose;
std::vector<apaSlotInfo> g_singleframe_locked_slots;

// ***************************输出的全局变量
slotfusion fusionslot;
StatusDecFusionInput psd2statemachine;
Sfus::Sfsuion2DecPlan psd2planning;
Sfus::FusionSlotInfovector psd2vcu;
Fsm::FusionSlotInfo2Location psd2location;
APAControlBumpInput psd2control;

int HMI_select_ID = 0; //HMI只发1s。HMI_select是HMI发的ID，
int HMI_temp_ID = 0;  //HMI_temp_ID是存下来的ID
int VCU_select_ID_ON = 0; //用ON获取的VCU发送的ID
int RECOMMEND_ID = 0; //推荐车位的ID（类似于已点击，点泊车立即泊车）
int final_select_ID = 0; //VCU和HMI最终统一的ID
int final_ID = 0; //结合选择、推荐后的最终ID
bool recommend_exist = false; //推荐车位是否已存在
bool already_has_recommend_slot = false;
bool in_release_range_last_frame = false; //车位释放范围连续帧要求
int stable_frame_count = 0;
const int STABLE_THRESHOLD = 3; // 连续帧数要求
int available_slot_flag_to_statemachine = 0; //可用车位数量flag
bool isNarrow = false; // 是否为窄车位
int parkout_flag = 0; //当前是否为泊出
int mirror_fold_flag_ahead = 0; // 从planning拿的原始折叠flag（存在提前）
int mirror_fold_flag = 0; //后视镜是否被折叠（准确值）
static bool target_slot_already_updated_once = false; //目标车位已经被更新过一次
bool target_slot_in_range = false; //目标车位是否在矩形范围内
Sfus::SfusionSlotType slot_type_before_update;
POINT_I search_target_center = {0, 0}; // 第一次的目标车位中心点世界坐标
POINT_I search_target_center_world = {0, 0}; // 第一次的目标车位中心点世界坐标（用于计算）
POINT_I new_target_center = {0, 0}; // 更新的目标车位中心点世界坐标
POINT_I new_target_center_world = {0,0}; // 更新的目标车位中心点世界坐标（用于计算）
const float MAX_SLOT_MOVE_DIST_MM = 1500.0f; // 最大容忍距离
POINT_I world_slot_memory[4]; // ABCD角点

CDT_PSD_FUSION_PROCESS_TEMPLATE(cpsd_fusion_process)

cpsd_fusion_process::cpsd_fusion_process()
{

}

cpsd_fusion_process::~cpsd_fusion_process()
{

}

void load_image_from_csv(const std::string& filename, std::vector<std::vector<int>>& image) {
    std::ifstream file(filename);
    std::string line;
    int row = 0;
    
    if (file.is_open()) {
        while (getline(file, line) && row < image.size()) {
            std::stringstream ss(line);
            std::string value;
            int col = 0;
            while (getline(ss, value, ',')) {
                image[row][col] = stoi(value); // 将字符串转换为整数
                ++col;
            }
            ++row;
        }
        file.close();
        LOGD("Load ipm camera id image success!");
    } else {
        LOGE("Load ipm camera id image false!");
    }
}

tResult cpsd_fusion_process::Init()
{
    LOGW("PSD Process Start Success!");
    // Load Config
    if (!PSDConfigUtils::LoadFromFile("/app/neo/psd_config.json")) {
        LOGD("Load config failed!");
    }

    // 读取ipm相机id图
    ipm_camera_id_image.resize(896, std::vector<int>(896, 0));
    load_image_from_csv("/app/neo/ipm_camera_id.csv", ipm_camera_id_image);

    // part1 初始化
    static int init_flag = 0;
    if(init_flag == 0){
        //updatevisionslots 初始化
        PSD_FusionModuleIFrunable.Initialize();
        init_flag = 1;

        //statemachine 初始化
        psd2statemachine.aps_apaParkType = 0;
        psd2statemachine.aps_apaAvailableSlot = 0;
        psd2statemachine.aps_apaHighlightSlot = 0;
        psd2statemachine.aps_apaNarrowSlot = 0;
        psd2statemachine.aps_apaParkFusionType = 0;
        psd2statemachine.aps_apaParkPlaceNum = 0;

        //planning 初始化
        psd2planning.targetSlot.slotType = Sfus::SLOTTYP_NULL;
        psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_NULL;
        psd2planning.targetSlot.slotCorners.cornerA.x = 0;
        psd2planning.targetSlot.slotCorners.cornerA.y = 0;
        psd2planning.targetSlot.slotCorners.cornerB.x = 0;
        psd2planning.targetSlot.slotCorners.cornerB.y = 0;
        psd2planning.targetSlot.slotCorners.cornerC.x = 0;
        psd2planning.targetSlot.slotCorners.cornerC.y = 0;
        psd2planning.targetSlot.slotCorners.cornerD.x = 0;
        psd2planning.targetSlot.slotCorners.cornerD.y = 0;
    }

    RETURN_NOERROR;
}

tResult cpsd_fusion_process::Term()
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::Start()
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::Stop()
{
    RETURN_NOERROR;
}

void cpsd_fusion_process::ProcessFarawayFilter(int index, Sfus::FusionSlotInfovector& vcu_data, const apaSlotInfo& slot_output)
{
    LOGD("[VCU NOTRELEASE1 range] FARAWAY_FILTER: %d, Rear-Front: [%f, %f], Left: [%f, %f], Right:[%f, %f]",
        FARAWAY_FILTER, FARAWAY_SLOTS_REAR, FARAWAY_SLOTS_FRONT, 
        FARAWAY_SLOTS_LEFT[0], FARAWAY_SLOTS_LEFT[1], FARAWAY_SLOTS_RIGHT[0], FARAWAY_SLOTS_RIGHT[1]);
    
    if (FARAWAY_FILTER) {
        // 车位中心点计算
        float center_x = 0.0f, center_y = 0.0f;
        for (int j = 0; j < 4; ++j) {
            center_x += vcu_data.FusionSlotInfo[index].pt[j].x;
            center_y += vcu_data.FusionSlotInfo[index].pt[j].y;
        }
        center_x /= 4.0f;
        center_y /= 4.0f;
        
        // 判断中心点是否在矩形范围内
        const float FARAWAY_DEADZONE = 0.1; // 死区（0.2m）
        float rear_limit = FARAWAY_SLOTS_REAR + FARAWAY_DEADZONE;
        float front_limit = FARAWAY_SLOTS_FRONT - FARAWAY_DEADZONE;
        float left1 = FARAWAY_SLOTS_LEFT[0] + FARAWAY_DEADZONE;
        float left2 = FARAWAY_SLOTS_LEFT[1] - FARAWAY_DEADZONE;
        float right1 = FARAWAY_SLOTS_RIGHT[0] + FARAWAY_DEADZONE;
        float right2 = FARAWAY_SLOTS_RIGHT[1] - FARAWAY_DEADZONE;
        
        bool out_of_x_range = (center_x <= rear_limit || center_x >= front_limit);
        bool out_of_y_range = !((center_y >= left1 && center_y <= left2) ||
                                (center_y >= right1 && center_y <= right2));
        
        if (slot_output.rectInfo.PStype == 0 || slot_output.rectInfo.PStype == 2) { // 垂直or斜列
            if (out_of_x_range || out_of_y_range) {
                vcu_data.FusionSlotInfo[index].slotStatusType = 4;
            } else {
                if (slot_output.rectInfo.iSodType == 1) {
                    vcu_data.FusionSlotInfo[index].slotStatusType = 4;
                } else {
                    vcu_data.FusionSlotInfo[index].slotStatusType = 3;
                }
            }
        }
        
        LOGD("[VCU NOTRELEASE1 range] ID: %d, center(%.1f,%.1f), x_out: %d, y_out: %d, stable_frame_count: %d, RD occupied: %d, VCU status: %d",
            vcu_data.FusionSlotInfo[index].slotLabel, center_x, center_y,
            out_of_x_range, out_of_y_range, stable_frame_count,
            slot_output.rectInfo.iSodType, vcu_data.FusionSlotInfo[index].slotStatusType);
    }
}

void cpsd_fusion_process::ProcessAngleFilter(int index, Sfus::FusionSlotInfovector& vcu_data, const apaSlotInfo& slot_output)
{
    LOGD("[VCU NOTRELEASE2 anglelimit] ANGLE_FILTER: %d, ANGEL_FILTER_LIMIT:%f", ANGEL_FILTER, ANGEL_FILTER_LIMIT);
    
    if (ANGEL_FILTER) {
        if (slot_output.rectInfo.PStype != 2) {
            float ax = 4.0f, ay = 0.0f; // 向量1 (-4.0)指向(0,0)
            float bx = vcu_data.FusionSlotInfo[index].pt[1].x - vcu_data.FusionSlotInfo[index].pt[0].x;
            float by = vcu_data.FusionSlotInfo[index].pt[1].y - vcu_data.FusionSlotInfo[index].pt[0].y; // 向量2 A指向B
            
            // 夹角计算
            float dot = ax * bx + ay * by;
            float normA = std::sqrt(ax * ax + ay * ay);
            float normB = std::sqrt(bx * bx + by * by);
            
            if (normA * normB >= 1e-6f) {
                float cos_theta = dot / (normA * normB);
                if (cos_theta > 1.0f) cos_theta = 1.0f;
                if (cos_theta < -1.0f) cos_theta = -1.0f;
                float angle_deg = std::acos(cos_theta) * 180.0f / M_PI;
                
                // 死区（5度）
                const float ANGEL_DEADZONE = 5.0;
                if (angle_deg > ANGEL_FILTER_LIMIT + ANGEL_DEADZONE) {
                    vcu_data.FusionSlotInfo[index].slotStatusType = 4;
                }
                
                LOGD("[VCU NOTRELEASE2 anglelimit] ID: %d, angle_deg: %.f", 
                    vcu_data.FusionSlotInfo[index].slotLabel, angle_deg);
            }
        } else {
            LOGD("[VCU NOTRELEASE2 anglelimit] ID: %d, DIAGONAL SLOT NO LIMIT.", 
                vcu_data.FusionSlotInfo[index].slotLabel);
        }
    }
}

void cpsd_fusion_process::ProcessWidthFilter(int index, Sfus::FusionSlotInfovector& vcu_data, const apaSlotInfo& slot_output)
{
    LOGD("[VCU NOTRELEASE3 abnarrow] VCU_TOO_SMALL_FILTER: %d, VCU_TOO_SMALL: %.3f", VCU_TOO_SMALL_FILTER, VCU_TOO_SMALL);
    
    if (VCU_TOO_SMALL_FILTER) {
        float VCU_AB = sqrt(pow(vcu_data.FusionSlotInfo[index].pt[0].x - vcu_data.FusionSlotInfo[index].pt[1].x, 2) +
                           pow(vcu_data.FusionSlotInfo[index].pt[0].y - vcu_data.FusionSlotInfo[index].pt[1].y, 2));
        
        // 死区（0.1m）
        const float TOO_SMALL_DEADZONE = 0.05;
        if (VCU_AB <= VCU_TOO_SMALL - TOO_SMALL_DEADZONE) {
            vcu_data.FusionSlotInfo[index].slotStatusType = 4;
        }
        
        LOGD("[VCU NOTRELEASE3 abnarrow] ID: %d, VCU_AB: %.3f, VCU occupied: %d",
            vcu_data.FusionSlotInfo[index].slotLabel, VCU_AB, vcu_data.FusionSlotInfo[index].slotStatusType);
    }
}

void cpsd_fusion_process::ProcessParallelFilter(int index, Sfus::FusionSlotInfovector& vcu_data, const apaSlotInfo& slot_output)
{
    LOGD("[VCU NOTRELEASE4 parallel] PARALLEL_VECTOR_FILTER: %d, PARALLEL_VECTOR_LIMIT: %f", 
         PARALLEL_VECTOR_FILTER, PARALLEL_VECTOR_LIMIT);
    
    if (PARALLEL_VECTOR_FILTER) {
        if (slot_output.rectInfo.PStype == 1) {
            POINT_F VCU_car_rear_axle_center = { -4.064, 0.0 };

            float vector_carrearaxlecenter2parallelAD = (
                (vcu_data.FusionSlotInfo[index].pt[3].y - vcu_data.FusionSlotInfo[index].pt[0].y) * VCU_car_rear_axle_center.x
              - (vcu_data.FusionSlotInfo[index].pt[3].x - vcu_data.FusionSlotInfo[index].pt[0].x) * VCU_car_rear_axle_center.y
              + vcu_data.FusionSlotInfo[index].pt[3].x * vcu_data.FusionSlotInfo[index].pt[0].y
              - vcu_data.FusionSlotInfo[index].pt[3].y * vcu_data.FusionSlotInfo[index].pt[0].x
            ) / sqrt(
                pow(vcu_data.FusionSlotInfo[index].pt[3].y - vcu_data.FusionSlotInfo[index].pt[0].y, 2)
              + pow(vcu_data.FusionSlotInfo[index].pt[3].x - vcu_data.FusionSlotInfo[index].pt[0].x, 2)
            );
            
            // 死区（0.2m）
            const float PARALLEL_VECTOR_DEADZONE = 0.1;
            if (vcu_data.FusionSlotInfo[index].pt[0].y >= 0) { // 右侧
                if (vector_carrearaxlecenter2parallelAD <= PARALLEL_VECTOR_LIMIT - PARALLEL_VECTOR_DEADZONE) { // -0.8
                    vcu_data.FusionSlotInfo[index].slotStatusType = 4;
                }
            } else {
                if (vector_carrearaxlecenter2parallelAD >= PARALLEL_VECTOR_LIMIT - PARALLEL_VECTOR_DEADZONE) { // -0.8
                    vcu_data.FusionSlotInfo[index].slotStatusType = 4;
                }
            }
            
            LOGD("[VCU NOTRELEASE4 parallel] PARALLEL VECTOR: %f, ID: %d, set to %d", 
                 vector_carrearaxlecenter2parallelAD, vcu_data.FusionSlotInfo[index].slotLabel, 
                 vcu_data.FusionSlotInfo[index].slotStatusType);
        } else {
            LOGD("[VCU NOTRELEASE4 parallel] NOT PARALLEL SLOT.");
        }
    }
}

void cpsd_fusion_process::SetObstacleProperties(int index, Sfus::FusionSlotInfovector& vcu_data, const apaSlotInfo& slot_output)
{
    // 障碍物属性
    vcu_data.FusionSlotInfo[index].stopperInSlot = slot_output.rectInfo.StopperInSlot;
    
    // 地锁
    vcu_data.FusionSlotInfo[index].lockInSlot = slot_output.rectInfo.LockInSlot;
    if (slot_output.rectInfo.iSodType == 1 && slot_output.rectInfo.LockInSlot == 1) {
        vcu_data.FusionSlotInfo[index].slotInnerObType = 3;
        vcu_data.FusionSlotInfo[index].lockLocation = 3;
    }
    
    // 锥桶/禁停牌
    if (slot_output.rectInfo.iSodType == 1 && slot_output.rectInfo.OBSInSlot == 1) {
        vcu_data.FusionSlotInfo[index].slotInnerObType = 2;
    }
    
    // 车
    if (slot_output.rectInfo.iSodType == 1 && slot_output.rectInfo.OBSInSlot != 1 && 
        slot_output.rectInfo.LockInSlot != 1 && slot_output.rectInfo.StopperInSlot != 1) {
        vcu_data.FusionSlotInfo[index].slotInnerObType = 1;
    }
}

void cpsd_fusion_process::ProcessRecommendationLogic(apaSlotInfo& selected_slot_in_world)
{
    // 认为自车的位置
    POINT_F VCU_car_pose = { -4.0, 0.0 };

    // Step 0: 清除旧的推荐信息
    for (int i = 0; i < psd2vcu.slotNum; ++i) {
        if (psd2vcu.FusionSlotInfo[i].slotStatusType == 7) {
            psd2vcu.FusionSlotInfo[i].slotStatusType = 3;
        }
        psd2vcu.FusionSlotInfo[i].displayLabel = 0;
    }

    std::vector<Sfus::FusionSlotInfo> vcu_available_slots; // 找出available车位
    std::vector<Sfus::FusionSlotInfo> cloest_slots; // 从psd2vcu拿到,找出available里的closet车位
    
    for (int icnt = 0; icnt < psd2vcu.slotNum; ++icnt) {
        if (psd2vcu.FusionSlotInfo[icnt].slotStatusType == 3) {
            Sfus::FusionSlotInfo vcu_slot;
            for (int jcnt = 0; jcnt < 4; ++jcnt) {
                vcu_slot.pt[jcnt].x = psd2vcu.FusionSlotInfo[icnt].pt[jcnt].x;
                vcu_slot.pt[jcnt].y = psd2vcu.FusionSlotInfo[icnt].pt[jcnt].y;
            }
            vcu_slot.slotLabel = psd2vcu.FusionSlotInfo[icnt].slotLabel;
            vcu_slot.slotStatusType = psd2vcu.FusionSlotInfo[icnt].slotStatusType;
            vcu_slot.slotType = psd2vcu.FusionSlotInfo[icnt].slotType;
            vcu_available_slots.push_back(vcu_slot);
        }
    }
    
    LOGD("vcu_available_slots size: %d", vcu_available_slots.size());
    if (vcu_available_slots.size() > 0) {
        available_slot_flag_to_statemachine = 1;
    } else {
        available_slot_flag_to_statemachine = 0;
    }
    
    cloest_slots = math::findClosesParkingSpots(VCU_car_pose, vcu_available_slots, 10); // 距离排序后的slots
    LOGD("cloest_slots size: %d", cloest_slots.size());

    if (final_select_ID == 0 && is_Still) { // 状态1：当没有点选ID且静止，使用推荐ID
        LOGD("RECOMMEND1: still, Start Recommend!");
        const int max_recommend_num = 3; // display设置为1，2，3

        // Step 1：设置 cloest_slots[0] 对应 slot 的 slotStatusType 为 7
        if (!cloest_slots.empty()) {
            int targetLabel = cloest_slots[0].slotLabel;
            RECOMMEND_ID = targetLabel;
            for (int i = 0; i < psd2vcu.slotNum; ++i) {
                if (psd2vcu.FusionSlotInfo[i].slotLabel == targetLabel) {
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 7;
                    break; // 只设置第一个推荐车位
                }
            }
        }
        
        // Step 2：设置推荐车位的 displayLabel 从 1 到 max_recommend_num
        for (int idx = 1; idx <= max_recommend_num && idx < cloest_slots.size(); ++idx) {
            int targetLabel = cloest_slots[idx].slotLabel;
            for (int i = 0; i < psd2vcu.slotNum; ++i) {
                if (psd2vcu.FusionSlotInfo[i].slotLabel == targetLabel) {
                    psd2vcu.FusionSlotInfo[i].displayLabel = idx; // 从1开始编号
                    break;
                }
            }
        }
        
        // 推荐车位作为final_ID
        final_ID = PSDSelectionLogic::RecommendSelectID(final_select_ID, RECOMMEND_ID);
    } else if (final_select_ID == 0 && !is_Still) { // 状态2：当没有点选ID且运动，保留RD原状态
        LOGD("RECOMMEND2: not still, NO Recommend!");
        RECOMMEND_ID = 0;
        final_select_ID = 0;
        final_ID = 0;
        recommend_exist = false;
        already_has_recommend_slot = false;
        
        for (int i = 0; i < slotlist_size; i++) {
            psd2vcu.FusionSlotInfo[i].displayLabel = 0;
            if (psd2vcu.FusionSlotInfo[i].slotStatusType == 4) { // 占用的保持占用
                psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
            } else if (psd2vcu.FusionSlotInfo[i].slotStatusType == 6) { // unavailable的保持unavailable
                psd2vcu.FusionSlotInfo[i].slotStatusType = 6;
            } else { // 剩下的回到available
                psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 设置为AVAILABLE状态
            }
        }
        memset(&psd2statemachine, 0, sizeof(StatusDecFusionInput));
    } else if (final_select_ID != 0 && is_Still) { // 状态3：当有点选车位且静止，使用点选ID
        LOGD("RECOMMEND3: still, Select!");
        RECOMMEND_ID = 0;
        final_ID = final_select_ID;
        already_has_recommend_slot = false;
        
        for (int i = 0; i < slotlist_size; i++) {
            psd2vcu.FusionSlotInfo[i].displayLabel = 0;
            // 找到目标车位ID 且 非占用
            if (psd2vcu.FusionSlotInfo[i].slotLabel == final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType != 4) {
                psd2vcu.FusionSlotInfo[i].slotStatusType = 5; // 设置为SELECTED状态
            }
            // 找到目标车位ID 且 占用
            if (psd2vcu.FusionSlotInfo[i].slotLabel == final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType == 4) {
                psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
            }
            // 剩下的非选中车位，占用的保持占用
            else if (psd2vcu.FusionSlotInfo[i].slotLabel != final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType == 4) {
                psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
            }
            // 剩下的非选中车位，unavailable的保持unavailable
            else if (psd2vcu.FusionSlotInfo[i].slotLabel != final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType == 6) {
                psd2vcu.FusionSlotInfo[i].slotStatusType = 6; // 设置为unavailable状态
            }
            // 剩下的非选中车位，不占用,不unavailable的回到available
            else if (psd2vcu.FusionSlotInfo[i].slotLabel != final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType != 4) {
                psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 设置为AVAILABLE状态
            }
        }
    } else { // 状态4：当有点选车位且运动，清除所有ID
        LOGD("RECOMMEND4: no still, no recommend, no select");
        HMI_temp_ID = 0;
        HMI_select_ID = 0;
        VCU_select_ID_ON = 0;
        final_select_ID = 0;
        RECOMMEND_ID = 0;
        final_ID = 0;
        recommend_exist = false;
        already_has_recommend_slot = false;
        
        for (int i = 0; i < slotlist_size; i++) {
            psd2vcu.FusionSlotInfo[i].displayLabel = 0;
            if (psd2vcu.FusionSlotInfo[i].slotStatusType == 4) { // 占用的保持占用
                psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
            } else if (psd2vcu.FusionSlotInfo[i].slotStatusType == 6) { // unavailable的保持unavailable
                psd2vcu.FusionSlotInfo[i].slotStatusType = 6;
            } else { // 不占用的回到available
                psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 设置为AVAILABLE状态
            }
        }
        memset(&psd2statemachine, 0, sizeof(StatusDecFusionInput));
        memset(&psd2planning.targetSlot, 0, sizeof(Sfus::SfusionSlots));
    }

    // final_ID 已记忆，每次清零全列表，并mark此车位。必须search时打标记
    PSD_FusionModuleIFrunable.markParkInSlot(outputSlot_FUSED, final_ID);
    LOGD("After markParkInSlot FUSIONSLOTS:");
    PSDConfigUtils::LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");

    // 设置 slotSelectedFlag
    int selected_label = -1;
    for (const auto& slot : outputSlot_FUSED.slots_in_cur_frame) {
        if (slot.rectInfo.ParkInSlot == 1) {
            selected_label = slot.rectInfo.label;
            break;
        }
    }
    
    for (int i = 0; i < psd2vcu.slotNum; ++i) {
        if (psd2vcu.FusionSlotInfo[i].slotLabel == selected_label) {
            psd2vcu.FusionSlotInfo[i].slotSelectedFlag = 1;
        } else {
            psd2vcu.FusionSlotInfo[i].slotSelectedFlag = 0;
        }
    }

    // 目标车位记忆
    // 遍历 outputSlot_FUSED.WorldoutRect 找到 final_ID 对应的车位
    for (const auto& slot : outputSlot_FUSED.WorldoutRect) {
        if (slot.rectInfo.label == final_ID) {
            selected_slot_in_world = slot;
            break;
        }
    }

    // selected_slot_in_world 为世界坐标下的目标车位
    LOGD("[TARGET SLOT WORLD] final_ID: %d. (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)",
        final_ID,
        selected_slot_in_world.rectInfo.pt[0].x, selected_slot_in_world.rectInfo.pt[0].y,
        selected_slot_in_world.rectInfo.pt[1].x, selected_slot_in_world.rectInfo.pt[1].y,
        selected_slot_in_world.rectInfo.pt[2].x, selected_slot_in_world.rectInfo.pt[2].y,
        selected_slot_in_world.rectInfo.pt[3].x, selected_slot_in_world.rectInfo.pt[3].y);
}

void cpsd_fusion_process::ProcessNonSearchVCUDisplay(uint64_t current1970_ms)
{
    // 泊入过程中显示所有车位
    LOGD("VCU display For NON-SEARCH");
    memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector));
    psd2vcu.slotNum = slotlist_size;
    
    if (psd2vcu.slotNum > 0) {
        int i = 0;
        LOGD("PSD2VCU apa_status: %d, outputslot_fused size: %d", apa_status, outputSlot_FUSED.slots_in_cur_frame.size());
        
        // 恢复目标车位标记
        PSD_FusionModuleIFrunable.restoreSelectedSlot(outputSlot_FUSED);
        LOGD("After restoreSelectedSlot FUSIONSLOTS (ParkInSlot REQUIRED):");
        PSDConfigUtils::LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");
        
        for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame) {
            if (i >= slotlist_size || i >= 50) {
                LOGD("die in VCU and size is:", slotlist_size);
                break;
            }

            psd2vcu.FusionSlotInfo[i].slotLabel = psd_m_output.rectInfo.label; // ID
            psd2vcu.FusionSlotInfo[i].displayLabel = 0;
            
            // ABCD顺序调整为VCU专用顺序
            // 左侧
            if (psd_m_output.rectInfo.pt[0].x <= 0 || psd_m_output.rectInfo.pt[1].x <= 0 || psd_m_output.rectInfo.pt[2].x < 0) {
                psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[0].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[0].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[1].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[1].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[1].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[2].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[2].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[3].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[3].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
            }
            // 右侧
            else {
                psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[0].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[0].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[1].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[1].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[1].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[2].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[2].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[3].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[3].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
            }

            // 状态判断
            if (psd_m_output.rectInfo.ParkInSlot == 1) {
                psd2vcu.FusionSlotInfo[i].slotStatusType = 5;
                psd2vcu.FusionSlotInfo[i].slotSelectedFlag = 1;
            } else {
                psd2vcu.FusionSlotInfo[i].slotSelectedFlag = 0;
                if (psd_m_output.rectInfo.iSodType == 1) {
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                } else {
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                }
            }

            // VCU显示障碍物
            psd2vcu.FusionSlotInfo[i].stopperInSlot = psd_m_output.rectInfo.StopperInSlot;
            // 地锁
            psd2vcu.FusionSlotInfo[i].lockInSlot = psd_m_output.rectInfo.LockInSlot;
            if (psd_m_output.rectInfo.iSodType == 1 && psd_m_output.rectInfo.LockInSlot == 1) {
                psd2vcu.FusionSlotInfo[i].slotInnerObType = 3;
                psd2vcu.FusionSlotInfo[i].lockLocation = 3;
            }
            // 锥桶/禁停牌
            if (psd_m_output.rectInfo.iSodType == 1 && psd_m_output.rectInfo.OBSInSlot == 1) {
                psd2vcu.FusionSlotInfo[i].slotInnerObType = 2;
            }
            // 车
            if (psd_m_output.rectInfo.iSodType == 1 && psd_m_output.rectInfo.OBSInSlot != 1 && 
                psd_m_output.rectInfo.LockInSlot != 1 && psd_m_output.rectInfo.StopperInSlot != 1) {
                psd2vcu.FusionSlotInfo[i].slotInnerObType = 1;
            }
            
            psd2vcu.FusionSlotInfo[i].backInAvailableFlag = 1;
            psd2vcu.FusionSlotInfo[i].parkInHeadInSoftButtonCurrentValue = 1;
            psd2vcu.FusionSlotInfo[i].timeStamp = (current1970_ms >= 0) ? static_cast<uint64_t>(current1970_ms) : 0;
            i++;
        }
    }
    
    for (int i = 0; i < slotlist_size; i++) {
        LOGD("[PSD2VCUSLOTLIST] IN GUIDANCE, Slot#%d, type: %d, Selected: %d, (%f,%f) (%f,%f) (%f,%f) (%f,%f), timestamp: %llu",
            psd2vcu.FusionSlotInfo[i].slotLabel,
            psd2vcu.FusionSlotInfo[i].slotStatusType,
            psd2vcu.FusionSlotInfo[i].slotSelectedFlag,
            psd2vcu.FusionSlotInfo[i].pt[0].x, psd2vcu.FusionSlotInfo[i].pt[0].y,
            psd2vcu.FusionSlotInfo[i].pt[1].x, psd2vcu.FusionSlotInfo[i].pt[1].y,
            psd2vcu.FusionSlotInfo[i].pt[2].x, psd2vcu.FusionSlotInfo[i].pt[2].y,
            psd2vcu.FusionSlotInfo[i].pt[3].x, psd2vcu.FusionSlotInfo[i].pt[3].y,
            psd2vcu.FusionSlotInfo[i].timeStamp);
    }
    
    EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
}

void cpsd_fusion_process::ProcessPSD2VCU(int apa_status, uint64_t current1970_ms, apaSlotInfo& selected_slot_in_world)
{
    if (apa_status == 2) {
        LOGD("VCU display for SEARCH");

        memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector)); //displayLabel 会被重置
        psd2vcu.slotNum = slotlist_size;

        if (psd2vcu.slotNum > 0) {
            int i = 0;
            LOGD("PSD2VCU apa_status: %d, outputslot_fused size: %d", apa_status, outputSlot_FUSED.slots_in_cur_frame.size());
            
            // ================= 匹配显示 =================
            for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame) {
                if (i >= slotlist_size || i >= 50) {
                    LOGD("die in VCU and size is:", slotlist_size);
                    break;
                }
                
                psd2vcu.FusionSlotInfo[i].slotLabel = psd_m_output.rectInfo.label; //ID
                
                // 角点转换
                psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[0].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[0].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[1].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[1].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[1].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[2].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[2].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[3].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[3].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[3].z = 0;

                // 状态判断
                if (psd_m_output.rectInfo.iSodType == 1) {
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                } else if (psd_m_output.rectInfo.iSodType != 1 && psd_m_output.rectInfo.label == RECOMMEND_ID) {
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 7;
                } else if (psd_m_output.rectInfo.iSodType != 1 && psd_m_output.rectInfo.label != RECOMMEND_ID) {
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                }
                
                //Unavailable 车位
                if (psd_m_output.rectInfo.NotToRelease == 1) {
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 6;
                }

                // ***************************车位不释放策略***************************
                // 1. 距离范围限制
                ProcessFarawayFilter(i, psd2vcu, psd_m_output);
                
                // 2. 角度限制
                ProcessAngleFilter(i, psd2vcu, psd_m_output);
                
                // 3. 车位宽度限制
                ProcessWidthFilter(i, psd2vcu, psd_m_output);
                
                // 4. 平行车位距离限制
                ProcessParallelFilter(i, psd2vcu, psd_m_output);

                // 障碍物属性设置
                SetObstacleProperties(i, psd2vcu, psd_m_output);
                
                psd2vcu.FusionSlotInfo[i].backInAvailableFlag = 1;
                psd2vcu.FusionSlotInfo[i].parkInHeadInSoftButtonCurrentValue = 1;
                psd2vcu.FusionSlotInfo[i].timeStamp = (current1970_ms >= 0) ? static_cast<uint64_t>(current1970_ms) : 0;
                i++;
            }

            // ================= 推荐逻辑 =================
            ProcessRecommendationLogic(selected_slot_in_world);
        }

        // 输出调试信息
        for (int icnt = 0; icnt < slotlist_size; icnt++) {
            LOGD("[PSD2VCUSLOTLIST] apa_status: %d, slotsize: %d, TYPE: %d, STATUS: %d, ID: %d, displayID: %d, SelectedFlag: %d (%f,%f) (%f,%f) (%f,%f) (%f,%f), timestamp: %llu",
                apa_status, slotlist_size,
                psd2vcu.FusionSlotInfo[icnt].slotType,
                psd2vcu.FusionSlotInfo[icnt].slotStatusType,
                psd2vcu.FusionSlotInfo[icnt].slotLabel,
                psd2vcu.FusionSlotInfo[icnt].displayLabel,
                psd2vcu.FusionSlotInfo[icnt].slotSelectedFlag,
                psd2vcu.FusionSlotInfo[icnt].pt[0].x, psd2vcu.FusionSlotInfo[icnt].pt[0].y,
                psd2vcu.FusionSlotInfo[icnt].pt[1].x, psd2vcu.FusionSlotInfo[icnt].pt[1].y,
                psd2vcu.FusionSlotInfo[icnt].pt[2].x, psd2vcu.FusionSlotInfo[icnt].pt[2].y,
                psd2vcu.FusionSlotInfo[icnt].pt[3].x, psd2vcu.FusionSlotInfo[icnt].pt[3].y,
                psd2vcu.FusionSlotInfo[icnt].timeStamp);
        }

        if (apa_status != 1) {
            LOGD("[RECOMMENDSELECTID] HMI %d, VCU %d, final select %d, recommend: %d, final_ID %d", 
                HMI_temp_ID, VCU_select_ID_ON, final_select_ID, RECOMMEND_ID, final_ID);
            EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
        }
    } else {
        // 泊入过程中显示所有车位
        ProcessNonSearchVCUDisplay(current1970_ms);
    }
}


void cpsd_fusion_process::ProcessPSD2APAHandle(uint64_t current1970_ms)
{
    //***********************************APAHANDLE 发送车位列表
    psd2location.slotNum = slotlist_size;
    if (psd2location.slotNum > 0) {
        int j = 0;

        for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame) {
            if (j >= slotlist_size || j >= 50) {
                std::cout << "die in APAhandle and size is:" << slotlist_size << std::endl;
                break;
            }
            
            psd2location.fusionSlotInfo[j].slotLabel = psd_m_output.rectInfo.label; //ID
            psd2location.fusionSlotInfo[j].slotType = slottype_rd2vcu(psd_m_output.rectInfo.PStype);
            
            if (psd_m_output.rectInfo.iMaterial == 1) {
                psd2location.fusionSlotInfo[j].fusionSlotType = 3;
            } else {
                if (psd_m_output.rectInfo.label >= 1000 && psd_m_output.rectInfo.label < 10000) {
                    psd2location.fusionSlotInfo[j].fusionSlotType = 0;
                } else {
                    psd2location.fusionSlotInfo[j].fusionSlotType = 1;
                }
            }
            
            psd2location.fusionSlotInfo[j].pt[0].x = psd_m_output.rectInfo.pt[0].x;
            psd2location.fusionSlotInfo[j].pt[0].y = psd_m_output.rectInfo.pt[0].y;
            psd2location.fusionSlotInfo[j].pt[1].x = psd_m_output.rectInfo.pt[1].x;
            psd2location.fusionSlotInfo[j].pt[1].y = psd_m_output.rectInfo.pt[1].y;
            psd2location.fusionSlotInfo[j].pt[2].x = psd_m_output.rectInfo.pt[2].x;
            psd2location.fusionSlotInfo[j].pt[2].y = psd_m_output.rectInfo.pt[2].y;
            psd2location.fusionSlotInfo[j].pt[3].x = psd_m_output.rectInfo.pt[3].x;
            psd2location.fusionSlotInfo[j].pt[3].y = psd_m_output.rectInfo.pt[3].y;

            //后视镜折叠状态
            if (j == 0) {
                psd2location.fusionSlotInfo[j].displayLabel = mirror_fold_flag;
            } else {
                psd2location.fusionSlotInfo[j].displayLabel = 0;
            }

            LOGD("[PSD2APAHANDLE] TOTAL SLOT NUM: %d, Slot#%d, slottype: %d, fusionslottype: %d, mirrorfold: %d (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                psd2location.slotNum,
                psd2location.fusionSlotInfo[j].slotLabel,
                psd2location.fusionSlotInfo[j].slotType,
                psd2location.fusionSlotInfo[j].fusionSlotType,
                psd2location.fusionSlotInfo[j].displayLabel,
                psd2location.fusionSlotInfo[j].pt[0].x,
                psd2location.fusionSlotInfo[j].pt[0].y,
                psd2location.fusionSlotInfo[j].pt[1].x,
                psd2location.fusionSlotInfo[j].pt[1].y,
                psd2location.fusionSlotInfo[j].pt[2].x,
                psd2location.fusionSlotInfo[j].pt[2].y,
                psd2location.fusionSlotInfo[j].pt[3].x,
                psd2location.fusionSlotInfo[j].pt[3].y);
            j++;
        }
        
        if (parkout_flag != 1) {
            EMC_psd_fusion_process_SetFieldFusionSlotInfo2Location(psd2location);
        }
    }
}

void cpsd_fusion_process::ProcessPSD2APAHandleTargetID()
{
    //***********************************APAHANDLE 发送目标车位ID
    Fsm::Slotlabel psd2apahandel_targetID;
    if (final_ID) {
        psd2apahandel_targetID.targetSlotLabel = final_ID;
        if (parkout_flag != 1) {
            EMC_psd_fusion_process_SetFieldSlotlabel(psd2apahandel_targetID);
        }
        LOGD("[PSD2APAHANDLE] target slot id: %d", final_ID);
    }
}

void cpsd_fusion_process::ProcessPSD2Planning(uint64_t current1970_ms)
{
    //***********************************PLANNING/HMI 发送车位列表
    //check psd output to planning(1 slot list)  
    //规划暂时不用车位列表，需等待预规划模块ready后。暂时用于HMI显示车位列表
    memset(psd2planning.SfusionSrchSlots, 0, sizeof(psd2planning.SfusionSrchSlots));
    int planning_slotnum = outputSlot_FUSED.slots_in_cur_frame.size();
    LOGD("slot list size:%d", planning_slotnum);
    
    int k = 0;
    for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame) {
        if (k >= slotlist_size || k >= 50) {
            std::cout << "die in planning and size is:" << slotlist_size << std::endl;
            break;
        }

        psd2planning.timeStamp = (current1970_ms >= 0) ? static_cast<uint64_t>(current1970_ms) : 0;
        psd2planning.SfusionSrchSlots[k].slotID = psd_m_output.rectInfo.label;
        
        // *******************正逆鱼骨
        double ABx = psd_m_output.rectInfo.pt[1].x - psd_m_output.rectInfo.pt[0].x;
        double ABy = psd_m_output.rectInfo.pt[1].y - psd_m_output.rectInfo.pt[0].y;
        double ADx = psd_m_output.rectInfo.pt[3].x - psd_m_output.rectInfo.pt[0].x;
        double ADy = psd_m_output.rectInfo.pt[3].y - psd_m_output.rectInfo.pt[0].y;
        double dotProduct = (ABx * ADx) + (ABy * ADy);
        double magnitudeAB = sqrt(ABx * ABx + ABy * ABy);
        double magnitudeAD = sqrt(ADx * ADx + ADy * ADy);
        // 计算夹角的余弦值
        double cosTheta = dotProduct / (magnitudeAB * magnitudeAD);
        // 计算角度（弧度转度）
        double angleRadians = acos(cosTheta);  // 计算弧度
        double angleDegrees = angleRadians * (180.0 / M_PI);  // 转换为度
        
        if (angleDegrees > 80 || angleDegrees < 100) {
            psd2planning.SfusionSrchSlots[k].slotType = slottype_rd2decplan(psd_m_output.rectInfo.PStype);
        } else if (angleDegrees <= 80) {
            psd2planning.SfusionSrchSlots[k].slotType = Sfus::SLOTTYP_RFOBL;
        } else {
            psd2planning.SfusionSrchSlots[k].slotType = Sfus::SLOTTYP_OBL;
        }
        //*******************
        
        if (psd2planning.SfusionSrchSlots[k].slotID >= 1000 && psd2planning.SfusionSrchSlots[k].slotID < 10000) {
            psd2planning.SfusionSrchSlots[k].slotSource = Sfus::SLOTSRC_VIS;
        } else if (psd2planning.SfusionSrchSlots[k].slotID >= 10000) {
            psd2planning.SfusionSrchSlots[k].slotSource = Sfus::SLOTSRC_USS;
        } else {
            psd2planning.SfusionSrchSlots[k].slotSource = Sfus::SLOTSRC_NULL;
        }
        
        psd2planning.SfusionSrchSlots[k].stopper_Dis = psd_m_output.rectInfo.StopperDistance;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerA.x = psd_m_output.rectInfo.pt[0].x;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerA.y = psd_m_output.rectInfo.pt[0].y;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerB.x = psd_m_output.rectInfo.pt[1].x;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerB.y = psd_m_output.rectInfo.pt[1].y;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerC.x = psd_m_output.rectInfo.pt[2].x;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerC.y = psd_m_output.rectInfo.pt[2].y;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerD.x = psd_m_output.rectInfo.pt[3].x;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerD.y = psd_m_output.rectInfo.pt[3].y;

        LOGD("[PSD2PLANNING] TOTAL SLOT NUM: %d, Slot#%d, type: %d, source: %d, stopdis: %f (%.1f, %.1f) (%.1f, %.1f) (%.1f, %.1f) (%.1f, %.1f)",
            planning_slotnum,
            psd2planning.SfusionSrchSlots[k].slotID,
            psd2planning.SfusionSrchSlots[k].slotType,
            psd2planning.SfusionSrchSlots[k].slotSource,
            psd2planning.SfusionSrchSlots[k].stopper_Dis,
            psd2planning.SfusionSrchSlots[k].slotCorners.cornerA.x,
            psd2planning.SfusionSrchSlots[k].slotCorners.cornerA.y,
            psd2planning.SfusionSrchSlots[k].slotCorners.cornerB.x,
            psd2planning.SfusionSrchSlots[k].slotCorners.cornerB.y,
            psd2planning.SfusionSrchSlots[k].slotCorners.cornerC.x,
            psd2planning.SfusionSrchSlots[k].slotCorners.cornerC.y,
            psd2planning.SfusionSrchSlots[k].slotCorners.cornerD.x,
            psd2planning.SfusionSrchSlots[k].slotCorners.cornerD.y);
        k++;
    }
}

void cpsd_fusion_process::ProcessPSD2PlanningTargetSlot(uint64_t current1970_ms, int& target_slot_fusionSlotType)
{
    //***********************************PLANNING 发送目标车位
    //check psd output to planning(2 target slot)
    //拿到目标车位ID后，发送目标车位信息给planning
    target_slot_fusionSlotType = 0;

    // if (final_ID > 0 && apa_status != 5 && parkout_flag != 1){ //进入guidance后固定目标车位角点
    if (final_ID > 0 && parkout_flag != 1) { //进入guidance后持续更新目标车位
        // if (final_ID > 0 && parkout_flag != 1 && (apa_status != 5 || (apa_status == 5 && !target_slot_already_updated_once))){ //进入guidance后只更新一次目标车位

        //SEARCH阶段持续更新
        if (apa_status != 5) {
            for (int i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size(); ++i) {
                if (final_ID == outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label) { // 找到目标车位

                    // *******************正逆鱼骨，车位类型*******************
                    double ABx = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x;
                    double ABy = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                    double ADx = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x;
                    double ADy = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                    double dotProduct = (ABx * ADx) + (ABy * ADy);
                    double magnitudeAB = sqrt(ABx * ABx + ABy * ABy);
                    double magnitudeAD = sqrt(ADx * ADx + ADy * ADy);
                    // 计算夹角的余弦值
                    double cosTheta = dotProduct / (magnitudeAB * magnitudeAD);
                    // 计算角度（弧度转度）
                    double angleRadians = acos(cosTheta);  // 计算弧度
                    double angleDegrees = angleRadians * (180.0 / M_PI);  // 转换为度
                    
                    if (angleDegrees > 80 || angleDegrees < 100) {
                        psd2planning.targetSlot.slotType = slottype_rd2decplan(outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.PStype);
                    } else if (angleDegrees <= 80) {
                        psd2planning.targetSlot.slotType = Sfus::SLOTTYP_RFOBL;
                    } else {
                        psd2planning.targetSlot.slotType = Sfus::SLOTTYP_OBL;
                    }

                    psd2planning.targetSlot.slotCorners.cornerA.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x;
                    psd2planning.targetSlot.slotCorners.cornerA.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                    psd2planning.targetSlot.slotCorners.cornerB.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x;
                    psd2planning.targetSlot.slotCorners.cornerB.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y;
                    psd2planning.targetSlot.slotCorners.cornerC.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].x;
                    psd2planning.targetSlot.slotCorners.cornerC.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].y;
                    psd2planning.targetSlot.slotCorners.cornerD.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x;
                    psd2planning.targetSlot.slotCorners.cornerD.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y;

                    // *******************目标车位记忆*******************
                    // 记忆更新前的车位类型
                    slot_type_before_update = psd2planning.targetSlot.slotType;
                    // 目标车位转世界坐标系
                    POINT_I ptA = { psd2planning.targetSlot.slotCorners.cornerA.x, psd2planning.targetSlot.slotCorners.cornerA.y };
                    POINT_I ptB = { psd2planning.targetSlot.slotCorners.cornerB.x, psd2planning.targetSlot.slotCorners.cornerB.y };
                    POINT_I ptC = { psd2planning.targetSlot.slotCorners.cornerC.x, psd2planning.targetSlot.slotCorners.cornerC.y };
                    POINT_I ptD = { psd2planning.targetSlot.slotCorners.cornerD.x, psd2planning.targetSlot.slotCorners.cornerD.y };
                    search_target_center.x = (ptA.x + ptB.x + ptC.x + ptD.x) / 4;
                    search_target_center.y = (ptA.y + ptB.y + ptC.y + ptD.y) / 4;
                    search_target_center_world = PSDConfigUtils::Local2Global(search_target_center, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);

                    // *******************目标车位世界坐标系顺序记忆*******************
                    world_slot_memory[0] = PSDConfigUtils::Local2Global(ptA, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                    world_slot_memory[1] = PSDConfigUtils::Local2Global(ptB, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                    world_slot_memory[2] = PSDConfigUtils::Local2Global(ptC, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                    world_slot_memory[3] = PSDConfigUtils::Local2Global(ptD, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                    LOGD("[PSD2PLANNING][UPDATE BEFORE GUIDANCE] world_slot_memory: A(%d,%d), B(%d,%d), C(%d,%d), D(%d,%d)",
                        world_slot_memory[0].x, world_slot_memory[0].y,
                        world_slot_memory[1].x, world_slot_memory[1].y,
                        world_slot_memory[2].x, world_slot_memory[2].y,
                        world_slot_memory[3].x, world_slot_memory[3].y);

                    // *******************判定是否狭窄车位*******************
                    double dx = psd2planning.targetSlot.slotCorners.cornerB.x - psd2planning.targetSlot.slotCorners.cornerA.x;
                    double dy = psd2planning.targetSlot.slotCorners.cornerB.y - psd2planning.targetSlot.slotCorners.cornerA.y;
                    double AB_dist = sqrt(dx * dx + dy * dy);
                    LOGD("isNarrow: %d, AB_dist: %f", isNarrow, AB_dist);
                    
                    if (AB_dist <= NARROWSLOT_THRESHOLD - 100) {
                        isNarrow = true;
                    } else if (AB_dist > NARROWSLOT_THRESHOLD + 100) {
                        isNarrow = false;
                    }
                    
                    //********************车位来源******************
                    psd2planning.targetSlot.stopper_Dis = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.StopperDistance;
                    if (final_ID >= 1000 && final_ID < 10000) {
                        psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
                    } else if (final_ID >= 10000) {
                        psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_USS;
                    } else {
                        psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
                    }
                    
                    //*****************无车位材质接口，借用，0视觉1超声波3草砖*********************
                    if (outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.iMaterial == 1) {
                        target_slot_fusionSlotType = 3;
                    } else {
                        if (outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label >= 1000 && outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label < 10000) {
                            target_slot_fusionSlotType = 0;
                        } else {
                            target_slot_fusionSlotType = 1;
                        }
                    }
                }
            }
        } else if (apa_status == 5) {
            ProcessGuidanceTargetSlotUpdate(target_slot_fusionSlotType);
        }
    }

    // 发送planning数据
    if (parkout_flag != 1) {
        if (apa_status == 1 || apa_status == 6 || apa_status == 7 || apa_status == 0) {
            target_slot_already_updated_once = false; // 重置标志位
            memset(&psd2planning, 0, sizeof(Sfus::Sfsuion2DecPlan));
            EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);
        } else {
            LOGD("[PSD2PLANNING] UPDATED: %d, TIMESTAMP: %llu, APASTATUS: %d, TARGET SLOT type: %d, source: %d, stopper dis: %f, (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
                target_slot_already_updated_once,
                psd2planning.timeStamp,
                apa_status,
                psd2planning.targetSlot.slotType,
                psd2planning.targetSlot.slotSource,
                psd2planning.targetSlot.stopper_Dis,
                psd2planning.targetSlot.slotCorners.cornerA.x,
                psd2planning.targetSlot.slotCorners.cornerA.y,
                psd2planning.targetSlot.slotCorners.cornerB.x,
                psd2planning.targetSlot.slotCorners.cornerB.y,
                psd2planning.targetSlot.slotCorners.cornerC.x,
                psd2planning.targetSlot.slotCorners.cornerC.y,
                psd2planning.targetSlot.slotCorners.cornerD.x,
                psd2planning.targetSlot.slotCorners.cornerD.y);
            LOGD("[PSD2PLANNING] target_slot_fusionSlotType: %d", target_slot_fusionSlotType);
            EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);

            if (target_slot_in_range && apa_status == 5) {
                POINT_I ptA = { psd2planning.targetSlot.slotCorners.cornerA.x, psd2planning.targetSlot.slotCorners.cornerA.y };
                POINT_I ptB = { psd2planning.targetSlot.slotCorners.cornerB.x, psd2planning.targetSlot.slotCorners.cornerB.y };
                POINT_I ptC = { psd2planning.targetSlot.slotCorners.cornerC.x, psd2planning.targetSlot.slotCorners.cornerC.y };
                POINT_I ptD = { psd2planning.targetSlot.slotCorners.cornerD.x, psd2planning.targetSlot.slotCorners.cornerD.y };
                POINT_I A_world = PSDConfigUtils::Local2Global(ptA, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I B_world = PSDConfigUtils::Local2Global(ptB, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I C_world = PSDConfigUtils::Local2Global(ptC, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I D_world = PSDConfigUtils::Local2Global(ptD, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                LOGD("[PSD2PLANNING] TARGET_SLOT_WORLD A(%d,%d), B(%d,%d), C(%d,%d), D(%d,%d)",
                    A_world.x, A_world.y,
                    B_world.x, B_world.y,
                    C_world.x, C_world.y,
                    D_world.x, D_world.y);
            }
        }
    }
}

void cpsd_fusion_process::ProcessGuidanceTargetSlotUpdate(int& target_slot_fusionSlotType)
{
    LOGD("[PSD2PLANNING][UPDATE IN GUIDANCE] world_slot_memory: A(%d,%d), B(%d,%d), C(%d,%d), D(%d,%d)",
        world_slot_memory[0].x, world_slot_memory[0].y,
        world_slot_memory[1].x, world_slot_memory[1].y,
        world_slot_memory[2].x, world_slot_memory[2].y,
        world_slot_memory[3].x, world_slot_memory[3].y);

    for (int i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size(); ++i) {
        if ((final_ID == outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label)) { // 找到目标车位

            //********************限位块距离******************
            psd2planning.targetSlot.stopper_Dis = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.StopperDistance;

            if (!target_slot_already_updated_once && slot_type_before_update == 1) { //进入guidance后只更新一次目标车位

                // *******************正逆鱼骨，车位类型*******************
                double ABx = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x;
                double ABy = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                double ADx = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x;
                double ADy = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                double dotProduct = (ABx * ADx) + (ABy * ADy);
                double magnitudeAB = sqrt(ABx * ABx + ABy * ABy);
                double magnitudeAD = sqrt(ADx * ADx + ADy * ADy);
                // 计算夹角的余弦值
                double cosTheta = dotProduct / (magnitudeAB * magnitudeAD);
                // 计算角度（弧度转度）
                double angleRadians = acos(cosTheta);  // 计算弧度
                double angleDegrees = angleRadians * (180.0 / M_PI);  // 转换为度
                
                if (angleDegrees > 80 || angleDegrees < 100) {
                    psd2planning.targetSlot.slotType = slottype_rd2decplan(outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.PStype);
                } else if (angleDegrees <= 80) {
                    psd2planning.targetSlot.slotType = Sfus::SLOTTYP_RFOBL;
                } else {
                    psd2planning.targetSlot.slotType = Sfus::SLOTTYP_OBL;
                }

                // *******************判定是否狭窄车位*******************
                double dx = psd2planning.targetSlot.slotCorners.cornerB.x - psd2planning.targetSlot.slotCorners.cornerA.x;
                double dy = psd2planning.targetSlot.slotCorners.cornerB.y - psd2planning.targetSlot.slotCorners.cornerA.y;
                double AB_dist = sqrt(dx * dx + dy * dy);
                LOGD("isNarrow: %d, AB_dist: %f", isNarrow, AB_dist);
                
                if (AB_dist <= NARROWSLOT_THRESHOLD - 100) {
                    isNarrow = true;
                } else if (AB_dist > NARROWSLOT_THRESHOLD + 100) {
                    isNarrow = false;
                }
                
                //********************车位来源******************
                if (final_ID >= 1000 && final_ID < 10000) {
                    psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
                } else if (final_ID >= 10000) {
                    psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_USS;
                } else {
                    psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
                }
                
                //*****************无车位材质接口，借用，0视觉1超声波3草砖*********************
                if (outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.iMaterial == 1) {
                    target_slot_fusionSlotType = 3;
                } else {
                    if (outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label >= 1000 && outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label < 10000) {
                        target_slot_fusionSlotType = 0;
                    } else {
                        target_slot_fusionSlotType = 1;
                    }
                }

                // *******************目标车位更新一次*******************
                // 锁定更新前的车位类型
                if (slot_type_before_update != NULL) {
                    psd2planning.targetSlot.slotType = slot_type_before_update;
                }

                // *******************检查四个角点是否在同相机*******************
                target_slot_in_range = true;
                POINT_I point_A = { outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y };
                POINT_I point_B = { outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y };
                POINT_I pointA_pixel = math::coordConvert_car_center_to_pixel(point_A);
                POINT_I pointB_pixel = math::coordConvert_car_center_to_pixel(point_B);
                int camera_id_A, camera_id_B;
                
                if (pointA_pixel.x > 0 && pointA_pixel.y > 0 && pointB_pixel.x > 0 && pointB_pixel.y > 0 &&
                    pointA_pixel.x <= 895 && pointA_pixel.y <= 895 && pointB_pixel.x <= 895 && pointB_pixel.y <= 895) {

                    camera_id_A = ipm_camera_id_image[pointA_pixel.y][pointA_pixel.x];
                    camera_id_B = ipm_camera_id_image[pointB_pixel.y][pointB_pixel.x];
                    LOGD("[PSD2PLANNING][UPDATE IN GUIDANCE] single frame slot pointA (%d, %d), pointB (%d, %d)", pointA_pixel.x, pointA_pixel.y, pointB_pixel.x, pointB_pixel.y);
                    LOGD("[PSD2PLANNING][UPDATE IN GUIDANCE] single frame slot id: %d, camera_id_A = %d, camera_id_B = %d", final_ID, camera_id_A, camera_id_B);
                    
                    if (camera_id_A == 0 || camera_id_B == 0) {
                        target_slot_in_range = false;
                        continue;
                    }
                    if (camera_id_A != camera_id_B) {
                        target_slot_in_range = false;
                        continue;
                    }

                    LOGD("[PSD2PLANNING][UPDATE IN GUIDANCE] target slot in range: %d", target_slot_in_range);
                    // 如果不在范围内，则跳过更新
                    if (!target_slot_in_range) {
                        continue;
                    }

                    // 新目标车位转世界坐标系
                    POINT_I ptA = { outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y };
                    POINT_I ptB = { outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y };
                    POINT_I ptC = { outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].y };
                    POINT_I ptD = { outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y };
                    new_target_center.x = (ptA.x + ptB.x + ptC.x + ptD.x) / 4;
                    new_target_center.y = (ptA.y + ptB.y + ptC.y + ptD.y) / 4;
                    new_target_center_world = PSDConfigUtils::Local2Global(new_target_center, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);

                    // 计算新旧目标车位中心点的距离，如果小于阈值，则更新目标车位
                    float target_slot_diff = PSDConfigUtils::CalcDistance(search_target_center_world, new_target_center_world);
                    LOGD("[PSD2PLANNING][UPDATE IN GUIDANCE][TARGET_SLOT_CHECK] world center moved: %f,threshold: %f", target_slot_diff, MAX_SLOT_MOVE_DIST_MM);
                    
                    if (target_slot_diff < MAX_SLOT_MOVE_DIST_MM) {
                        // 根据世界坐标最小距离匹配角点顺序
                        POINT_I cur_local_pts[4] = { ptA, ptB, ptC, ptD };
                        POINT_I cur_world_pts[4];
                        for (int j = 0; j < 4; ++j) {
                            cur_world_pts[j] = PSDConfigUtils::Local2Global(cur_local_pts[j], pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                        }

                        int matched_idx[4] = { -1, -1, -1, -1 };
                        bool used_flag[4] = { false, false, false, false };

                        for (int k = 0; k < 4; ++k) {
                            float min_dist = 1e9;
                            int best_j = -1;
                            for (int j = 0; j < 4; ++j) {
                                if (used_flag[j]) continue;
                                float dx = world_slot_memory[k].x - cur_world_pts[j].x;
                                float dy = world_slot_memory[k].y - cur_world_pts[j].y;
                                float dist = std::sqrt(dx * dx + dy * dy);
                                if (dist < min_dist) {
                                    min_dist = dist;
                                    best_j = j;
                                }
                            }
                            matched_idx[k] = best_j;
                            used_flag[best_j] = true;
                        }

                        // 未排角点顺序
                        psd2planning.targetSlot.slotCorners.cornerA.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x;
                        psd2planning.targetSlot.slotCorners.cornerA.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                        psd2planning.targetSlot.slotCorners.cornerB.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x;
                        psd2planning.targetSlot.slotCorners.cornerB.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y;
                        psd2planning.targetSlot.slotCorners.cornerC.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].x;
                        psd2planning.targetSlot.slotCorners.cornerC.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].y;
                        psd2planning.targetSlot.slotCorners.cornerD.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x;
                        psd2planning.targetSlot.slotCorners.cornerD.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y;
                        target_slot_already_updated_once = true;
                    }
                }
            }
        }
    }
}

void cpsd_fusion_process::ProcessPSD2Perception(uint64_t current1970_ms, int target_slot_fusionSlotType)
{
    //***********************************PERCEPTION 发送目标车位
    // 拿到目标车位后，发送给planning的目标车位信息，再给perception
    if (parkout_flag != 1) {
        Sfus::SfusionSlots psd2perception;
        memset(&psd2perception, 0, sizeof(Sfus::SfusionSlots));
        
        if (psd2planning.targetSlot.slotCorners.cornerA.x != 0) {
            psd2perception.slotCorners.cornerA.x = psd2planning.targetSlot.slotCorners.cornerA.x;
            psd2perception.slotCorners.cornerA.y = psd2planning.targetSlot.slotCorners.cornerA.y;
            psd2perception.slotCorners.cornerB.x = psd2planning.targetSlot.slotCorners.cornerB.x;
            psd2perception.slotCorners.cornerB.y = psd2planning.targetSlot.slotCorners.cornerB.y;
            psd2perception.slotCorners.cornerC.x = psd2planning.targetSlot.slotCorners.cornerC.x;
            psd2perception.slotCorners.cornerC.y = psd2planning.targetSlot.slotCorners.cornerC.y;
            psd2perception.slotCorners.cornerD.x = psd2planning.targetSlot.slotCorners.cornerD.x;
            psd2perception.slotCorners.cornerD.y = psd2planning.targetSlot.slotCorners.cornerD.y;
            psd2perception.slotType = psd2planning.targetSlot.slotType;

            //*****************无车位材质接口，借用，0视觉1超声波3草砖*********************
            switch (target_slot_fusionSlotType) {
            case 0:
                psd2perception.targetPosType = Sfus::POSHEADING_NULL;
                break;
            case 1:
                psd2perception.targetPosType = Sfus::POSHEADING_AB;
                break;
            case 3:
                psd2perception.targetPosType = Sfus::POSHEADING_CD;
                break;
            default:
                psd2perception.targetPosType = Sfus::POSHEADING_NULL;
                break;
            }

            psd2perception.slotSource = psd2planning.targetSlot.slotSource;
            psd2perception.timeStamp = (current1970_ms >= 0) ? static_cast<uint64_t>(current1970_ms) : 0;
            psd2perception.flag_valid = mirror_fold_flag;//后视镜折叠状态
        }

        LOGD("[PSD2PERCEPTION] TIMESTAMP: %llu, APASTATUS: %d, TARGET SLOT type: %d, targetPosType: %d, source: %d, mirrorfold: %d (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
            psd2perception.timeStamp,
            apa_status,
            psd2perception.slotType,
            psd2perception.targetPosType,
            psd2perception.slotSource,
            psd2perception.flag_valid,
            psd2perception.slotCorners.cornerA.x,
            psd2perception.slotCorners.cornerA.y,
            psd2perception.slotCorners.cornerB.x,
            psd2perception.slotCorners.cornerB.y,
            psd2perception.slotCorners.cornerC.x,
            psd2perception.slotCorners.cornerC.y,
            psd2perception.slotCorners.cornerD.x,
            psd2perception.slotCorners.cornerD.y);
        EMC_psd_fusion_process_SetFieldSfusionSlots(psd2perception);
    }
}

void cpsd_fusion_process::ProcessPSD2StateMachine()
{
    //***********************************STATEMACHINE 交互
    psd2statemachine.aps_apaParkPlaceNum = slotlist_size;

    if (apa_status == 1 || apa_status == 6 || apa_status == 7) {
        psd2statemachine.aps_apaParkType = 0;
        psd2statemachine.aps_apaParkPlaceNum = 0;
        psd2statemachine.aps_apaHighlightSlot = 0;
        final_ID = 0;
        HMI_temp_ID = 0;
    }
    
    if (final_ID > 0) { //有点选或推荐
        psd2statemachine.aps_apaParkType = slottype_decplan2statemachine(psd2planning.targetSlot.slotType);
        if (final_ID >= 10000) {
            psd2statemachine.aps_apaParkFusionType = 1;
        } else {
            psd2statemachine.aps_apaParkFusionType = 0;
        }

        if (slotlist_size > 0) {
            psd2statemachine.aps_apaHighlightSlot = 1;
        }

        psd2statemachine.aps_apaAvailableSlot = 1;
    } else { //无点选或推荐
        psd2statemachine.aps_apaParkType = 0;
        psd2statemachine.aps_apaHighlightSlot = 0;
        psd2statemachine.aps_apaAvailableSlot = available_slot_flag_to_statemachine;
    }
    
    LOGD("[PSD2STATEMACHINE] SELECT ID: %d, ParkType = %d, ParkFusionType, %d, NarrowSlot: %d, ParkPlaceNum: %d, AvailableSlot: %d, HighlightSlot: %d",
        final_ID,
        psd2statemachine.aps_apaParkType,
        psd2statemachine.aps_apaParkFusionType,
        psd2statemachine.aps_apaNarrowSlot,
        psd2statemachine.aps_apaParkPlaceNum,
        psd2statemachine.aps_apaAvailableSlot,
        psd2statemachine.aps_apaHighlightSlot);

    S2S_MCore_Bridge_SetSigStatusDecFusionInput(&psd2statemachine);
}

void cpsd_fusion_process::ProcessPSD2USS()
{
    //***********************************USS 发送目标车位ID
    short targetUssSlotID = 0;

    if (final_ID >= 10000) {
        if (final_ID > SHRT_MAX) {
            LOGD("Error: final_ID exceeds short range!\n");
        } else {
            targetUssSlotID = (short)final_ID;
            S2S_MCore_Bridge_SetSigtargetUssSlotLabel(&targetUssSlotID);
        }
    }
}

void cpsd_fusion_process::ProcessPSD2Control()
{
    //***********************************Control 发送限位块信息
    memset(&psd2control, 0, sizeof(APAControlBumpInput));
    
    if (final_ID != 0) {
        for (int i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size(); ++i) {
            if (outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label == final_ID) {
                for (int j = 0; j < 2; ++j) {
                    psd2control.apc_LimitBarX[j] = static_cast<tInt16>(outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.StopperX[j]);
                    psd2control.apc_LimitBarY[j] = static_cast<tInt16>(outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.StopperY[j]);
                }

                // 若限位块均为(0,0)，则设置为(-20000,-20000)
                if (psd2control.apc_LimitBarX[0] == 0 && psd2control.apc_LimitBarY[0] == 0 &&
                    psd2control.apc_LimitBarX[1] == 0 && psd2control.apc_LimitBarY[1] == 0) {
                    psd2control.apc_LimitBarX[0] = -20000;
                    psd2control.apc_LimitBarY[0] = -20000;
                    psd2control.apc_LimitBarX[1] = -20000;
                    psd2control.apc_LimitBarY[1] = -20000;
                }
                break;
            }
        }
    } else if (parkout_flag == 1) { // 泊出时，限位块设置为(-20000,-20000)
        psd2control.apc_LimitBarX[0] = -20000;
        psd2control.apc_LimitBarY[0] = -20000;
        psd2control.apc_LimitBarX[1] = -20000;
        psd2control.apc_LimitBarY[1] = -20000;
    }
    
    LOGD("[PSD2CONTROL]LimitBar for target slot: (%d, %d), (%d, %d)",
        psd2control.apc_LimitBarX[0], psd2control.apc_LimitBarY[0],
        psd2control.apc_LimitBarX[1], psd2control.apc_LimitBarY[1]);

    S2S_MCore_Bridge_SetSigAPAControlBumpInput(&psd2control);
}

tResult cpsd_fusion_process::TimeTrigger_thread_50ms_1()
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::TimeTrigger_thread_50ms_2()
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnVehicleCanData(const VehicleCanData& userData)
{
    auto CANcurrent = std::chrono::system_clock::now(); 
    auto CANcurrent1970 = CANcurrent.time_since_epoch();
    auto CANcurrent1970_ms = std::chrono::duration_cast<std::chrono::milliseconds>(CANcurrent1970).count(); //用于J5时间同步
    LOGD("[CANData] timestamp: %llu, WhlDistEdgeCntrLRHigFreq: %d, WhlDistEdgeCntrRRHigFreq: %d, WhlDistEdgeCntrRFHigFreq: %d, WhlDistEdgeCntrLFHigFreq: %d, WhlAngVelRFrtAuth: %f, WhlAngVelLFrtAuth: %f, WhlAngVelRRrAuth: %f, WhlAngVelLRrAuth: %f, IMULonAccPri: %f, IMULonAccSec: %f, IMULatAccPrim: %f, IMULatACCSec: %f, IMUYawRtPri: %f, IMUYawRtSec: %f, StrWhAng: %f, VehSpdAvgNDrvn: %f, TARS_TransActRng: %d",
        static_cast<uint64_t>(CANcurrent1970_ms),
        userData.WhlDistEdgeCntrLRHigFreq,
        userData.WhlDistEdgeCntrRRHigFreq,
        userData.WhlDistEdgeCntrRFHigFreq,
        userData.WhlDistEdgeCntrLFHigFreq,
        userData.WhlAngVelRFrtAuth,
        userData.WhlAngVelLFrtAuth,
        userData.WhlAngVelRRrAuth,
        userData.WhlAngVelLRrAuth,
        userData.IMULonAccPri,
        userData.IMULonAccSec,
        userData.IMULatAccPrim,
        userData.IMULatACCSec,
        userData.IMUYawRtPri,
        userData.IMUYawRtSec,
        userData.StrWhAng,
        userData.VehSpdAvgNDrvn,
        userData.TARS_TransActRng);
        
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnStatusDecOutput(const StatusDecOutput& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnUssIf_stPLVOutputInfo(const UssIf_stPLVOutputInfo_t& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnAPAControlPlanOutput(const APAControlPlanOutput& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnStatusDecFusionOutput(const StatusDecFusionOutput& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnAPAControlDebugOutput(const APAControlDebugOutput& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnStatusDec2FusionDebug(const StatusDec2FusionDebug& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnStateMachine_Output(const StateMachine_Output& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnApp2emap_DR(const Loc::App2emap_DR& userData)
{
    DRPoseWithTime dr_with_time = { userData, userData.timeStamp };
    {
        std::lock_guard<std::mutex> lock(_dr_mutex);
        dr_pose_buffer.push_back(dr_with_time);
        if (dr_pose_buffer.size() > MAX_BUFFER_SIZE) {
            dr_pose_buffer.pop_front();
        }
    }

    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnMapInfo(const Loc::MapInfo& userData)
{
    LOGD("[APA_SLAM] OnMapInfo size: %d", userData.ParkingSlot.size());
    {
        std::lock_guard<std::mutex> lock(_map_mutex);
        LOGD("[APA_SLAM] Pushing map info");

        map_info_buffer.push_back(userData);
        if (map_info_buffer.size() > MAX_BUFFER_SIZE) {
            map_info_buffer.pop_front();
        }
        LOGD("[APA_SLAM] Pushing map info Done");

    }
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnObstacles(const od::Obstacles& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnSApaPSInfo(const rd::SApaPSInfo& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnQuadParkingSlots(const rd::QuadParkingSlots& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnImage(const rd::Image& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnHMI_InputInfo(const HMI_InputInfo& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnSelectSlot(const Sfus::SelectSlot& userData)
{
    VCU_select_ID_ON = userData.SelectSlotID;
    LOGD("[SELECTID] OnSelectSlot VCU ID: %d !!!!",VCU_select_ID_ON);
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnParkInHeadInSwitch(const Sfus::ParkInHeadInSwitch& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnSelectSlot2(const Sfus::SelectSlot& userData)
{
    HMI_select_ID = userData.SelectSlotID;
    LOGD("[SELECTID] OnSelectSlot2 HMI ID: %d !!!!",HMI_select_ID);
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnEmapWorkMode(const Fus::EmapWorkMode& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnFusionTimeStamp(const Fus::FusionTimeStamp& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnPkEmapObs(const Fus::PkEmapObs& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnEmapSlotVector(const Fus::EmapSlotVector& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnStableEmapObs(const Fus::StableEmapObs& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnFusionSlotInfo(const Fus::FusionSlotInfo& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnParkInHeadInSwitch2(const Sfus::ParkInHeadInSwitch& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnSelectSlot3(const Sfus::SelectSlot& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnHMI_InputInfo2(const HMI_InputInfo& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnDecPlan2Emap(const Pla::DecPlan2Emap& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnPlan2Psd(const Pla::Plan2Psd& userData)
{
    mirror_fold_flag_ahead = userData.mirrorFoldFlg;
    // 0 -> 1
    if (mirror_fold_flag == 0 && mirror_fold_flag_ahead == 1)
    {
        mirror_fold_flag = 1;
    }
    //KEEP
    if (mirror_fold_flag == 1 && apa_status == 5)
    {
        mirror_fold_flag = 1;
    }
    //RESET
    if (mirror_fold_flag_ahead == 0 && apa_status != 5)
    {
        mirror_fold_flag = 0;
    }

    LOGD("[MIRRORFOLD] OnPlan2Psd apastatus: %d, mirror_fold_flag_ahead: %d, mirror_fold_flag: %d",apa_status,mirror_fold_flag_ahead,mirror_fold_flag);

    RETURN_NOERROR;
}

tResult cpsd_fusion_process::TimeTrigger_thread_100ms_1()
{
    // ============================================================Part0 行泊切换
	static kbd::sm::StateClient state_client;
    if (state_client.parking_stop()){
        // 行泊切换清零，输出发送空值
        PSD_FusionModuleIFrunable.ClearSlotsMap();
        g_singleframe_locked_slots.clear();
        target_slot_already_updated_once = false;

        LOGW("NOT IN PARKING STATE");
        RETURN_NOERROR;
    }

    // ============================================================Part1 GetInput

    auto current = std::chrono::system_clock::now(); 
    auto current1970 = current.time_since_epoch();
    auto current1970_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current1970).count();
    auto start = std::chrono::steady_clock::now();

    LOGD("PSD Version: 07031508 emos10.0.1 [LYK]: 0703 release add dignoal slot");
    // GET方式获取
    GetInput getInput;
    getInput.GetAllInput();

    rd_info = getInput.rd_info;
    singleframeslotsID = getInput.singleframeslotsID;
    singleframeslots = getInput.singleframeslots;
    dr_pose = getInput.dr_pose;
    pose_globaldata = getInput.pose_globaldata;
    obs_info_get = getInput.obs_info_get;
    apa_status = getInput.apa_status;
    auto uss_info_restruct = uss_info;
    static bool has_cleared_for_SEARCH_once = false; //是否已经保护清零（进入SEARCH时清零一次）
    static int cached_selected_label = -1; // SEARCH-GUIDANCE切换时固定的目标ID
    // search_interrupt = getInput.search_interrupt; //@TODO

    // ============================================================Part2 InputChecker

    HandleStateBasedClear(apa_status);
    HandleSearchStateFirstEnter(apa_status, has_cleared_for_SEARCH_once);
    UpdateParkoutAndStill(apa_status, dr_pose, previous_dr_pose, parkout_flag, is_Still);
    if (mirror_fold_flag == 1) {
        HandleMirrorFold(singleframeslots, singleframeslotsID, rd_info);
    }
    PrintSlotCheckStatus(singleframeslots, outputSlot_VIS, outputSlot_USS, outputSlot_FUSED);

    // ============================================================Part3 SlotProcessor
    // Yukan: Convert Slot using Mapinfo
    Loc::MapInfo latest_map_info;
    {
       LOGD("[APA_SLAM] Acquiring latest map info");
       std::lock_guard<std::mutex> lock(_map_mutex);
       if (map_info_buffer.size() == 0){
        RETURN_NOERROR;
       }
       latest_map_info = map_info_buffer.back();
       LOGD("[APA_SLAM] Acquiring latest map info Done");
    }
    outputSlot_VIS.slots_in_cur_frame.clear();
    outputSlot_VIS.WorldoutRect.clear();
    LOGD("[APA_SLAM] start processing slot list");
    ProcessMapInfoSlot(latest_map_info, pose_globaldata, outputSlot_VIS, PSD_FusionModuleIFrunable);

    if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7){
        PSDConfigUtils::ClearParkingSlots(singleframeslots, singleframeslotsID, outputSlot_VIS, outputSlot_USS, outputSlot_FUSED, parkout_flag);
    }

    ProcessUssSlots(uss_info, uss_info_restruct, outputSlot_USS, outputSlot_VIS, outputSlot_FUSED, fusionslot);

    // outputSlot_FUSED优化：类型修正
    LOGD("Without SlotTypeCorrect FUSIONSLOTS:")
    PSDConfigUtils::LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");
    PSD_FusionModuleIFrunable.SlotTypeCorrect(outputSlot_FUSED);
    LOGD("After SlotTypeCorrect FUSIONSLOTS:")
    PSDConfigUtils::LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");

    // outputSlot_FUSED优化：限位块、地锁、其他障碍物
    LOGD("Without StopperLockOBS FUSIONSLOTS:")
    PSDConfigUtils::LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");
    PSD_FusionModuleIFrunable.StopperLockOBS(obs_info_get,outputSlot_FUSED);
    LOGD("After StopperLockOBS FUSIONSLOTS:")
    PSDConfigUtils::LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");

    if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7){
        PSDConfigUtils::ClearParkingSlots(singleframeslots, singleframeslotsID, outputSlot_VIS, outputSlot_USS, outputSlot_FUSED, parkout_flag);
        dr_first = true;
        slotlist_size = 0;
    }

    //============================================================Part4 点选目标ID

    // VCU,HMI 双终端接收点选的目标车位
    final_select_ID = PSDSelectionLogic::HMIVCUSelect(HMI_temp_ID,HMI_select_ID,VCU_select_ID_ON,final_select_ID);
    LOGD("[HMIVCUSELECT OUT] final_select_id: %d",final_select_ID);

    // APAStatus == standby/finish/error时，清零车位ID
    // Clear, new
    if (apa_status == 1 || apa_status == 6 || apa_status == 7){
        PSDConfigUtils::ClearSelectRecommendSlot(HMI_temp_ID, HMI_select_ID, VCU_select_ID_ON, final_select_ID, RECOMMEND_ID, final_ID, apa_status);
        already_has_recommend_slot = false;
    }
    LOGD("[STATUSSELECT] HMI %d, VCU %d, final select %d",HMI_temp_ID,VCU_select_ID_ON,final_select_ID);

    // ============================================================Part5 输出下游    

    //------------------------------------------
    //VCU 发送车位列表
    apaSlotInfo selected_slot_in_world;
    ProcessPSD2VCU(apa_status, current1970_ms, selected_slot_in_world);

    //------------------------------------------
    //APAHANDLE 发送车位列表
    ProcessPSD2APAHandle(current1970_ms);

    //------------------------------------------
    //APAHANDLE 发送目标车位ID
    ProcessPSD2APAHandleTargetID();

    //------------------------------------------
    //PLANNING/HMI 发送车位列表
    ProcessPSD2Planning(current1970_ms);

    //------------------------------------------
    //PLANNING 发送目标车位
    int target_slot_fusionSlotType = 0;
    ProcessPSD2PlanningTargetSlot(current1970_ms, target_slot_fusionSlotType);

    //------------------------------------------
    //PERCEPTION 发送目标车位
    ProcessPSD2Perception(current1970_ms, target_slot_fusionSlotType);

    //------------------------------------------
    //STATEMACHINE 交互
    ProcessPSD2StateMachine();

    //------------------------------------------
    //USS 发送目标车位ID
    ProcessPSD2USS();

    //------------------------------------------
    //Control 发送限位块信息
    ProcessPSD2Control();


    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOGD("[TIMECOST]Timetrigger100 time is: %d",elapsed.count());

    RETURN_NOERROR;
}


