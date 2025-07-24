#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "state_client.hpp"
#include <iostream>
#include <typeinfo>
#include "math.hpp"
#include "GetInput.hpp"
#include "utils.h"
#include <float.h>
#include <fstream>
#include "json.hpp"


using json = nlohmann::json;


// // ***************************标定量
// #define VEHICLE_LENGTH 5259.9 
// #define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7
// #define MM_TO_M 1000.0

// // ***************************配置文件修改的参数(@TODO：从配置文件读取后转成const)

// bool FARAWAY_FILTER = false; //距离范围限制车位释放功能开关
// float FARAWAY_SLOTS_LEFT[2] = {-99999.0,0}; 
// float FARAWAY_SLOTS_RIGHT[2] = {0,99999.0};
// float FARAWAY_SLOTS_REAR = -99999.0;
// float FARAWAY_SLOTS_FRONT = 99999.0;
// bool ANGEL_FILTER = false;
// float ANGEL_FILTER_LIMIT = -99999.0;
// bool VCU_TOO_SMALL_FILTER = false;
// float VCU_TOO_SMALL = -99999.0;
// bool PARALLEL_VECTOR_FILTER = false;
// float PARALLEL_VECTOR_LIMIT = -99999.0;
// float NARROWSLOT_THRESHOLD = -99999.0;


// // ***************************输入的全局变量，用于ON方式获取
// // 输入的全局变量，用于GET方式获取
// int apa_status = 0;
// int park_request = 0;
// const int search_interrupt = 0; //@TODO VC7 RELEASE
// int is_Still;
// Loc::App2emap_DR previous_dr_pose;




// int HMI_select_ID = 0; //HMI只发1s。HMI_select是HMI发的ID，
// int HMI_temp_ID = 0;  //HMI_temp_ID是存下来的ID
// int VCU_select_ID_ON = 0; //用ON获取的VCU发送的ID
// int RECOMMEND_ID = 0; //推荐车位的ID（类似于已点击，点泊车立即泊车）
// int final_select_ID = 0; //VCU和HMI最终统一的ID
// int final_ID = 0; //结合选择、推荐后的最终ID
// bool recommend_exist = false; //推荐车位是否已存在
// bool already_has_recommend_slot = false;

// int available_slot_flag_to_statemachine = 0; //可用车位数量flag
// bool isNarrow = false; // 是否为窄车位


// int parkout_flag = 0; //当前是否为泊出
// int mirror_fold_flag_ahead = 0; // 从planning拿的原始折叠flag（存在提前）
// int mirror_fold_flag = 0; //后视镜是否被折叠（准确值）
// static bool target_slot_already_updated_once = false; //目标车位已经被更新过一次
// bool target_slot_in_range = false; //目标车位是否在矩形范围内
// Sfus::SfusionSlotType slot_type_before_update;
// POINT_I search_target_center = {0, 0}; // 第一次的目标车位中心点世界坐标
// POINT_I search_target_center_world = {0, 0}; // 第一次的目标车位中心点世界坐标（用于计算）
// POINT_I new_target_center = {0, 0}; // 更新的目标车位中心点世界坐标
// POINT_I new_target_center_world = {0,0}; // 更新的目标车位中心点世界坐标（用于计算）
// const float MAX_SLOT_MOVE_DIST_MM = 1500.0f; // 最大容忍距离

// POINT_I world_slot_memory[4]; // ABCD角点




// ***************************输出的全局变量
slotfusion fusionslot;
StatusDecFusionInput psd2statemachine;
Sfus::Sfsuion2DecPlan psd2planning; //动态车位列表
Sfus::FusionSlotInfovector psd2vcu;
Fsm::FusionSlotInfo2Location psd2location;
APAControlBumpInput psd2control;

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
    
    auto& configManager = PSDConfigManager::getInstance();
    
    // 读取配置文件
    if (!configManager.loadConfigFromFile("/app/neo/psd_config.json")) {
        LOGD("Load config failed!");
    }

    // 读取ipm相机id图
    ipm_camera_id_image.resize(896, std::vector<int>(896, 0));
    load_image_from_csv("/app/neo/ipm_camera_id.csv", ipm_camera_id_image);

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
        
        // 初始化配置管理器状态
        configManager.resetForNewSession();
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


POINT_I Local2Global(const POINT_I& pt_local, const float& x, const float& y, const float& yaw)
{
    float theta = yaw * acos(-1) / 180.0;
    POINT_I pt_global;
    pt_global.x = pt_local.x * cos(theta) + pt_local.y * sin(theta) + x;
    pt_global.y = pt_local.y * cos(theta) - pt_local.x * sin(theta) + y;
    return pt_global;
}


int cpsd_fusion_process::HMIVCUSelect(int &hmi_temp, const int &hmi_select, const int &vcu_select)
{
    auto& configManager = PSDConfigManager::getInstance();
    
    LOGD("[HMIVCUSELECT IN] HMI:%d HMI temp:%d VCU:%d", hmi_select, hmi_temp, vcu_select);

    //HMI 部分
    //中间变量保存HMI发送的 [0 - ID - 0]，一秒内发送五次
    if (hmi_select) {
        hmi_temp = hmi_select;
        configManager.setHMISelectID(hmi_select);
    }
    
    //VCU 部分，已从GET获取
    if (vcu_select != 0) {
        configManager.setVCUSelectID(vcu_select);
    }
    
    int final_select_ID = 0;
    //VCU接收的点选车位 与 HMI接收的点选车位 二选一。优先选择VCU接收的点选车位
    if (vcu_select != 0 && hmi_temp == 0) {
        final_select_ID = vcu_select; 
    } 
    else if (vcu_select == 0 && hmi_temp != 0) {
        final_select_ID = hmi_temp;
    } 
    else if (vcu_select != 0 && hmi_temp != 0 && (vcu_select == hmi_temp)) {
        final_select_ID = hmi_temp;
    }
    else if (vcu_select == 0 && hmi_temp == 0) {
        final_select_ID = 0;
    }
    else {
        final_select_ID = vcu_select;
    }
    
    configManager.setFinalSelectID(final_select_ID);

    return final_select_ID;
}

int cpsd_fusion_process::RecommendSelectID(const int &final_select, const int &recommend)
{
    auto& configManager = PSDConfigManager::getInstance();
    
    int result = 0;
    if (final_select) {
        result = final_select;
    } else {
        result = recommend;
    }
    
    configManager.setFinalID(result);
    
    return result;
}

int cpsd_fusion_process::IsParkOut(int apastatus)
{
    auto& configManager = PSDConfigManager::getInstance();
    auto state = configManager.getGlobalState();
    
    if (apastatus == 3) {
        state.parkout_flag = 1;
    } else if (apastatus == 2) {
        state.parkout_flag = 0;
    }
    
    configManager.updateGlobalState(state);
    
    return state.parkout_flag;
}

int cpsd_fusion_process::IsStill(const Loc::App2emap_DR drpose, Loc::App2emap_DR& previous_drpose)
{
    auto& configManager = PSDConfigManager::getInstance();
    previous_drpose = configManager.getPreviousDRPose();
    
    static int no_change_count = 0;
    float epsilon = 30.0; // 设置阈值，可以根据需要调整
    bool has_changed = false; // 比较 drpose 和 previous_drpose 是否变化
    int still_threshold = 5; //静止阈值，连续多少次没有变化算静止
    
    LOGD("[STILL] dr: x:%f, y:%f, yaw: %f,previous: x:%f, y:%f, yaw:%f",
        drpose.x, drpose.y, drpose.canAng,
        previous_drpose.x, previous_drpose.y, previous_drpose.canAng);

    if (fabs(drpose.x - previous_drpose.x) > epsilon ||
        fabs(drpose.y - previous_drpose.y) > epsilon ||
        fabs(drpose.canAng - previous_drpose.canAng) > epsilon)
    {
        has_changed = true;
    }
    
    LOGD("[STILL] has_changed:%d", has_changed);

    // 如果没有变化，增加连续无变化计数
    if (!has_changed) {
        no_change_count++;
    } else {
        no_change_count = 0;  // 有变化时重置计数器
    }
    
    LOGD("[STILL] no_change_count:%d", no_change_count);

    int is_still_result = 0;
    // 如果连续still_threshold次没有变化，则认为是静止状态
    if (no_change_count >= still_threshold) {
        is_still_result = 1;  // is_Still = 1
    } else {
        is_still_result = 0;  // is_Still = 0
    }
    
    // 更新previous_drpose到配置管理器
    configManager.setPreviousDRPose(drpose);
    previous_drpose = drpose; // 同时更新传入的引用
    
    // 更新is_Still状态
    auto state = configManager.getGlobalState();
    state.is_Still = is_still_result;
    configManager.updateGlobalState(state);
    
    return is_still_result;
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
    auto& configManager = PSDConfigManager::getInstance();
    
    // 更新VCU选择ID到配置管理器
    configManager.setVCUSelectID(userData.SelectSlotID);
    
    LOGD("[SELECTID] OnSelectSlot VCU ID: %d !!!!", userData.SelectSlotID);
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnParkInHeadInSwitch(const Sfus::ParkInHeadInSwitch& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnSelectSlot2(const Sfus::SelectSlot& userData)
{
    auto& configManager = PSDConfigManager::getInstance();
    
    // 更新HMI选择ID到配置管理器
    configManager.setHMISelectID(userData.SelectSlotID);
    
    LOGD("[SELECTID] OnSelectSlot2 HMI ID: %d !!!!", userData.SelectSlotID);
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
    auto& configManager = PSDConfigManager::getInstance();
    auto state = configManager.getGlobalState();
    int apa_status = state.apa_status;
    
    state.mirror_fold_flag_ahead = userData.mirrorFoldFlg;
    
    // 0 -> 1
    if (state.mirror_fold_flag == 0 && state.mirror_fold_flag_ahead == 1) {
        state.mirror_fold_flag = 1;
    }
    
    //KEEP
    if (state.mirror_fold_flag == 1 && apa_status == 5) {
        state.mirror_fold_flag = 1;
    }
    
    //RESET
    if (state.mirror_fold_flag_ahead == 0 && apa_status != 5) {
        state.mirror_fold_flag = 0;
    }

    // 更新状态到配置管理器
    configManager.updateGlobalState(state);

    LOGD("[MIRRORFOLD] OnPlan2Psd apastatus: %d, mirror_fold_flag_ahead: %d, mirror_fold_flag: %d",
         apa_status, state.mirror_fold_flag_ahead, state.mirror_fold_flag);

    RETURN_NOERROR;
}

tResult cpsd_fusion_process::TimeTrigger_thread_100ms_1()
{
    auto& configManager = PSDConfigManager::getInstance();

    // ============================================================Part0 行泊切换
	static kbd::sm::StateClient state_client;
    if (state_client.parking_stop()){
        // 行泊切换清零，输出发送空值
        PSD_FusionModuleIFrunable.ClearSlotsMap();
        configManager.setTargetSlotUpdatedOnce(false);

        LOGW("NOT IN PARKING STATE");
        RETURN_NOERROR;
    }

    auto current = std::chrono::system_clock::now(); 
    auto current1970 = current.time_since_epoch();
    auto current1970_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current1970).count();
    auto start = std::chrono::steady_clock::now(); // 计算TIMECOST

    LOGD("PSD Version: 07171125 emos10.0.1 [LYK]: 0717 release add dignoal slot");


    // ============================================================Part1 GET获取输入
    GetInput inputData;
    inputData.GetAllInput();

    auto& rd_info = inputData.rd_info;
    auto& singleframeslotsID = inputData.singleframeslotsID;
    auto& singleframeslots = inputData.singleframeslots;
    auto& dr_pose = inputData.dr_pose;
    auto& pose_globaldata = inputData.pose_globaldata;
    auto& obs_info_get = inputData.obs_info_get;
    auto apa_status = inputData.apa_status;
    configManager.setApaStatus(apa_status);

    static bool has_cleared_for_SEARCH_once = false; //是否已经保护清零（进入SEARCH时清零一次）



    // ============================================================Part2 校验输入
    if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7){
        inputData.ClearAllInput();
        has_cleared_for_SEARCH_once = false; //flag重置
    }
    else if (apa_status == 2 && !has_cleared_for_SEARCH_once){ //第一次进search清零
        inputData.ClearAllInput();
        has_cleared_for_SEARCH_once = true;
        configManager.setTargetSlotUpdatedOnce(false);

        // 第一次清零，向下游发送空值
        memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector));
        EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
        memset(&psd2statemachine, 0, sizeof(StatusDecFusionInput));
        S2S_MCore_Bridge_SetSigStatusDecFusionInput(&psd2statemachine);
    }

    


    // ============================================================Part3 算法
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
    double pose_x = static_cast<double>(pose_globaldata.coord.x) * 0.001;
    double pose_y = static_cast<double>(pose_globaldata.coord.y) * 0.001;
    double theta = pose_globaldata.yaw / 180.0 * M_PI;
    Eigen::Vector2d twb(pose_x, pose_y);
    Eigen::Matrix2d Rwb;
    Rwb << std::cos(theta), -std::sin(theta),
           std::sin(theta), std::cos(theta);
    LOGD("[APA_SLAM] latest_map_info.ParkingSlot.size(): %d", latest_map_info.ParkingSlot.size());
    for (size_t i = 0; i < latest_map_info.ParkingSlot.size(); ++i) {
        const auto& id = latest_map_info.ParkingSlot.at(i).id;
        const auto& type = latest_map_info.ParkingSlot.at(i).psType;
        const auto& sodtype = latest_map_info.ParkingSlot.at(i).isOccupancy;
        Eigen::Vector2d c_b(latest_map_info.ParkingSlot.at(i).center.x, latest_map_info.ParkingSlot.at(i).center.y);
        Eigen::Vector2d lon_dir(latest_map_info.ParkingSlot.at(i).longDirection.x, latest_map_info.ParkingSlot.at(i).longDirection.y);
        Eigen::Vector2d w_dir(latest_map_info.ParkingSlot.at(i).wideDirection.x, latest_map_info.ParkingSlot.at(i).wideDirection.y);
        
        Eigen::Vector2d pt0_b = c_b + lon_dir * latest_map_info.ParkingSlot.at(i).length * .5f - w_dir * latest_map_info.ParkingSlot.at(i).width * .5f;
        Eigen::Vector2d pt1_b = c_b + lon_dir * latest_map_info.ParkingSlot.at(i).length * .5f + w_dir * latest_map_info.ParkingSlot.at(i).width * .5f;
        Eigen::Vector2d pt2_b = c_b - lon_dir * latest_map_info.ParkingSlot.at(i).length * .5f + w_dir * latest_map_info.ParkingSlot.at(i).width * .5f;
        Eigen::Vector2d pt3_b = c_b - lon_dir * latest_map_info.ParkingSlot.at(i).length * .5f - w_dir * latest_map_info.ParkingSlot.at(i).width * .5f;
        auto test1 = lon_dir.dot(w_dir);
        LOGD("test1 : %f",test1);

        Eigen::Vector2d pt0_w = Rwb * pt0_b + twb;
        Eigen::Vector2d pt1_w = Rwb * pt1_b + twb;
        Eigen::Vector2d pt2_w = Rwb * pt2_b + twb;
        Eigen::Vector2d pt3_w = Rwb * pt3_b + twb;

        apaSlotInfo info_cur, info_w;

        info_cur.rectInfo.pt[1].x = -pt0_b.y() * 1000;
        info_cur.rectInfo.pt[1].y = pt0_b.x() * 1000;
        info_cur.rectInfo.pt[0].x = -pt1_b.y() * 1000;
        info_cur.rectInfo.pt[0].y = pt1_b.x() * 1000;
        info_cur.rectInfo.pt[3].x = -pt2_b.y() * 1000;
        info_cur.rectInfo.pt[3].y = pt2_b.x() * 1000;
        info_cur.rectInfo.pt[2].x = -pt3_b.y() * 1000;
        info_cur.rectInfo.pt[2].y = pt3_b.x() * 1000;
        PSD_FusionModuleIFrunable.adjustRectOrder(info_cur);
        info_cur.rectInfo.label = id;
        info_cur.rectInfo.PStype = type;
        info_cur.rectInfo.iSodType = sodtype;



        info_w.rectInfo.pt[0].x = pt0_w.x() * 1000;
        info_w.rectInfo.pt[0].y = pt0_w.y() * 1000;
        info_w.rectInfo.pt[1].x = pt1_w.x() * 1000;
        info_w.rectInfo.pt[1].y = pt1_w.y() * 1000;
        info_w.rectInfo.pt[2].x = pt2_w.x() * 1000;
        info_w.rectInfo.pt[2].y = pt2_w.y() * 1000;
        info_w.rectInfo.pt[3].x = pt3_w.x() * 1000;
        info_w.rectInfo.pt[3].y = pt3_w.y() * 1000;
        info_w.rectInfo.label = id;
        info_w.rectInfo.PStype = type;
        info_w.rectInfo.iSodType = sodtype;

        outputSlot_VIS.slots_in_cur_frame.push_back(info_cur);
        outputSlot_VIS.WorldoutRect.push_back(info_w);

    }
    
    LOGD("[APA_SLAM] finish processing slot list");
    LogSlotInfo(outputSlot_VIS, "ORIGIN VISSLOTS");

    //------------------------------------------
    // get USS
    UssIf_stPLVOutputInfo_t uss_info;
    auto uss_info_restruct = uss_info;
    S2S_MCore_Bridge_GetSigUssIf_stPLVOutputInfo(&uss_info);
    fusionslot.fillVisonstruct(uss_info, outputSlot_USS);
    fusionslot.clearInvalidUSSslots(outputSlot_USS);
    fusionslot.postprocessUSSslots(uss_info_restruct);
    fusionslot.mergeSlotLists(outputSlot_USS, outputSlot_VIS, outputSlot_FUSED);





    //============================================================Part4 算法处理
    auto currentState = configManager.getGlobalState();
    
    // outputSlot_FUSED：类型修正
    PSD_FusionModuleIFrunable.SlotTypeCorrect(outputSlot_FUSED);
    LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");

    // outputSlot_FUSED：限位块、地锁、其他障碍物
    PSD_FusionModuleIFrunable.StopperLockOBS(obs_info_get,outputSlot_FUSED, currentState.final_ID);
    LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");

    


    // VCU,HMI双终端接收点选的目标车位
    int final_select_ID = HMIVCUSelect(currentState.HMI_temp_ID, currentState.HMI_select_ID, currentState.VCU_select_ID_ON);
    LOGD("[HMIVCUSELECT OUT] final_select_id: %d", final_select_ID);
    LOGD("[STATUSSELECT] HMI %d, VCU %d, final select %d", currentState.HMI_temp_ID, currentState.VCU_select_ID_ON, final_select_ID);

    // 泊出判断
    int parkout_flag = IsParkOut(apa_status);
    LOGD("[PARKOUT] flag: %d", parkout_flag);

    // 静止判断
    Loc::App2emap_DR previous_dr_pose = configManager.getPreviousDRPose();
    int is_Still = IsStill(dr_pose, previous_dr_pose);
    LOGD("[STILL] is Still: %d", is_Still);

    // 车位数统计
    slotlist_size = outputSlot_FUSED.slots_in_cur_frame.size();
    LOGD("[SLOTLISTSIZE] slotlist_size: %d", slotlist_size);

    // 清零
    if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7){
        ClearParkingSlots(singleframeslots, singleframeslotsID, outputSlot_VIS, outputSlot_USS, outputSlot_FUSED, parkout_flag);
        currentState.HMI_temp_ID = 0;
        currentState.HMI_select_ID = 0;
        currentState.VCU_select_ID_ON = 0;
        currentState.final_select_ID = 0;
        currentState.RECOMMEND_ID = 0;
        currentState.final_ID = 0;
        currentState.already_has_recommend_slot = false;
        configManager.updateGlobalState(currentState);  
    }

    // ============================================================Part6 输出下游

    //------------------------------------------
    //目标车位
    apaSlotInfo selected_slot_in_world;

    //------------------------------------------
    //VCU 发送车位列表
    if (apa_status == 2){
        LOGD("VCU display for SEARCH");

        memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector)); //displayLabel 会被重置
        psd2vcu.slotNum = slotlist_size;

        if (psd2vcu.slotNum > 0){
            int i = 0;
            LOGD("PSD2VCU apa_status: %d, outputslot_fused size: %d",apa_status,outputSlot_FUSED.slots_in_cur_frame.size());
            
            // ================= 匹配显示 =================
            for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame){
                if (i >= slotlist_size || i >= 50){
                    LOGD("die in VCU and size is:",slotlist_size);
                    break;
                }
                
                psd2vcu.FusionSlotInfo[i].slotLabel = psd_m_output.rectInfo.label; //ID
                
                // 角点转换 - 使用配置管理器中的车辆参数
                float vehicle_length = PSDConfigManager::getVehicleLength();
                float rear_axle_center = PSDConfigManager::getRearAxleCenterVehicleRear();
                float mm_to_m = PSDConfigManager::getMmToM();
                
                psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[0].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[0].x / mm_to_m;
                psd2vcu.FusionSlotInfo[i].pt[0].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[1].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[1].x / mm_to_m;
                psd2vcu.FusionSlotInfo[i].pt[1].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[2].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[2].x / mm_to_m;
                psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[3].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[3].x / mm_to_m;
                psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
                
                // 状态判断
                if (psd_m_output.rectInfo.iSodType == 1){
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                }
                else if (psd_m_output.rectInfo.iSodType != 1 && psd_m_output.rectInfo.label == currentState.RECOMMEND_ID){
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 7;
                }
                else if (psd_m_output.rectInfo.iSodType != 1 && psd_m_output.rectInfo.label != currentState.RECOMMEND_ID){
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                }
                //Unavailable 车位
                if (psd_m_output.rectInfo.NotToRelease == 1){
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 6;
                }

                // ***************************车位不释放策略***************************
                // ***************************1 车位的中心点是否在允许释放的区域
                
                // 获取配置参数
                auto config = configManager.getConfig();
                
                //限制范围（前后、左右）
                LOGD("[VCU NOTRELEASE1 range] FARAWAY_FILTER: %d, Rear-Front: [%f, %f], Left: [%f, %f], Right:[%f, %f]",
                    config.FARAWAY_FILTER, config.FARAWAY_SLOTS_REAR, config.FARAWAY_SLOTS_FRONT,
                    config.FARAWAY_SLOTS_LEFT[0], config.FARAWAY_SLOTS_LEFT[1],
                    config.FARAWAY_SLOTS_RIGHT[0], config.FARAWAY_SLOTS_RIGHT[1]);
                    
                if (config.FARAWAY_FILTER){
                    //车位中心点
                    float center_x = 0.0f;
                    float center_y = 0.0f;
                    for (int j = 0; j < 4; ++j) {
                        center_x += psd2vcu.FusionSlotInfo[i].pt[j].x;
                        center_y += psd2vcu.FusionSlotInfo[i].pt[j].y;
                    }
                    center_x /= 4.0f;
                    center_y /= 4.0f;
                    
                    //判断中心点是否在矩形范围内
                    const float FARAWAY_DEADZONE = 0.1f;// 死区（0.2m）
                    float rear_limit    = config.FARAWAY_SLOTS_REAR + FARAWAY_DEADZONE;
                    float front_limit   = config.FARAWAY_SLOTS_FRONT - FARAWAY_DEADZONE;
                    float left1         = config.FARAWAY_SLOTS_LEFT[0] + FARAWAY_DEADZONE;
                    float left2         = config.FARAWAY_SLOTS_LEFT[1] - FARAWAY_DEADZONE;
                    float right1        = config.FARAWAY_SLOTS_RIGHT[0] + FARAWAY_DEADZONE;
                    float right2        = config.FARAWAY_SLOTS_RIGHT[1] - FARAWAY_DEADZONE;
                    
                    bool out_of_x_range = (center_x <= rear_limit || center_x >= front_limit);
                    bool out_of_y_range = !((center_y >= left1 && center_y <= left2) ||
                                            (center_y >= right1 && center_y <= right2));
                                            
                    if (psd_m_output.rectInfo.PStype == 0 || psd_m_output.rectInfo.PStype == 2){ // 垂直or斜列
                        if (out_of_x_range || out_of_y_range) {
                            psd2vcu.FusionSlotInfo[i].slotStatusType = 4;
                        } else {
                            if (psd_m_output.rectInfo.iSodType == 1) {
                                psd2vcu.FusionSlotInfo[i].slotStatusType = 4;
                            } else {
                                psd2vcu.FusionSlotInfo[i].slotStatusType = 3;
                            }
                        }
                    }
                    LOGD("[VCU NOTRELEASE1 range] ID: %d, center(%.1f,%.1f), x_out: %d, y_out: %d, RD occupied: %d, VCU status: %d",
                        psd2vcu.FusionSlotInfo[i].slotLabel, center_x, center_y,
                        out_of_x_range, out_of_y_range,
                        psd_m_output.rectInfo.iSodType, psd2vcu.FusionSlotInfo[i].slotStatusType);
                }

                // ***************************2 车位与自车夹角是否在允许释放的角度范围内
                LOGD("[VCU NOTRELEASE2 anglelimit] ANGLE_FILTER: %d, ANGEL_FILTER_LIMIT:%f", config.ANGEL_FILTER, config.ANGEL_FILTER_LIMIT);
                if (config.ANGEL_FILTER){
                    if (psd_m_output.rectInfo.PStype != 2){
                        float ax = 4.0f;
                        float ay = 0.0f; //向量1 (-4.0)指向(0,0)
                        float bx = psd2vcu.FusionSlotInfo[i].pt[1].x - psd2vcu.FusionSlotInfo[i].pt[0].x;
                        float by = psd2vcu.FusionSlotInfo[i].pt[1].y - psd2vcu.FusionSlotInfo[i].pt[0].y;//向量2 A指向B
                        //夹角计算
                        float dot = ax * bx + ay * by;
                        float normA = std::sqrt(ax * ax + ay * ay);
                        float normB = std::sqrt(bx * bx + by * by);
                        if (normA * normB < 1e-6f) continue;
                        float cos_theta = dot / (normA * normB);
                        if (cos_theta > 1.0f) cos_theta = 1.0f;
                        if (cos_theta < -1.0f) cos_theta = -1.0f;
                        float angle_deg = std::acos(cos_theta) * 180.0f / M_PI;
                        
                        // 死区（5度）
                        const float ANGEL_DEADZONE = 5.0f;
                        if (angle_deg > config.ANGEL_FILTER_LIMIT + ANGEL_DEADZONE){
                            psd2vcu.FusionSlotInfo[i].slotStatusType = 4;
                        }

                        LOGD("[VCU NOTRELEASE2 anglelimit] ID: %d, angle_deg: %.f", psd2vcu.FusionSlotInfo[i].slotLabel, angle_deg);
                    } else {
                        LOGD("VCU NOTRELEASE2 anglelimit] ID: %d, DIAGONAL SLOT NO LIMIT.", psd2vcu.FusionSlotInfo[i].slotLabel);
                    }
                }
                
                // ***************************3 车位入口边宽度小于THRESHOLD 不释放
                LOGD("[VCU NOTRELEASE3 abnarrow] VCU_TOO_SMALL_FILTER: %d, VCU_TOO_SMALL: %.3f", config.VCU_TOO_SMALL_FILTER, config.VCU_TOO_SMALL);
                if (config.VCU_TOO_SMALL_FILTER){
                    float VCU_AB = sqrt(pow(psd2vcu.FusionSlotInfo[i].pt[0].x - psd2vcu.FusionSlotInfo[i].pt[1].x, 2) +
                                    pow(psd2vcu.FusionSlotInfo[i].pt[0].y - psd2vcu.FusionSlotInfo[i].pt[1].y, 2));
                    // 死区（0.1m）
                    const float TOO_SMALL_DEADZONE = 0.05f;
                    if (VCU_AB <= config.VCU_TOO_SMALL - TOO_SMALL_DEADZONE) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4;
                    } 
                    LOGD("[VCU NOTRELEASE3 abnarrow] ID: %d, VCU_AB: %.3f, VCU occupied: %d",
                        psd2vcu.FusionSlotInfo[i].slotLabel, VCU_AB, psd2vcu.FusionSlotInfo[i].slotStatusType);
                }

                // ***************************4 驶过水平车位，后轴中点到AD边VECTOR_THRESHOLD才可以释放
                LOGD("[VCU NOTRELEASE4 parallel] PARALLEL_VECTOR_FILTER: %d, PARALLEL_VECTOR_LIMIT: %f", config.PARALLEL_VECTOR_FILTER, config.PARALLEL_VECTOR_LIMIT);
                if (config.PARALLEL_VECTOR_FILTER){
                    if (psd_m_output.rectInfo.PStype == 1){
                        POINT_F VCU_car_rear_axle_center = { -4.064f, 0.0f };

                        float vector_carrearaxlecenter2parallelAD = (
                            (psd2vcu.FusionSlotInfo[i].pt[3].y - psd2vcu.FusionSlotInfo[i].pt[0].y) * VCU_car_rear_axle_center.x
                        - (psd2vcu.FusionSlotInfo[i].pt[3].x - psd2vcu.FusionSlotInfo[i].pt[0].x) * VCU_car_rear_axle_center.y
                        + psd2vcu.FusionSlotInfo[i].pt[3].x * psd2vcu.FusionSlotInfo[i].pt[0].y
                        - psd2vcu.FusionSlotInfo[i].pt[3].y * psd2vcu.FusionSlotInfo[i].pt[0].x
                        ) / sqrt(
                            pow(psd2vcu.FusionSlotInfo[i].pt[3].y - psd2vcu.FusionSlotInfo[i].pt[0].y, 2)
                        + pow(psd2vcu.FusionSlotInfo[i].pt[3].x - psd2vcu.FusionSlotInfo[i].pt[0].x, 2)
                        );
                        // 死区（0.2m）
                        const float PARALLEL_VECTOR_DEADZONE = 0.1f;
                        if (psd2vcu.FusionSlotInfo[i].pt[0].y >= 0){ // 右侧
                            if (vector_carrearaxlecenter2parallelAD <= config.PARALLEL_VECTOR_LIMIT - PARALLEL_VECTOR_DEADZONE){ // -0.8
                                psd2vcu.FusionSlotInfo[i].slotStatusType = 4;
                            }
                        }
                        else{
                            if (vector_carrearaxlecenter2parallelAD >= config.PARALLEL_VECTOR_LIMIT - PARALLEL_VECTOR_DEADZONE){ // -0.8
                                psd2vcu.FusionSlotInfo[i].slotStatusType = 4;
                            }
                        }
                        LOGD("[VCU NOTRELEASE4 parallel] PARALLEL VECTOR: %f, ID: %d, set to %d", vector_carrearaxlecenter2parallelAD, psd2vcu.FusionSlotInfo[i].slotLabel, psd2vcu.FusionSlotInfo[i].slotStatusType);
                    } else {
                        LOGD("[VCU NOTRELEASE4 parallel] NOT PARALLEL SLOT.");
                    }
                }
                
                // 障碍物属性
                psd2vcu.FusionSlotInfo[i].stopperInSlot = psd_m_output.rectInfo.StopperInSlot;
                //地锁
                psd2vcu.FusionSlotInfo[i].lockInSlot = psd_m_output.rectInfo.LockInSlot;
                if (psd_m_output.rectInfo.iSodType == 1 && psd_m_output.rectInfo.LockInSlot == 1){
                    psd2vcu.FusionSlotInfo[i].slotInnerObType = 3;
                    psd2vcu.FusionSlotInfo[i].lockLocation = 3;
                }
                //锥桶/禁停牌
                if (psd_m_output.rectInfo.iSodType == 1 && psd_m_output.rectInfo.OBSInSlot == 1){
                    psd2vcu.FusionSlotInfo[i].slotInnerObType = 2;
                }
                //车
                if (psd_m_output.rectInfo.iSodType == 1 && psd_m_output.rectInfo.OBSInSlot != 1 && psd_m_output.rectInfo.LockInSlot != 1 && psd_m_output.rectInfo.StopperInSlot != 1){
                    psd2vcu.FusionSlotInfo[i].slotInnerObType = 1;
                }

                psd2vcu.FusionSlotInfo[i].backInAvailableFlag = 1;
                psd2vcu.FusionSlotInfo[i].parkInHeadInSoftButtonCurrentValue = 1;
                psd2vcu.FusionSlotInfo[i].timeStamp = (current1970_ms >= 0) ? static_cast<uint64_t>(current1970_ms) : 0;
                i++;
            }

            // ================= 推荐逻辑 =================
            // 认为自车的位置
            POINT_F VCU_car_pose = { -4.0f, 0.0f };

            // Step 0: 清除旧的推荐信息
            for (int i = 0; i < psd2vcu.slotNum; ++i) {
                if (psd2vcu.FusionSlotInfo[i].slotStatusType == 7) {
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 3;
                }
                psd2vcu.FusionSlotInfo[i].displayLabel = 0;
            }

            std::vector<Sfus::FusionSlotInfo> vcu_available_slots; //找出available车位
            std::vector<Sfus::FusionSlotInfo> cloest_slots; // 从psd2vcu拿到,找出available里的closet车位
            for (int icnt = 0; icnt < psd2vcu.slotNum; ++icnt){
                if (psd2vcu.FusionSlotInfo[icnt].slotStatusType == 3){
                    Sfus::FusionSlotInfo vcu_slot;
                    for (int jcnt = 0; jcnt < 4; ++jcnt){
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
            
            
            int available_slot_flag_to_statemachine = 0;
            if (vcu_available_slots.size() > 0){
                available_slot_flag_to_statemachine = 1;
            } else {
                available_slot_flag_to_statemachine = 0;
            }
            
            // 更新available_slot_flag到状态管理器
            currentState.available_slot_flag_to_statemachine = available_slot_flag_to_statemachine;
            
            cloest_slots = math::findClosesParkingSpots(VCU_car_pose, vcu_available_slots, 10); //距离排序后的slots
            LOGD("cloest_slots size: %d", cloest_slots.size());

            if (final_select_ID == 0 && is_Still) { //状态1：当没有点选ID且静止，使用推荐ID
                LOGD("RECOMMEND1: still, Start Recommend!")
                const int max_recommend_num = 3; //display设置为1，2，3

                // Step 1：设置 cloest_slots[0] 对应 slot 的 slotStatusType 为 7
                if (!cloest_slots.empty()) {
                    int targetLabel = cloest_slots[0].slotLabel;
                    currentState.RECOMMEND_ID = targetLabel;
                    
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
                int final_ID = RecommendSelectID(final_select_ID, currentState.RECOMMEND_ID);
                currentState.final_ID = final_ID;
            }
            else if (final_select_ID == 0 && !is_Still) { //状态2：当没有点选ID且运动，保留RD原状态
                LOGD("RECOMMEND2: not still, NO Recommend!")
                currentState.RECOMMEND_ID = 0;
                currentState.final_select_ID = 0;
                currentState.final_ID = 0;
                currentState.recommend_exist = false;
                currentState.already_has_recommend_slot = false;
                
                for (int i = 0; i < slotlist_size; i++) {
                    psd2vcu.FusionSlotInfo[i].displayLabel = 0;
                    if (psd2vcu.FusionSlotInfo[i].slotStatusType == 4) { //占用的保持占用
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
                    }
                    else if (psd2vcu.FusionSlotInfo[i].slotStatusType == 6){ //unavailable的保持unavailable
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 6;
                    }
                    else { //剩下的回到available
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 设置为AVAILABLE状态
                    }
                }
                memset(&psd2statemachine, 0, sizeof(StatusDecFusionInput));
            }
            else if (final_select_ID != 0 && is_Still) { //状态3：当有点选车位且静止，使用点选ID
                LOGD("RECOMMEND3: still, Select!")
                currentState.RECOMMEND_ID = 0;
                currentState.final_ID = final_select_ID;
                currentState.already_has_recommend_slot = false;
                
                for (int i = 0; i < slotlist_size; i++) {
                    psd2vcu.FusionSlotInfo[i].displayLabel = 0;
                    //找到目标车位ID 且 非占用
                    if (psd2vcu.FusionSlotInfo[i].slotLabel == currentState.final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType != 4) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 5; // 设置为SELECTED状态
                    } 
                    //找到目标车位ID 且 占用
                    if (psd2vcu.FusionSlotInfo[i].slotLabel == currentState.final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType == 4) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
                    } 
                    //剩下的非选中车位，占用的保持占用
                    else if (psd2vcu.FusionSlotInfo[i].slotLabel != currentState.final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType == 4) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
                    }
                    //剩下的非选中车位，unavailable的保持unavailable
                    else if (psd2vcu.FusionSlotInfo[i].slotLabel != currentState.final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType == 6) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 6; // 设置为unavailable状态
                    }
                    //剩下的非选中车位，不占用,不unavailable的回到available
                    else if (psd2vcu.FusionSlotInfo[i].slotLabel != currentState.final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType != 4) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 设置为AVAILABLE状态
                    }
                }
            }
            else { //状态4：当有点选车位且运动，清除所有ID。@TODO 前后距离超过一定值
                LOGD("RECOMMEND4: no still, no recommend, no select")
                currentState.HMI_temp_ID = 0;
                currentState.HMI_select_ID = 0;
                currentState.VCU_select_ID_ON = 0;
                currentState.final_select_ID = 0;
                currentState.RECOMMEND_ID = 0;
                currentState.final_ID = 0;
                currentState.recommend_exist = false;
                currentState.already_has_recommend_slot = false;
                
                for (int i = 0; i < slotlist_size; i++) {
                    psd2vcu.FusionSlotInfo[i].displayLabel = 0;
                    if (psd2vcu.FusionSlotInfo[i].slotStatusType == 4) { //占用的保持占用
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
                    }
                    else if (psd2vcu.FusionSlotInfo[i].slotStatusType == 6){ //unavailable的保持unavailable
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 6;
                    }
                    else { //不占用的回到available
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 设置为AVAILABLE状态
                    }
                }
                memset(&psd2statemachine, 0, sizeof(StatusDecFusionInput));
                memset(&psd2planning.targetSlot, 0, sizeof(Sfus::SfusionSlots));
            }

            // 更新状态到配置管理器
            configManager.updateGlobalState(currentState);

            //final_ID 已记忆，每次清零全列表，并mark此车位。必须search时打标记
            PSD_FusionModuleIFrunable.markParkInSlot(outputSlot_FUSED, currentState.final_ID);
            LOGD("After markParkInSlot FUSIONSLOTS:")
            LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");

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
                if (slot.rectInfo.label == currentState.final_ID) {
                    selected_slot_in_world = slot; 
                    break;
                }
            }

            // selected_slot_in_world 为世界坐标下的目标车位
            LOGD("[TARGET SLOT WORLD] final_ID: %d. (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)",
                currentState.final_ID,
                selected_slot_in_world.rectInfo.pt[0].x,
                selected_slot_in_world.rectInfo.pt[0].y,
                selected_slot_in_world.rectInfo.pt[1].x,
                selected_slot_in_world.rectInfo.pt[1].y,
                selected_slot_in_world.rectInfo.pt[2].x,
                selected_slot_in_world.rectInfo.pt[2].y,
                selected_slot_in_world.rectInfo.pt[3].x,
                selected_slot_in_world.rectInfo.pt[3].y)
        }

        // For Test VCU slot lists
        for (int icnt = 0; icnt < slotlist_size; icnt++){
            LOGD("[PSD2VCUSLOTLIST] apa_status: %d, slotsize: %d, TYPE: %d, STATUS: %d, ID: %d, displayID: %d, SelectedFlag: %d (%f,%f) (%f,%f) (%f,%f) (%f,%f), timestamp: %llu",
            apa_status,
            slotlist_size,
            psd2vcu.FusionSlotInfo[icnt].slotType,
            psd2vcu.FusionSlotInfo[icnt].slotStatusType,
            psd2vcu.FusionSlotInfo[icnt].slotLabel,
            psd2vcu.FusionSlotInfo[icnt].displayLabel,
            psd2vcu.FusionSlotInfo[icnt].slotSelectedFlag,
            psd2vcu.FusionSlotInfo[icnt].pt[0].x,
            psd2vcu.FusionSlotInfo[icnt].pt[0].y,
            psd2vcu.FusionSlotInfo[icnt].pt[1].x,
            psd2vcu.FusionSlotInfo[icnt].pt[1].y,
            psd2vcu.FusionSlotInfo[icnt].pt[2].x,
            psd2vcu.FusionSlotInfo[icnt].pt[2].y,
            psd2vcu.FusionSlotInfo[icnt].pt[3].x,
            psd2vcu.FusionSlotInfo[icnt].pt[3].y,
            psd2vcu.FusionSlotInfo[icnt].timeStamp);
        }

        if (apa_status != 1){
            LOGD("[RECOMMENDSELECTID] HMI %d, VCU %d, final select %d, recommend: %d, final_ID %d",
                 currentState.HMI_temp_ID, currentState.VCU_select_ID_ON, currentState.final_select_ID, 
                 currentState.RECOMMEND_ID, currentState.final_ID);
            EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
        }
    }

    else { //泊入过程中显示所有车位
        LOGD("VCU display For NON-SEARCH");
        memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector));
        psd2vcu.slotNum = slotlist_size;
        if (psd2vcu.slotNum > 0){
            int i = 0;
            LOGD("PSD2VCU apa_status: %d, outputslot_fused size: %d",apa_status,outputSlot_FUSED.slots_in_cur_frame.size());
            // 恢复目标车位标记
            PSD_FusionModuleIFrunable.restoreSelectedSlot(outputSlot_FUSED);
            LOGD("After restoreSelectedSlot FUSIONSLOTS (ParkInSlot REQUIRED):")
            LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");
            
            // 使用配置管理器中的车辆参数
            float vehicle_length = PSDConfigManager::getVehicleLength();
            float rear_axle_center = PSDConfigManager::getRearAxleCenterVehicleRear();
            float mm_to_m = PSDConfigManager::getMmToM();
            
            for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame){
                if (i >= slotlist_size || i >= 50){
                    LOGD("die in VCU and size is:",slotlist_size);
                    break;
                }

                psd2vcu.FusionSlotInfo[i].slotLabel = psd_m_output.rectInfo.label; //ID
                psd2vcu.FusionSlotInfo[i].displayLabel = 0;
                
                //ABCD顺序调整为VCU专用顺序
                //左侧
                if (psd_m_output.rectInfo.pt[0].x <= 0 || psd_m_output.rectInfo.pt[1].x <= 0 || psd_m_output.rectInfo.pt[2].x < 0){
                    psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[0].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[0].x / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[0].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[1].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[1].x / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[1].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[2].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[2].x / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[3].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[3].x / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
                } else { //右侧
                    psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[0].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[0].x / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[0].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[1].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[1].x / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[1].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[2].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[2].x / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[3].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[3].x / mm_to_m;
                    psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
                }

                // 状态判断
                if (psd_m_output.rectInfo.ParkInSlot == 1){
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 5;
                    psd2vcu.FusionSlotInfo[i].slotSelectedFlag = 1;
                } else {
                    psd2vcu.FusionSlotInfo[i].slotSelectedFlag = 0;
                    if (psd_m_output.rectInfo.iSodType == 1){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                    } else {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                    }
                }

                // 障碍物属性设置...
                psd2vcu.FusionSlotInfo[i].stopperInSlot = psd_m_output.rectInfo.StopperInSlot;
                psd2vcu.FusionSlotInfo[i].lockInSlot = psd_m_output.rectInfo.LockInSlot;
                if (psd_m_output.rectInfo.iSodType == 1 && psd_m_output.rectInfo.LockInSlot == 1){
                    psd2vcu.FusionSlotInfo[i].slotInnerObType = 3;
                    psd2vcu.FusionSlotInfo[i].lockLocation = 3;
                }
                if (psd_m_output.rectInfo.iSodType == 1 && psd_m_output.rectInfo.OBSInSlot == 1){
                    psd2vcu.FusionSlotInfo[i].slotInnerObType = 2;
                }
                if (psd_m_output.rectInfo.iSodType == 1 && psd_m_output.rectInfo.OBSInSlot != 1 && psd_m_output.rectInfo.LockInSlot != 1 && psd_m_output.rectInfo.StopperInSlot != 1){
                    psd2vcu.FusionSlotInfo[i].slotInnerObType = 1;
                }
                psd2vcu.FusionSlotInfo[i].backInAvailableFlag = 1;
                psd2vcu.FusionSlotInfo[i].parkInHeadInSoftButtonCurrentValue = 1;
                psd2vcu.FusionSlotInfo[i].timeStamp = (current1970_ms >= 0) ? static_cast<uint64_t>(current1970_ms) : 0;
                i++;
            }
        }
        
        for (int i = 0; i < slotlist_size; i++){
            LOGD("[PSD2VCUSLOTLIST] IN GUIDANCE, Slot#%d, type: %d, Selected: %d, (%f,%f) (%f,%f) (%f,%f) (%f,%f), timestamp: %llu",
            psd2vcu.FusionSlotInfo[i].slotLabel,
            psd2vcu.FusionSlotInfo[i].slotStatusType,
            psd2vcu.FusionSlotInfo[i].slotSelectedFlag,
            psd2vcu.FusionSlotInfo[i].pt[0].x,
            psd2vcu.FusionSlotInfo[i].pt[0].y,
            psd2vcu.FusionSlotInfo[i].pt[1].x,
            psd2vcu.FusionSlotInfo[i].pt[1].y,
            psd2vcu.FusionSlotInfo[i].pt[2].x,
            psd2vcu.FusionSlotInfo[i].pt[2].y,
            psd2vcu.FusionSlotInfo[i].pt[3].x,
            psd2vcu.FusionSlotInfo[i].pt[3].y,
            psd2vcu.FusionSlotInfo[i].timeStamp);
        }
        EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
    }

    //***********************************APAHANDLE 发送车位列表
    psd2location.slotNum = slotlist_size;
    if (psd2location.slotNum > 0){
        int j = 0;
        
        for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame){
            if (j >= slotlist_size || j >= 50){
                std::cout<<"die in APAhandle and size is:"<<slotlist_size<<std::endl;
                break;
            }
            psd2location.fusionSlotInfo[j].slotLabel = psd_m_output.rectInfo.label; //ID
            psd2location.fusionSlotInfo[j].slotType = slottype_rd2vcu(psd_m_output.rectInfo.PStype);
            if (psd_m_output.rectInfo.iMaterial == 1){
                psd2location.fusionSlotInfo[j].fusionSlotType = 3;
            } else {
                if (psd_m_output.rectInfo.label >= 1000 && psd_m_output.rectInfo.label < 10000){
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
            if (j == 0){
                psd2location.fusionSlotInfo[j].displayLabel = currentState.mirror_fold_flag;
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
        if (parkout_flag != 1){
            EMC_psd_fusion_process_SetFieldFusionSlotInfo2Location(psd2location);
        }
    }

    //***********************************APAHANDLE 发送目标车位ID
    Fsm::Slotlabel psd2apahandel_targetID;
    if (currentState.final_ID){
        psd2apahandel_targetID.targetSlotLabel = currentState.final_ID;
        if (parkout_flag != 1){
            EMC_psd_fusion_process_SetFieldSlotlabel(psd2apahandel_targetID);
        }
        LOGD("[PSD2APAHANDLE] target slot id: %d", currentState.final_ID);
    }

    //***********************************PLANNING/HMI 发送车位列表
    //check psd output to planning(2 target slot)
    //拿到目标车位ID后，发送目标车位信息给planning
    int target_slot_fusionSlotType = 0;

    auto targetMemory = configManager.getTargetMemory();
    
    // if (currentState.final_ID > 0 && apa_status != 5 && parkout_flag != 1){ //进入guidance后固定目标车位角点
    if (currentState.final_ID > 0 && parkout_flag != 1){ //进入guidance后持续更新目标车位
    // if (currentState.final_ID > 0 && parkout_flag != 1 && (apa_status != 5 || (apa_status == 5 && !configManager.isTargetSlotUpdatedOnce()))){ //进入guidance后只更新一次目标车位

        //SEARCH阶段持续更新
        if (apa_status != 5){ 
            for (int i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size(); ++i){
                if (currentState.final_ID == outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label){ // 找到目标车位
    
                    // *******************正逆鱼骨，车位类型*******************
                    double ABx = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x;
                    double ABy = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                    double ADx = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x;
                    double ADy = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                    double dotProduct = (ABx * ADx) + (ABy * ADy);
                    double magnitudeAB = sqrt(ABx * ABx + ABy * ABy);
                    double magnitudeAD = sqrt(ADx * ADx + ADy * ADy);
                    double cosTheta = dotProduct / (magnitudeAB * magnitudeAD);
                    double angleRadians = acos(cosTheta);
                    double angleDegrees = angleRadians * (180.0 / M_PI);
                    
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
                    targetMemory.slot_type_before_update = psd2planning.targetSlot.slotType;
                    
                    // 目标车位转世界坐标系
                    POINT_I ptA = {psd2planning.targetSlot.slotCorners.cornerA.x, psd2planning.targetSlot.slotCorners.cornerA.y};
                    POINT_I ptB = {psd2planning.targetSlot.slotCorners.cornerB.x, psd2planning.targetSlot.slotCorners.cornerB.y};
                    POINT_I ptC = {psd2planning.targetSlot.slotCorners.cornerC.x, psd2planning.targetSlot.slotCorners.cornerC.y};
                    POINT_I ptD = {psd2planning.targetSlot.slotCorners.cornerD.x, psd2planning.targetSlot.slotCorners.cornerD.y};
                    targetMemory.search_target_center.x = (ptA.x + ptB.x + ptC.x + ptD.x) / 4;
                    targetMemory.search_target_center.y = (ptA.y + ptB.y + ptC.y + ptD.y) / 4;
                    targetMemory.search_target_center_world = Local2Global(targetMemory.search_target_center, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);

                    // *******************目标车位世界坐标系顺序记忆*******************
                    targetMemory.world_slot_memory[0] = Local2Global(ptA, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                    targetMemory.world_slot_memory[1] = Local2Global(ptB, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                    targetMemory.world_slot_memory[2] = Local2Global(ptC, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                    targetMemory.world_slot_memory[3] = Local2Global(ptD, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                    
                    LOGD("[PSD2PLANNING][UPDATE BEFORE GUIDANCE] world_slot_memory: A(%d,%d), B(%d,%d), C(%d,%d), D(%d,%d)",
                            targetMemory.world_slot_memory[0].x, targetMemory.world_slot_memory[0].y,
                            targetMemory.world_slot_memory[1].x, targetMemory.world_slot_memory[1].y,
                            targetMemory.world_slot_memory[2].x, targetMemory.world_slot_memory[2].y,
                            targetMemory.world_slot_memory[3].x, targetMemory.world_slot_memory[3].y);

                    // *******************判定是否狭窄车位*******************
                    auto config = configManager.getConfig();
                    double dx = psd2planning.targetSlot.slotCorners.cornerB.x - psd2planning.targetSlot.slotCorners.cornerA.x;
                    double dy = psd2planning.targetSlot.slotCorners.cornerB.y - psd2planning.targetSlot.slotCorners.cornerA.y;
                    double AB_dist = sqrt(dx * dx + dy * dy);
                    
                    LOGD("isNarrow: %d, AB_dist: %f", currentState.isNarrow, AB_dist);
                    if (AB_dist <= config.NARROWSLOT_THRESHOLD - 100) {
                        currentState.isNarrow = true;
                    } else if (AB_dist > config.NARROWSLOT_THRESHOLD + 100) {
                        currentState.isNarrow = false;
                    }
                    
                    //********************车位来源******************
                    psd2planning.targetSlot.stopper_Dis = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.StopperDistance;
                    if (currentState.final_ID >= 1000 && currentState.final_ID < 10000){
                        psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
                    } else if (currentState.final_ID >= 10000){
                        psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_USS;
                    } else {
                        psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
                    }
                    
                    //*****************无车位材质接口，借用，0视觉1超声波3草砖*********************
                    if (outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.iMaterial == 1){
                        target_slot_fusionSlotType = 3;
                    } else {
                        if (outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label >= 1000 && outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label < 10000){
                            target_slot_fusionSlotType = 0;
                        } else {
                            target_slot_fusionSlotType = 1;
                        }
                    }
                    
                    // 更新记忆到配置管理器
                    configManager.updateTargetMemory(targetMemory);
                    break;
                }
            }
        }
        
        else if (apa_status == 5){ 
            LOGD("[PSD2PLANNING][UPDATE IN GUIDANCE] world_slot_memory: A(%d,%d), B(%d,%d), C(%d,%d), D(%d,%d)",
                            targetMemory.world_slot_memory[0].x, targetMemory.world_slot_memory[0].y,
                            targetMemory.world_slot_memory[1].x, targetMemory.world_slot_memory[1].y,
                            targetMemory.world_slot_memory[2].x, targetMemory.world_slot_memory[2].y,
                            targetMemory.world_slot_memory[3].x, targetMemory.world_slot_memory[3].y);

            for (int i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size(); ++i){
                if ((currentState.final_ID == outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label)){ // 找到目标车位

                    //********************限位块距离******************
                    psd2planning.targetSlot.stopper_Dis = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.StopperDistance;

                    if (!configManager.isTargetSlotUpdatedOnce() && targetMemory.slot_type_before_update == 1){ //进入guidance后只更新一次目标车位
        
                        // *******************正逆鱼骨，车位类型*******************
                        double ABx = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x;
                        double ABy = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                        double ADx = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x;
                        double ADy = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y - outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                        double dotProduct = (ABx * ADx) + (ABy * ADy);
                        double magnitudeAB = sqrt(ABx * ABx + ABy * ABy);
                        double magnitudeAD = sqrt(ADx * ADx + ADy * ADy);
                        double cosTheta = dotProduct / (magnitudeAB * magnitudeAD);
                        double angleRadians = acos(cosTheta);
                        double angleDegrees = angleRadians * (180.0 / M_PI);
                        
                        if (angleDegrees > 80 || angleDegrees < 100) {
                            psd2planning.targetSlot.slotType = slottype_rd2decplan(outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.PStype);
                        } else if (angleDegrees <= 80) {
                            psd2planning.targetSlot.slotType = Sfus::SLOTTYP_RFOBL;
                        } else {
                            psd2planning.targetSlot.slotType = Sfus::SLOTTYP_OBL;
                        }

                        // *******************判定是否狭窄车位*******************
                        auto config = configManager.getConfig();
                        double dx = psd2planning.targetSlot.slotCorners.cornerB.x - psd2planning.targetSlot.slotCorners.cornerA.x;
                        double dy = psd2planning.targetSlot.slotCorners.cornerB.y - psd2planning.targetSlot.slotCorners.cornerA.y;
                        double AB_dist = sqrt(dx * dx + dy * dy);
                        
                        LOGD("isNarrow: %d, AB_dist: %f", currentState.isNarrow, AB_dist);
                        if (AB_dist <= config.NARROWSLOT_THRESHOLD - 100) {
                            currentState.isNarrow = true;
                        } else if (AB_dist > config.NARROWSLOT_THRESHOLD + 100) {
                            currentState.isNarrow = false;
                        }
                        
                        //********************车位来源******************
                        if (currentState.final_ID >= 1000 && currentState.final_ID < 10000){
                            psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
                        } else if (currentState.final_ID >= 10000){
                            psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_USS;
                        } else {
                            psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
                        }
                        
                        //*****************无车位材质接口，借用，0视觉1超声波3草砖*********************
                        if (outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.iMaterial == 1){
                            target_slot_fusionSlotType = 3;
                        } else {
                            if (outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label >= 1000 && outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label < 10000){
                                target_slot_fusionSlotType = 0;
                            } else {
                                target_slot_fusionSlotType = 1;
                            }
                        }

                        // *******************目标车位更新一次*******************
                        // 锁定更新前的车位类型
                        if (targetMemory.slot_type_before_update != 0){  // 假设0是NULL值
                            psd2planning.targetSlot.slotType = targetMemory.slot_type_before_update;
                        }

                        // *******************检查四个角点是否在同相机*******************
                        currentState.target_slot_in_range = true;
                        POINT_I point_A = {outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y};
                        POINT_I point_B = {outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y};
                        POINT_I pointA_pixel = math::coordConvert_car_center_to_pixel(point_A);
                        POINT_I pointB_pixel = math::coordConvert_car_center_to_pixel(point_B);
                        
                        int camera_id_A, camera_id_B;
                        if (pointA_pixel.x > 0 && pointA_pixel.y > 0 && pointB_pixel.x > 0 && pointB_pixel.y > 0 && 
                            pointA_pixel.x <= 895 && pointA_pixel.y <= 895 && pointB_pixel.x <= 895 && pointB_pixel.y <= 895){
                            
                            camera_id_A = ipm_camera_id_image[pointA_pixel.y][pointA_pixel.x];
                            camera_id_B = ipm_camera_id_image[pointB_pixel.y][pointB_pixel.x];
                            
                            LOGD("[PSD2PLANNING][UPDATE IN GUIDANCE] single frame slot pointA (%d, %d), pointB (%d, %d)", pointA_pixel.x, pointA_pixel.y, pointB_pixel.x, pointB_pixel.y);
                            LOGD("[PSD2PLANNING][UPDATE IN GUIDANCE] single frame slot id: %d, camera_id_A = %d, camera_id_B = %d", currentState.final_ID, camera_id_A, camera_id_B);
                            
                            if (camera_id_A == 0 || camera_id_B == 0) {
                                currentState.target_slot_in_range = false;
                                continue;
                            }
                            if (camera_id_A != camera_id_B) {
                                currentState.target_slot_in_range = false;
                                continue;
                            }

                            LOGD("[PSD2PLANNING][UPDATE IN GUIDANCE] target slot in range: %d", currentState.target_slot_in_range);
                            
                            // 如果不在范围内，则跳过更新
                            if(!currentState.target_slot_in_range) {
                                continue;
                            }

                            // 新目标车位转世界坐标系
                            POINT_I ptA = {outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y};
                            POINT_I ptB = {outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y};
                            POINT_I ptC = {outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].y};
                            POINT_I ptD = {outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x, outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y};
                            targetMemory.new_target_center.x = (ptA.x + ptB.x + ptC.x + ptD.x) / 4;
                            targetMemory.new_target_center.y = (ptA.y + ptB.y + ptC.y + ptD.y) / 4;
                            targetMemory.new_target_center_world = Local2Global(targetMemory.new_target_center, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                            
                            // 计算新旧目标车位中心点的距离，如果小于阈值，则更新目标车位
                            float target_slot_diff = CalcDistance(targetMemory.search_target_center_world, targetMemory.new_target_center_world);
                            LOGD("[PSD2PLANNING][UPDATE IN GUIDANCE][TARGET_SLOT_CHECK] world center moved: %f, threshold: %f", target_slot_diff, TargetSlotMemory::MAX_SLOT_MOVE_DIST_MM);
                            
                            if (target_slot_diff < TargetSlotMemory::MAX_SLOT_MOVE_DIST_MM){
                                
                                // 根据世界坐标最小距离匹配角点顺序
                                POINT_I cur_local_pts[4] = {ptA, ptB, ptC, ptD};
                                POINT_I cur_world_pts[4];
                                for (int j = 0; j < 4; ++j) {
                                    cur_world_pts[j] = Local2Global(cur_local_pts[j], pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                                }

                                int matched_idx[4] = {-1, -1, -1, -1};
                                bool used_flag[4] = {false, false, false, false};

                                for (int k = 0; k < 4; ++k) {
                                    float min_dist = 1e9f;
                                    int best_j = -1;
                                    for (int j = 0; j < 4; ++j) {
                                        if (used_flag[j]) continue;
                                        float dx = static_cast<float>(targetMemory.world_slot_memory[k].x - cur_world_pts[j].x);
                                        float dy = static_cast<float>(targetMemory.world_slot_memory[k].y - cur_world_pts[j].y);
                                        float dist = std::sqrt(dx * dx + dy * dy);
                                        if (dist < min_dist) {
                                            min_dist = dist;
                                            best_j = j;
                                        }
                                    }
                                    matched_idx[k] = best_j;
                                    used_flag[best_j] = true;
                                }
                                
                                // 未排角点顺序，直接使用原始角点
                                psd2planning.targetSlot.slotCorners.cornerA.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x; 
                                psd2planning.targetSlot.slotCorners.cornerA.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                                psd2planning.targetSlot.slotCorners.cornerB.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x;
                                psd2planning.targetSlot.slotCorners.cornerB.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y;
                                psd2planning.targetSlot.slotCorners.cornerC.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].x;
                                psd2planning.targetSlot.slotCorners.cornerC.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].y;
                                psd2planning.targetSlot.slotCorners.cornerD.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x;
                                psd2planning.targetSlot.slotCorners.cornerD.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y;
                                
                                configManager.setTargetSlotUpdatedOnce(true);
                            }
                        }
                    }
                    break;
                }
            }
        }
        
        // 更新状态到配置管理器
        configManager.updateGlobalState(currentState);
        configManager.updateTargetMemory(targetMemory);
    }

    if (parkout_flag != 1){
        if (apa_status == 1 || apa_status == 6 || apa_status == 7 || apa_status == 0){
            configManager.setTargetSlotUpdatedOnce(false); // 重置标志位
            memset(&psd2planning, 0, sizeof(Sfus::Sfsuion2DecPlan));
            EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);
        } else {
            LOGD("[PSD2PLANNING] UPDATED: %d, TIMESTAMP: %llu, APASTATUS: %d, TARGET SLOT type: %d, source: %d, stopper dis: %f, (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
            configManager.isTargetSlotUpdatedOnce(),
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

            if (currentState.target_slot_in_range && apa_status == 5){
                POINT_I ptA = {psd2planning.targetSlot.slotCorners.cornerA.x, psd2planning.targetSlot.slotCorners.cornerA.y};
                POINT_I ptB = {psd2planning.targetSlot.slotCorners.cornerB.x, psd2planning.targetSlot.slotCorners.cornerB.y};
                POINT_I ptC = {psd2planning.targetSlot.slotCorners.cornerC.x, psd2planning.targetSlot.slotCorners.cornerC.y};
                POINT_I ptD = {psd2planning.targetSlot.slotCorners.cornerD.x, psd2planning.targetSlot.slotCorners.cornerD.y};
                POINT_I A_world = Local2Global(ptA, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I B_world = Local2Global(ptB, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I C_world = Local2Global(ptC, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I D_world = Local2Global(ptD, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                LOGD("[PSD2PLANNING] TARGET_SLOT_WORLD A(%d,%d), B(%d,%d), C(%d,%d), D(%d,%d)",
                    A_world.x, A_world.y, B_world.x, B_world.y, C_world.x, C_world.y, D_world.x, D_world.y);
            }
        }
    }


    //***********************************PERCEPTION 发送目标车位
    // 拿到目标车位后，发送给planning的目标车位信息，再给perception
    if (parkout_flag != 1){
        if (apa_status == 1 || apa_status == 6 || apa_status == 7 || apa_status == 0){
            configManager.setTargetSlotUpdatedOnce(false); // 重置标志位
            memset(&psd2planning, 0, sizeof(Sfus::Sfsuion2DecPlan));
            EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);
        } else {
            LOGD("[PSD2PLANNING] UPDATED: %d, TIMESTAMP: %llu, APASTATUS: %d, TARGET SLOT type: %d, source: %d, stopper dis: %f, (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
            configManager.isTargetSlotUpdatedOnce(),
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

            if (currentState.target_slot_in_range && apa_status == 5){
                POINT_I ptA = {psd2planning.targetSlot.slotCorners.cornerA.x, psd2planning.targetSlot.slotCorners.cornerA.y};
                POINT_I ptB = {psd2planning.targetSlot.slotCorners.cornerB.x, psd2planning.targetSlot.slotCorners.cornerB.y};
                POINT_I ptC = {psd2planning.targetSlot.slotCorners.cornerC.x, psd2planning.targetSlot.slotCorners.cornerC.y};
                POINT_I ptD = {psd2planning.targetSlot.slotCorners.cornerD.x, psd2planning.targetSlot.slotCorners.cornerD.y};
                POINT_I A_world = Local2Global(ptA, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I B_world = Local2Global(ptB, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I C_world = Local2Global(ptC, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I D_world = Local2Global(ptD, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                LOGD("[PSD2PLANNING] TARGET_SLOT_WORLD A(%d,%d), B(%d,%d), C(%d,%d), D(%d,%d)",
                    A_world.x, A_world.y, B_world.x, B_world.y, C_world.x, C_world.y, D_world.x, D_world.y);
            }
        }
    }

    //***********************************PERCEPTION 发送目标车位
    // 拿到目标车位后，发送给planning的目标车位信息，再给perception
    if (parkout_flag != 1){
        Sfus::SfusionSlots psd2perception;
        memset(&psd2perception, 0, sizeof(Sfus::SfusionSlots));
        if (psd2planning.targetSlot.slotCorners.cornerA.x != 0){
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
            
            psd2perception.flag_valid = currentState.mirror_fold_flag;//后视镜折叠状态
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

    //***********************************STATEMACHINE 交互
    
    psd2statemachine.aps_apaParkPlaceNum = slotlist_size;

    if (apa_status == 1 || apa_status == 6 || apa_status == 7){
        psd2statemachine.aps_apaParkType = 0;
        psd2statemachine.aps_apaParkPlaceNum = 0;
        psd2statemachine.aps_apaHighlightSlot = 0;
        
        // 清零状态
        currentState.final_ID = 0;
        currentState.HMI_temp_ID = 0;
        configManager.updateGlobalState(currentState);
    }
    
    if (currentState.final_ID > 0){ //有点选或推荐
        psd2statemachine.aps_apaParkType = slottype_decplan2statemachine(psd2planning.targetSlot.slotType);
        if (currentState.final_ID >= 10000){
            psd2statemachine.aps_apaParkFusionType = 1;
        } else {
            psd2statemachine.aps_apaParkFusionType = 0;
        }

        if (slotlist_size > 0){
            psd2statemachine.aps_apaHighlightSlot = 1;
        }

        psd2statemachine.aps_apaAvailableSlot = 1;
        
        // 设置窄车位标志
        psd2statemachine.aps_apaNarrowSlot = currentState.isNarrow ? 1 : 0;

    } else { //无点选或推荐
        psd2statemachine.aps_apaParkType = 0;
        psd2statemachine.aps_apaHighlightSlot = 0;
        psd2statemachine.aps_apaAvailableSlot = currentState.available_slot_flag_to_statemachine;
        psd2statemachine.aps_apaNarrowSlot = 0;
    }
    
    LOGD("[PSD2STATEMACHINE] SELECT ID: %d, ParkType = %d, ParkFusionType, %d, NarrowSlot: %d, ParkPlaceNum: %d, AvailableSlot: %d, HighlightSlot: %d",
    currentState.final_ID,
    psd2statemachine.aps_apaParkType,
    psd2statemachine.aps_apaParkFusionType,
    psd2statemachine.aps_apaNarrowSlot,
    psd2statemachine.aps_apaParkPlaceNum,
    psd2statemachine.aps_apaAvailableSlot,
    psd2statemachine.aps_apaHighlightSlot);
    
    S2S_MCore_Bridge_SetSigStatusDecFusionInput(&psd2statemachine);

    //***********************************USS 发送目标车位ID
    short targetUssSlotID = 0;

    if (currentState.final_ID >= 10000) {
        if (currentState.final_ID > SHRT_MAX) {
            LOGD("Error: final_ID exceeds short range!\n");
        } else {
            targetUssSlotID = (short) currentState.final_ID;
            S2S_MCore_Bridge_SetSigtargetUssSlotLabel(&targetUssSlotID);
        }
    }

    //***********************************Control 发送限位块信息
    memset(&psd2control, 0, sizeof(APAControlBumpInput));
    if (currentState.final_ID != 0){
        for (int i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size(); ++i){
            if (outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label == currentState.final_ID){
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

    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOGD("[TIMECOST]Timetrigger100 time is: %d", elapsed.count());

    RETURN_NOERROR;
}


