#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "state_client.hpp"
#include <iostream>
#include <typeinfo>
#include "math.hpp"
#include "GetInput.h"
#include "utils.h"
#include <float.h>
#include <fstream>
#include "json.hpp"


using json = nlohmann::json;

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

tResult cpsd_fusion_process::Init()
{
    LOGW("PSD Process Start Success!");
    
    auto& configManager = PSDConfigManager::getInstance();
    
    // 读取配置文件
    if (!configManager.loadConfigFromFile("/app/neo/psd_config.json")) {
        LOGD("Load config failed!");
    }

    // 使用重构后的函数读取ipm相机id图
    ipm_camera_id_image.resize(896, std::vector<int>(896, 0));
    if (!PSDInputManager::loadIPMCameraIdFromCSV("/app/neo/ipm_camera_id.csv", ipm_camera_id_image)) {
        LOGW("Failed to load IPM camera ID image, using default values");
    }

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

    // ------------------------------------------------------------
    // 行泊切换
    // ------------------------------------------------------------
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

    LOGD("PSD Version: 07271525 emos10.0.1 [LYK] REBUILD");


    // ------------------------------------------------------------
    // Get Input
    // ------------------------------------------------------------
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



    // ------------------------------------------------------------
    // Input Check
    // ------------------------------------------------------------
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

    


    // ------------------------------------------------------------
    // VIS/USS Fusion
    // ------------------------------------------------------------
    //Yukan: Convert Slot using Mapinfo
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

    //USS
    UssIf_stPLVOutputInfo_t uss_info;
    auto uss_info_restruct = uss_info;
    S2S_MCore_Bridge_GetSigUssIf_stPLVOutputInfo(&uss_info);
    fusionslot.fillVisonstruct(uss_info, outputSlot_USS);
    fusionslot.clearInvalidUSSslots(outputSlot_USS);
    fusionslot.postprocessUSSslots(uss_info_restruct);
    fusionslot.mergeSlotLists(outputSlot_USS, outputSlot_VIS, outputSlot_FUSED);



    // ------------------------------------------------------------
    // Slot Process
    // ------------------------------------------------------------
    auto currentState = configManager.getGlobalState();

    // outputSlot_FUSED：类型修正
    PSD_FusionModuleIFrunable.SlotTypeCorrect(outputSlot_FUSED);
    LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");

    // outputSlot_FUSED：限位块、地锁、其他障碍物
    PSD_FusionModuleIFrunable.StopperLockOBS(obs_info_get,outputSlot_FUSED, currentState.final_ID);
    LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");


    // ------------------------------------------------------------
    // Target Slot Selection Logic
    // ------------------------------------------------------------
    int final_select_ID = inputManager_.processHMIVCUSelection(
        currentState.HMI_temp_ID, 
        currentState.HMI_select_ID, 
        currentState.VCU_select_ID_ON
    );
    LOGD("[HMIVCUSELECT OUT] final_select_id: %d", final_select_ID);

    // 使用重构后的推荐选择处理
    int final_ID = inputManager_.processRecommendSelection(final_select_ID, currentState.RECOMMEND_ID);
    LOGD("[RECOMMEND SELECT] final_ID: %d", final_ID);

    // 使用重构后的泊出判断
    int parkout_flag = inputManager_.determineParkOutFlag(apa_status);
    LOGD("[PARKOUT] flag: %d", parkout_flag);

    // 使用重构后的静止判断
    Loc::App2emap_DR previous_dr_pose = configManager.getPreviousDRPose();
    int is_Still = inputManager_.detectVehicleStillState(dr_pose, previous_dr_pose);
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

    // ------------------------------------------------------------
    // Output
    // ------------------------------------------------------------ 

    //***********************************VCU 发送车位列表
    outputManager_.sendVCUSlotList(outputSlot_FUSED, apa_status, currentState, is_Still, current1970_ms);
    

    //***********************************VCU 标记目标车位
    auto updatedState = configManager.getGlobalState();
    PSD_FusionModuleIFrunable.markParkInSlot(outputSlot_FUSED, updatedState.final_ID);
    LOGD("After markParkInSlot FUSIONSLOTS:");
    LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");


    //***********************************APAHANDLE 发送车位列表和目标车位ID
    outputManager_.sendAPAHandleSlotInfo(outputSlot_FUSED, updatedState, parkout_flag);


    //***********************************PLANNING/HMI 发送车位列表
    outputManager_.sendPlanningAndPerceptionTargetSlot(outputSlot_FUSED, updatedState, pose_globaldata, 
                                                   apa_status, parkout_flag, current1970_ms, 
                                                   ipm_camera_id_image, psd2planning);

    //***********************************STATEMACHINE 交互
    outputManager_.sendStateMachineInfo(slotlist_size, apa_status, updatedState, psd2planning.targetSlot.slotType);

    //***********************************USS 发送目标车位ID  
    outputManager_.sendUSSTargetSlotID(updatedState.final_ID);                                                  

    //***********************************Control 发送限位块信息
    outputManager_.sendControlBumpInfo(outputSlot_FUSED, currentState.final_ID, parkout_flag);




    
    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOGD("[TIMECOST]Timetrigger100 time is: %d", elapsed.count());

    RETURN_NOERROR;
}


