#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "state_client.hpp"
#include "ConfigManager.h"
#include <iostream>
#include <typeinfo>
#include <float.h>
#include "math.hpp"

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
    // Load Config
    if (!ConfigManager::getInstance().loadFromFile("/app/neo/psd_config.json")) {
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
    ClearHistoricalSlots(apa_status);
    ClearHistoricalSlotsOnceSearch(apa_status, has_cleared_for_SEARCH_once);
    UpdateParkoutAndStill(apa_status, dr_pose, previous_dr_pose, parkout_flag, is_Still);
    if (mirror_fold_flag == 1) {
        HandleMirrorFold(singleframeslots, singleframeslotsID, rd_info);
    }
    CheckSlotStatus(singleframeslots, outputSlot_VIS, outputSlot_USS, outputSlot_FUSED);

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
    PSDConfigUtils::LogSlotInfo(outputSlot_USS, "ORIGIN USSSLOTS");

    // outputSlot_FUSED优化：融合后统计车位数
    slotlist_size = outputSlot_FUSED.slots_in_cur_frame.size();
    LOGD("After VIS/USS Merge FUSIONSLOTS:")
    PSDConfigUtils::LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");

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
    //VCU 发送车位列表
    apaSlotInfo selected_slot_in_world;
    ProcessPSD2VCU(apa_status, current1970_ms, selected_slot_in_world, psd2vcu, slotlist_size, is_Still, outputSlot_FUSED, PSD_FusionModuleIFrunable);

    //APAHANDLE 发送车位列表
    ProcessPSD2APAHandle(current1970_ms, psd2location, slotlist_size, outputSlot_FUSED, mirror_fold_flag, parkout_flag);

    //APAHANDLE 发送目标车位ID
    ProcessPSD2APAHandleTargetID(final_ID, parkout_flag);

    //PLANNING/HMI 发送车位列表
    ProcessPSD2Planning(current1970_ms, psd2planning, slotlist_size, outputSlot_FUSED);

    //PLANNING 发送目标车位
    int target_slot_fusionSlotType = 0;
    ProcessPSD2PlanningTargetSlot(current1970_ms, target_slot_fusionSlotType, final_ID, apa_status, parkout_flag, psd2planning, outputSlot_FUSED, pose_globaldata, ipm_camera_id_image);

    //PERCEPTION 发送目标车位
    ProcessPSD2Perception(current1970_ms, target_slot_fusionSlotType, parkout_flag, psd2planning, mirror_fold_flag);

    //STATEMACHINE 交互
    ProcessPSD2StateMachine(final_ID, slotlist_size, apa_status, psd2statemachine, psd2planning, available_slot_flag_to_statemachine);

    //USS 发送目标车位ID
    ProcessPSD2USS(final_ID);

    //Control 发送限位块信息
    ProcessPSD2Control(final_ID, parkout_flag, outputSlot_FUSED, psd2control);


    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOGD("[TIMECOST]Timetrigger100 time is: %d",elapsed.count());

    RETURN_NOERROR;
}


