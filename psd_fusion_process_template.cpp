#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "state_client.hpp"
#include <iostream>
#include <typeinfo>
#include "math.hpp"
#include "GetInput.hpp"
#include "utils.h"
#include "psd2vcu.h"

// ***************************标定量
#define VEHICLE_LENGTH 5259.9 
#define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7
#define MM_TO_M 1000.0

// ***************************配置文件修改的参数(@TODO：从配置文件读取后转成const)
bool DEBUG = false; //功能开关
 
// // ***************************输入的全局变量，用于ON方式获取
// //RD
// rd::QuadParkingSlots rd_info;
// unsigned long long singleframeslotsID;
// std::vector<padVisionSlotCoord> singleframeslots;
// //DR
// Loc::App2emap_DR dr_pose;
// padVehiclePose pose_globaldata;
// Loc::App2emap_DR previous_dr_pose = {0};
// bool is_Still = true;
// //perception
// Fus::PkEmapObs obs_info_on; // ON获取
// Fus::PkEmapObs obs_info_get;
// //statemachine
// StatusDecOutput apastatus_info;
// int apa_status = 0;
// StatusDecFusionOutput searchpark_info;
// int park_request = 0;
// const int search_interrupt = 0; //@TODO OTA1

// 输入的全局变量，用于GET方式获取
int apa_status = 0;
int park_request = 0;
const int search_interrupt = 0; //@TODO VC7 RELEASE
int is_Still;
Loc::App2emap_DR previous_dr_pose;


SaveFileToJson filetojson;

// ***************************输出的全局变量
slotfusion fusionslot;
StatusDecFusionInput psd2statemachine;
Sfus::Sfsuion2DecPlan psd2planning; //动态车位列表
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


int parkout_flag = 0; //当前是否为泊出
int mirror_fold_flag = 0; //后视镜是否被折叠


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
    if (!LoadFromFile("/app/neo/psd_config.json")) {
        LOGD("Load config failed!");
    }

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

bool cpsd_fusion_process::LoadFromFile(const std::string& filename){
    std::ifstream inFile(filename);
    if(!inFile.is_open()){
        DEBUG = false;
        LOGD("无法打开配置文件: %s, DEBUG: %d", filename.c_str(), DEBUG);
        return false;
    }

    try{
        json j;
        inFile >> j;

        //解析文件路径
        j.at("debug").at("save_to_json").get_to(DEBUG);
    }
    catch (json::exception& e) {
        LOGD("配置文件解析错误: %s", e.what());
        return false;
    }
    inFile.close();
    return true;
}

void cpsd_fusion_process::Slot2Global(Sfus::Sfsuion2DecPlan &slot, const float &x, const float &y, const float &yaw)
{
    float theta = yaw * acos(-1) / 180.;

    float slot_Apt_temp_x = slot.targetSlot.slotCorners.cornerA.x * cos(theta) + slot.targetSlot.slotCorners.cornerA.y * sin(theta) + x;
    float slot_Apt_temp_y = slot.targetSlot.slotCorners.cornerA.y * cos(theta) - slot.targetSlot.slotCorners.cornerA.x * sin(theta) + y;

    float slot_Bpt_temp_x = slot.targetSlot.slotCorners.cornerB.x * cos(theta) + slot.targetSlot.slotCorners.cornerB.y * sin(theta) + x;
    float slot_Bpt_temp_y = slot.targetSlot.slotCorners.cornerB.y * cos(theta) - slot.targetSlot.slotCorners.cornerB.x * sin(theta) + y;

    float slot_Cpt_temp_x = slot.targetSlot.slotCorners.cornerC.x * cos(theta) + slot.targetSlot.slotCorners.cornerC.y * sin(theta) + x;
    float slot_Cpt_temp_y = slot.targetSlot.slotCorners.cornerC.y * cos(theta) - slot.targetSlot.slotCorners.cornerC.x * sin(theta) + y;

    float slot_Dpt_temp_x = slot.targetSlot.slotCorners.cornerD.x * cos(theta) + slot.targetSlot.slotCorners.cornerD.y * sin(theta) + x;
    float slot_Dpt_temp_y = slot.targetSlot.slotCorners.cornerD.y * cos(theta) - slot.targetSlot.slotCorners.cornerD.x * sin(theta) + y;

    slot.targetSlot.slotCorners.cornerA.x = slot_Apt_temp_x;
    slot.targetSlot.slotCorners.cornerA.y = slot_Apt_temp_y;

    slot.targetSlot.slotCorners.cornerB.x = slot_Bpt_temp_x;
    slot.targetSlot.slotCorners.cornerB.y = slot_Bpt_temp_y;

    slot.targetSlot.slotCorners.cornerC.x = slot_Cpt_temp_x;
    slot.targetSlot.slotCorners.cornerC.y = slot_Cpt_temp_y;

    slot.targetSlot.slotCorners.cornerD.x = slot_Dpt_temp_x;
    slot.targetSlot.slotCorners.cornerD.y = slot_Dpt_temp_y;
}

void cpsd_fusion_process::Slot2Local(Sfus::Sfsuion2DecPlan &slot, const float &x, const float &y, const float &yaw)
{
    float theta = yaw * static_cast<float>(M_PI) / 180.0f;

    float tmp_A_x = slot.targetSlot.slotCorners.cornerA.x - x;
    float tmp_A_y = slot.targetSlot.slotCorners.cornerA.y - y;
    float tmp_B_x = slot.targetSlot.slotCorners.cornerB.x - x;
    float tmp_B_y = slot.targetSlot.slotCorners.cornerB.y - y;
    float tmp_C_x = slot.targetSlot.slotCorners.cornerC.x - x;
    float tmp_C_y = slot.targetSlot.slotCorners.cornerC.y - y;
    float tmp_D_x = slot.targetSlot.slotCorners.cornerD.x - x;
    float tmp_D_y = slot.targetSlot.slotCorners.cornerD.y - y;

    float slot_Apt_temp_x = tmp_A_x * cos(theta) - tmp_A_y * sin(theta);
    float slot_Apt_temp_y = tmp_A_x * sin(theta) + tmp_A_y * cos(theta);

    float slot_Bpt_temp_x = tmp_B_x * cos(theta) - tmp_B_y * sin(theta);
    float slot_Bpt_temp_y = tmp_B_x * sin(theta) + tmp_B_y * cos(theta);

    float slot_Cpt_temp_x = tmp_C_x * cos(theta) - tmp_C_y * sin(theta);
    float slot_Cpt_temp_y = tmp_C_x * sin(theta) + tmp_C_y * cos(theta);

    float slot_Dpt_temp_x = tmp_D_x * cos(theta) - tmp_D_y * sin(theta);
    float slot_Dpt_temp_y = tmp_D_x * sin(theta) + tmp_D_y * cos(theta);

    slot.targetSlot.slotCorners.cornerA.x = slot_Apt_temp_x;
    slot.targetSlot.slotCorners.cornerA.y = slot_Apt_temp_y;

    slot.targetSlot.slotCorners.cornerB.x = slot_Bpt_temp_x;
    slot.targetSlot.slotCorners.cornerB.y = slot_Bpt_temp_y;

    slot.targetSlot.slotCorners.cornerC.x = slot_Cpt_temp_x;
    slot.targetSlot.slotCorners.cornerC.y = slot_Cpt_temp_y;

    slot.targetSlot.slotCorners.cornerD.x = slot_Dpt_temp_x;
    slot.targetSlot.slotCorners.cornerD.y = slot_Dpt_temp_y;
}

int cpsd_fusion_process::HMIVCUSelect(int &hmi_temp, const int &hmi_select, const int &vcu_select)
{
    LOGD("[HMIVCUSELECT IN] HMI:%d HMI temp:%d VCU:%d",hmi_select,hmi_temp,vcu_select);
    //HMI 部分
    //中间变量保存HMI发送的 [0 - ID - 0]，一秒内发送五次
    if (hmi_select){
        hmi_temp = hmi_select;
    }
    //VCU 部分，已从GET获取
    //VCU接收的点选车位 与 HMI接收的点选车位 二选一
    if (vcu_select != 0 && hmi_temp == 0) {
        final_select_ID = vcu_select; 
    } 
    else if (vcu_select == 0 && hmi_temp != 0) {
        final_select_ID = hmi_temp;
    } 
    else if (vcu_select != 0 && hmi_temp != 0 && (vcu_select == hmi_temp)) {
        final_select_ID = hmi_temp;
    }
    else if (vcu_select == 0 && hmi_temp == 0){
        final_select_ID = 0;
    }
    else {
        final_select_ID = vcu_select;
    }
    //结合HMI和VCU，得到final_select_ID
    return final_select_ID;
}

int cpsd_fusion_process::RecommendSelectID(const int &final_select, const int &recommend)
{
    if (final_select){
        return final_select;
    }
    else{
        return recommend;
    }
}

int cpsd_fusion_process::IsParkOut(int apastatus)
{
    if (apastatus == 3){
        return 1;
    }
    else{
        return 0;
    }
}

int cpsd_fusion_process::IsStill(const Loc::App2emap_DR drpose, Loc::App2emap_DR& previous_drpose)
{
    static int no_change_count = 0;
    float epsilon = 3.0; // 设置阈值，可以根据需要调整
    bool has_changed = false; // 比较 drpose 和 previous_drpose 是否变化
    int still_threshold = 5; //静止阈值，连续多少次没有变化算静止
    LOGD("[STILL] dr: x:%f, y:%f, yaw: %f,previous: x:%f, y:%f, yaw:%f",
    drpose.x,
    drpose.y,
    drpose.canAng,
    previous_drpose.x,
    previous_drpose.y,
    previous_drpose.canAng);

    if (fabs(drpose.x - previous_drpose.x) > epsilon ||
        fabs(drpose.y - previous_drpose.y) > epsilon ||
        fabs(drpose.canAng - previous_drpose.canAng) > epsilon)
    {
        has_changed = true;
    }
    LOGD("[STILL] has_changed:%d",has_changed);

    // 如果没有变化，增加连续无变化计数
    if (!has_changed)
    {
        no_change_count++;
    }
    else
    {
        no_change_count = 0;  // 有变化时重置计数器
    }
    LOGD("[STILL] no_change_count:%d",no_change_count);

    // 如果连续still_threshold次没有变化，则认为是静止状态
    if (no_change_count >= still_threshold)
    {
        previous_drpose = drpose; // 更新 previous_drpose 为当前的 drpose
        return 1;  // is_Still = 1
    }

    previous_drpose = drpose; // 更新 previous_drpose 为当前的 drpose
    return 0;  // is_Still = 0
}


tResult cpsd_fusion_process::TimeTrigger_thread_50ms_1()
{
    // auto start50 = std::chrono::steady_clock::now();
    // auto end50 = std::chrono::steady_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end50 - start50);
    // std::cout<<"[TIMECOST]Timetrigger50_1 time is:"<< elapsed.count() <<std::endl;
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
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnMapInfo(const Loc::MapInfo& userData)
{
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
    mirror_fold_flag = userData.mirrorFoldFlg;
    LOGD("[MIRRORFOLD] OnPlan2Psd mirror_fold_flag: %d !!!!",mirror_fold_flag);
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::TimeTrigger_thread_100ms_1()
{
	static kbd::sm::StateClient state_client;
    if (state_client.parking_stop()){
        LOGW("NOT IN PARKING STATE");
        RETURN_NOERROR;
    }



    
    auto current = std::chrono::system_clock::now(); 
    auto current1970 = current.time_since_epoch();
    auto current1970_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current1970).count(); //用于J5时间同步
    
    auto start = std::chrono::steady_clock::now(); // 用于计算TIMECOST

    LOGD("PSD Version: 03261927 emos9 add psd2control, rewrite RECOMMEND, add ego2slotcenter. ENABLE: NotToRelease(2000,9500), isNeedSingleframe2Update outputslot_fused order, occupy realtime update. DISABLE: psd2vcu fixed");
    
    // part2 输入，上游：RD, DR, USS, peception, VCU select ID, statemachine

    // GET方式获取
    rd::QuadParkingSlots rd_info;
    unsigned long long singleframeslotsID;
    std::vector<padVisionSlotCoord> singleframeslots;
    Loc::App2emap_DR dr_pose;
    padVehiclePose pose_globaldata;
    Fus::PkEmapObs obs_info_get;
    StatusDecOutput statemachine_info;

    GetInput getInput;
    getInput.GetAllInput();

    rd_info = getInput.rd_info;
    singleframeslotsID = getInput.singleframeslotsID;
    singleframeslots = getInput.singleframeslots;
    dr_pose = getInput.dr_pose;
    pose_globaldata = getInput.pose_globaldata;
    obs_info_get = getInput.obs_info_get;
    apa_status = getInput.apa_status;
    // search_interrupt = getInput.search_interrupt; //@TODO VC9 RELEASE

    parkout_flag = IsParkOut(apa_status);
    LOGD("[PARKOUT] flag: %d", parkout_flag);
    is_Still = IsStill(dr_pose,previous_dr_pose);
    LOGD("[STILL] is Still: %d", is_Still);

    // 时间同步 1500ms
    if (!CheckTimeSync(current1970_ms, rd_info.frameTimeStampNs, dr_pose.timeStamp)) {
        RETURN_NOERROR;
    }

    // RD拿到重复帧
    if (!CheckRDFrameTimestamp(rd_info.frameTimeStampNs)){
        RETURN_NOERROR;
    }

    // 折叠后视镜，清空单帧
    LOGD("[MIRRORFOLD] Flag: %d, singleframeslots.size: %d",mirror_fold_flag,singleframeslots.size());
    if (mirror_fold_flag == 1){
        rd_info.frameTimeStampNs = 0;
        rd_info.quadParkingSlotList.clear();
        singleframeslots.clear();
        singleframeslotsID = 0;
    }

    if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7){
        ClearParkingSlots(singleframeslots, singleframeslotsID, outputSlot_VIS, outputSlot_USS, outputSlot_FUSED, parkout_flag);
    }

    //***********************************check
    LOGD("[CHECK SIZE] CHECK singleframeslots size: %d",singleframeslots.size());
    LOGD("[CHECK SIZE] before update, vis: %d, uss: %d, fused: %d",outputSlot_VIS.slots_in_cur_frame.size()
                                                ,outputSlot_USS.slots_in_cur_frame.size()
                                                ,outputSlot_FUSED.slots_in_cur_frame.size());

    // part3 算法
    PSD_FusionModuleIFrunable.UpdateVechiclePose(pose_globaldata);
    PSD_FusionModuleIFrunable.UpdateVisionSlots(singleframeslotsID, singleframeslots, apa_status, search_interrupt);
    outputSlot_VIS = PSD_FusionModuleIFrunable.GetOutputSlot();

    //类型修正
    LOGD("Without SlotTypeCorrect VISSLOTSLIST:")
    LogSlotInfo(outputSlot_VIS, "ORIGIN VISSLOTS");
    PSD_FusionModuleIFrunable.SlotTypeCorrect(outputSlot_VIS);
    LOGD("After SlotTypeCorrect VISSLOTSLIST:")
    LogSlotInfo(outputSlot_VIS, "ORIGIN VISSLOTS");

    //限位块、地锁、其他障碍物
    LOGD("Without StopperLockOBS VISSLOTSLIST:")
    LogSlotInfo(outputSlot_VIS, "ORIGIN VISSLOTS");
    PSD_FusionModuleIFrunable.StopperLockOBS(obs_info_get,outputSlot_VIS);
    LOGD("After StopperLockOBS VISSLOTSLIST:")
    LogSlotInfo(outputSlot_VIS, "ORIGIN VISSLOTS");

        
    if (DEBUG == true){
        filetojson.SaveapaSlotListInfoToJson(outputSlot_VIS,"/userdata/psd/VISapaSlotListInfo.json");
    }

    if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7){
        ClearParkingSlots(singleframeslots, singleframeslotsID, outputSlot_VIS, outputSlot_USS, outputSlot_FUSED, parkout_flag);
    }


    //***********************************get USS
    UssIf_stPLVOutputInfo_t uss_info;
    S2S_MCore_Bridge_GetSigUssIf_stPLVOutputInfo(&uss_info);
    if (DEBUG == true){
        filetojson.SaveUssInfoToJson(uss_info,"/userdata/psd/USSapaSlotListInfo.json");
    }
    fusionslot.fillVisonstruct(uss_info, outputSlot_USS);
    auto uss_info_restruct = uss_info;
    fusionslot.postprocessUSSslots(uss_info_restruct);
    fusionslot.mergeSlotLists(outputSlot_USS, outputSlot_VIS, outputSlot_FUSED);

    // **************************outputSlot_FUSED优化：去除内部重叠车位
    PSD_FusionModuleIFrunable.removeOverlappingSlots(outputSlot_FUSED);
    slotlist_size = outputSlot_FUSED.slots_in_cur_frame.size();

    
    // *****************************outputSlot_FUSED优化：以单帧结果修复
    apaSlotListInfo singleframe_local_slots = math::ConvertSingeleframe2Local(singleframeslots);
    math::adjustOutputSlotFusedRectOrder(singleframe_local_slots);
    for (auto & slot : outputSlot_FUSED.slots_in_cur_frame){
        // LOGD("single_frame update! fused size:%d", outputSlot_FUSED.slots_in_cur_frame.size());
        // LOGD("single_frame update! singleframe_local_slots size:%d", singleframe_local_slots.slots_in_cur_frame.size());
        
        for (auto& single_frame_slot : singleframe_local_slots.slots_in_cur_frame){
            if(math::isNeedSingleframe2Update(slot, single_frame_slot)){
                // LOGD("single_frame update! apa_status:%d", apa_status);
                
                PSD_FusionModuleIFrunable.shrink_quad(single_frame_slot);
                for (int icnt = 0; icnt < 4; ++icnt){
                    LOGD("single_frame update! fused_slot(%d, %d)", slot.rectInfo.pt[icnt].x, slot.rectInfo.pt[icnt].y);
                    LOGD("single_frame update! single_slot(%d, %d)", single_frame_slot.rectInfo.pt[icnt].x, single_frame_slot.rectInfo.pt[icnt].y);

                    slot.rectInfo.pt[icnt].x = single_frame_slot.rectInfo.pt[icnt].x;
                    slot.rectInfo.pt[icnt].y = single_frame_slot.rectInfo.pt[icnt].y;
                }
                break;
            }else{
                continue;
            }
        }
    }

    // *****************************outputSlot_FUSED优化：标记入口边过窄的车位，用于不释放
    double AB_threshold = 2000.0;
    double faraway_threshold = 9500.0;

    LOGD("Without markNotToReleaseSlot FUSIONSLOTS:")
    LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");
    PSD_FusionModuleIFrunable.markNotToReleaseSlot(outputSlot_FUSED,AB_threshold,faraway_threshold);
    LOGD("After markNotToReleaseSlot FUSIONSLOTS:")
    LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");


    // *****************************outputSlot_FUSED优化：标记进入GUIDANCE时的目标车位
    PSD_FusionModuleIFrunable.markParkInSlot(outputSlot_FUSED,apa_status,final_ID);


    //******************************

    if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7){
        ClearParkingSlots(singleframeslots, singleframeslotsID, outputSlot_VIS, outputSlot_USS, outputSlot_FUSED, parkout_flag);
        dr_first = true;
        slotlist_size = 0;
    }

    
    // 输出USS FUSION车位列表
    LogSlotInfo(outputSlot_USS, "ORIGIN USSSLOTS");
    LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");





    //part4 目标车位ID处理。

    // VCU,HMI 双终端接收点选的目标车位
    final_select_ID = HMIVCUSelect(HMI_temp_ID,HMI_select_ID,VCU_select_ID_ON);
    LOGD("[HMIVCUSELECT OUT] final_select_id: %d",final_select_ID);

    // // 车动起来后，清除点选车位
    // if (!is_Still && apa_status == 2){
    //     HMI_temp_ID = 0;
    //     VCU_select_ID_ON = 0;
    //     final_select_ID = 0;
    // }

    // APAStatus == standby/finish/error时，清零车位ID
    // Clear, new
    if (apa_status == 1 || apa_status == 6 || apa_status == 7){
        ClearSelectRecommendSlot(HMI_temp_ID, HMI_select_ID, VCU_select_ID_ON, final_select_ID, RECOMMEND_ID, final_ID, apa_status);
        already_has_recommend_slot = false;
    }
    LOGD("[STATUSSELECT] HMI %d, VCU %d, final select %d",HMI_temp_ID,VCU_select_ID_ON,final_select_ID);








    // part5 输出。下游：VCU，APAHANDLE, PERCEPTION, VCU, PLANNING，STATEMACHINE, USS


    //***********************************VCU 发送车位列表
    //非GUIDANCE时，显示整个车位列表
    if (apa_status != 5){
        LOGD("The apa staus is not 5!");

        memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector)); //displayLabel 会被重置
        psd2vcu.slotNum = slotlist_size;

        if (psd2vcu.slotNum > 0){
            
            int i = 0;
            LOGD("PSD2VCU apa_status: %d, outputslot_fused size: %d",apa_status,outputSlot_FUSED.slots_in_cur_frame.size());
            
            for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame){
                if (i >= slotlist_size || i >= 50){
                    LOGD("die in VCU and size is:",slotlist_size);
                    break;
                }
                
                //psd2vcu.FusionSlotInfo[i].slotType = slottype_rd2vcu(psd_m_output.rectInfo.PStype);
                psd2vcu.FusionSlotInfo[i].slotLabel = psd_m_output.rectInfo.label; //ID
                

                //ABCD顺序调整为VCU专用顺序
                //左侧
                if (psd_m_output.rectInfo.pt[0].x <= 0 || psd_m_output.rectInfo.pt[1].x <= 0 || psd_m_output.rectInfo.pt[2].x < 0){
                    psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  ) / MM_TO_M; //mm 转 m , VCU坐标系上x右y
                    psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[0].x / MM_TO_M; //后轴中心转前保中心
                    psd2vcu.FusionSlotInfo[i].pt[0].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[1].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  ) / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[1].x/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[2].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[2].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[3].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[3].x/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[3].z = 0;

                    // 占用判断 + 忽略推荐车位
                    if (psd_m_output.rectInfo.iSodType == 1){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                    }
                    else if (psd_m_output.rectInfo.iSodType != 1 && psd_m_output.rectInfo.label == RECOMMEND_ID){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 7;
                    }
                    else if (psd_m_output.rectInfo.iSodType != 1 && psd_m_output.rectInfo.label != RECOMMEND_ID){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                    }
                    // // 0228ride临时占用判断：车头越过车位后才开始算占用非占用
                    // if (psd2vcu.FusionSlotInfo[i].pt[0].x >= -4.5){
                    //     psd2vcu.FusionSlotInfo[i].slotStatusType = 4;  // 被占用
                    // }
                    // else{
                    //     if (psd_m_output.rectInfo.iSodType == 1){
                    //         psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                    //     }
                    //     else{
                    //         psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                    //     }

                    // }
                    // LOGD("[VCU occupied] slot.x: %f, RD occupied: %d, VCU occupied: %d",
                    //     psd2vcu.FusionSlotInfo[i].pt[0].x,
                    //     psd_m_output.rectInfo.iSodType,
                    //     psd2vcu.FusionSlotInfo[i].slotStatusType);
                    

                    // // 原本的占用判断
                    // if (psd_m_output.rectInfo.iSodType == 1){
                    //     psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                    // }else {
                    //     psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                    // }

                    // Unavailable 车位判断
                    if (psd_m_output.rectInfo.NotToRelease == 1){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 6;
                    }


                    // VCU显示障碍物
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
                }
                //右侧
                else{
                    psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  )/ MM_TO_M; //mm 转 m
                    psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[0].x / MM_TO_M; //后轴中心转前保中心
                    psd2vcu.FusionSlotInfo[i].pt[0].z = 0;

                    psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[1].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[1].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].z = 0;

                    psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[2].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[2].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].z = 0;

                    psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[3].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[3].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
                    LOGD("[VCU occupied] isodtype:%d", psd_m_output.rectInfo.iSodType);



                    // 占用判断 + 忽略推荐车位
                    if (psd_m_output.rectInfo.iSodType == 1){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                    }
                    else if (psd_m_output.rectInfo.iSodType != 1 && psd_m_output.rectInfo.label == RECOMMEND_ID){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 7;
                    }
                    else if (psd_m_output.rectInfo.iSodType != 1 && psd_m_output.rectInfo.label != RECOMMEND_ID){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                    }
                    // // 0228ride临时占用判断：车头越过车位后才开始算占用非占用
                    // if (psd2vcu.FusionSlotInfo[i].pt[0].x > -4.5){
                    //     psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                    // }
                    // else{
                    //     if (psd_m_output.rectInfo.iSodType == 1){
                    //         psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                    //     }else {
                    //         psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                    //     }
                    // } 
                    // LOGD("[VCU occupied] slot.x: %f, RD occupied: %d, VCU occupied: %d",
                    //     psd2vcu.FusionSlotInfo[i].pt[0].x,
                    //     psd_m_output.rectInfo.iSodType,
                    //     psd2vcu.FusionSlotInfo[i].slotStatusType);


                    // // 原本的占用判断
                    // if (psd_m_output.rectInfo.iSodType == 1){
                    //     psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                    // }else {
                    //     psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                    // }


                    // Unavailable 车位判断
                    if (psd_m_output.rectInfo.NotToRelease == 1){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 6;
                    }





                    // VCU显示障碍物
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
                }
                i++;
            }

            // 认为自车的位置
            POINT_F VCU_car_pose;
            VCU_car_pose.x = -4.0;
            VCU_car_pose.y = 0.0;

            //0227 推荐车位可用
            // 从psd2vcu拿到最近的车位列表cloest_slots
            // std::vector<Fsm::FusionSlotInfo> cloest_slots;
            // std::vector<Fsm::FusionSlotInfo> vcu_available_slots;
            // for (int icnt = 0; icnt < psd2vcu.slotNum; ++icnt){
            //     if (psd2vcu.FusionSlotInfo[icnt].slotStatusType == 3){
            //         Fsm::FusionSlotInfo vcu_slot;
            //         for (int jcnt = 0; jcnt < 4; ++jcnt){
            //             vcu_slot.pt[jcnt].x = psd2vcu.FusionSlotInfo[icnt].pt[jcnt].x;
            //             vcu_slot.pt[jcnt].y = psd2vcu.FusionSlotInfo[icnt].pt[jcnt].y;
            //         }
            //         vcu_slot.slotLabel = psd2vcu.FusionSlotInfo[icnt].slotLabel;
            //         vcu_slot.slotStatusType = psd2vcu.FusionSlotInfo[icnt].slotStatusType;
            //         vcu_slot.slotType = psd2vcu.FusionSlotInfo[icnt].slotType;

            //         vcu_available_slots.push_back(vcu_slot);
            //     }
            // }
            // LOGD("vcu_available_slots size: %d",vcu_available_slots.size());
            // cloest_slots = math::findClosesParkingSpots(VCU_car_pose,vcu_available_slots ,10);
            // LOGD("cloest_slots size: %d",cloest_slots.size());

            // dev版本，更新成一样的结构体
            // 从psd2vcu拿到最近的车位列表cloest_slots
            std::vector<Sfus::FusionSlotInfo> vcu_available_slots; //找出available车位
            std::vector<Sfus::FusionSlotInfo> cloest_slots; // 找出available里的closet车位
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
            LOGD("vcu_available_slots size: %d",vcu_available_slots.size());
            cloest_slots = math::findClosesParkingSpots(VCU_car_pose,vcu_available_slots ,10); //距离排序后的slots
            LOGD("cloest_slots size: %d",cloest_slots.size());


            // *****************************************************0326 重写版 BEGIN
            if (final_select_ID == 0 && is_Still) { //当没有点选ID且静止，使用推荐ID
                LOGD("RECOMMEND1: still, Start Recommend!")

                const int max_recommend_num = 3; //display设置为1，2，3

                // 步骤 1：设置 cloest_slots[0] 对应 slot 的 slotStatusType 为 7
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

                // 步骤 2：设置推荐车位的 displayLabel 从 1 到 max_recommend_num
                for (int idx = 1; idx <= max_recommend_num && idx < cloest_slots.size(); ++idx) {
                    int targetLabel = cloest_slots[idx].slotLabel;

                    for (int i = 0; i < psd2vcu.slotNum; ++i) {
                        if (psd2vcu.FusionSlotInfo[i].slotLabel == targetLabel) {
                            psd2vcu.FusionSlotInfo[i].displayLabel = idx; // 从1开始编号
                            break;
                        }
                    }
                }



            // *****************************************************0326 重写版 END


            // *****************************************************0320 爆改版 BEGIN
            // // 维护全局变量，存储第一次计算的推荐车位和 displayLabel 车位
            // static int saved_RECOMMEND_ID = -1; // 存储 slotStatusType = 7 的车位
            // static std::vector<int> saved_displayLabels(4, -1); // 存储 displayLabel 车位的 ID

            // // 根据推荐/点选状态，改变VCU车位列表status
            // //点选与推荐的四种情况
            // if (final_select_ID == 0 && is_Still) { //当没有点选ID且静止，使用推荐ID
            //     LOGD("RECOMMEND1: still, Start Recommend!")

            //     // // *************************************************************************0320 记忆版本
            //     int closest_slots_size = std::min(5, static_cast<int>(cloest_slots.size()));
            //     if (saved_RECOMMEND_ID == -1) {  
            //         LOGD("First-time recommendation, saving slot information!");

            //         for (int i = 0; i < closest_slots_size; ++i) {
            //             for (int icnt = 0; icnt < psd2vcu.slotNum; ++icnt) { 
            //                 if (psd2vcu.FusionSlotInfo[icnt].slotLabel == cloest_slots[i].slotLabel) {  

            //                     if (psd2vcu.FusionSlotInfo[icnt].slotStatusType == 4 || psd2vcu.FusionSlotInfo[icnt].slotStatusType == 6){ //跳过占用,unavailable车位
            //                         continue;
            //                     }

            //                     if (i == 0) { // 推荐最近的车位
            //                         psd2vcu.FusionSlotInfo[icnt].slotStatusType = 7; 
            //                         saved_RECOMMEND_ID = psd2vcu.FusionSlotInfo[icnt].slotLabel; // 保存推荐车位 ID
            //                         RECOMMEND_ID = saved_RECOMMEND_ID;
            //                     } 
            //                     else if (i <= 4) { // 赋值 displayLabel
            //                         psd2vcu.FusionSlotInfo[icnt].displayLabel = i;
            //                         saved_displayLabels[i - 1] = psd2vcu.FusionSlotInfo[icnt].slotLabel; // 保存 displayLabel 车位 ID
            //                     }
            //                 }
            //             }
            //         }
            //     }
            //     else {  
            //         // 如果已经有推荐车位，不重新计算，而是继续沿用之前的值
            //         LOGD("Reusing saved slot recommendations!");

            //         for (int icnt = 0; icnt < psd2vcu.slotNum; ++icnt) {
            //             if (psd2vcu.FusionSlotInfo[icnt].slotLabel == saved_RECOMMEND_ID) {  
            //                 psd2vcu.FusionSlotInfo[icnt].slotStatusType = 7; 
            //                 RECOMMEND_ID = saved_RECOMMEND_ID;
            //             }
            //             for (int j = 0; j < 4; ++j) {  
            //                 if (psd2vcu.FusionSlotInfo[icnt].slotLabel == saved_displayLabels[j]) {  
            //                     psd2vcu.FusionSlotInfo[icnt].displayLabel = j + 1;  
            //                 }
            //             }
            //         }
            //     }
            //     // *************************************************************************

			    // // 0320 实时更新版本
                // int closest_slots_size = std::min(5, static_cast<int>(cloest_slots.size()));

                // for (int i = 0; i < closest_slots_size; ++i) {
                //     for (int icnt = 0; icnt < psd2vcu.slotNum; ++icnt) { //在PSD2VCU里找对应的车位

                //         if (psd2vcu.FusionSlotInfo[icnt].slotLabel == cloest_slots[i].slotLabel) {  //找到
                            
                //             //推荐最近的车位
                //             if (i == 0){ 
                //                 psd2vcu.FusionSlotInfo[icnt].slotStatusType = 7; 
                //                 RECOMMEND_ID = psd2vcu.FusionSlotInfo[icnt].slotLabel;
                //             }else{
                //                 psd2vcu.FusionSlotInfo[icnt].displayLabel = i;
                //             }
                //         }
                //     }
                // }
            // *****************************************************0320 爆改版 END




            // // *************************************0318 推荐车位正常，但无displayID
            // // 根据推荐/点选状态，改变VCU车位列表status
            // int closest_slots_size = std::min(5, static_cast<int>(cloest_slots.size()));

            // LOGD("RECOMMEND condition: final_select_ID: %d, is_Still: %d",final_select_ID,is_Still);
            // //点选与推荐的四种情况
            // if (final_select_ID == 0 && is_Still) { //当没有点选ID且静止，使用推荐ID
            //     LOGD("RECOMMEND1: still, Start Recommend!")
            //     int near_ID = 1;
            //     for (int i = 0; i < closest_slots_size; ++i) {
            //         for (int icnt = 0; icnt < psd2vcu.slotNum; ++icnt) { //在PSD2VCU里找对应的车位

            //             if (psd2vcu.FusionSlotInfo[icnt].slotLabel == cloest_slots[i].slotLabel && recommend_exist == false) {  //找到

            //                 if (psd2vcu.FusionSlotInfo[icnt].slotStatusType == 4){ //跳过占用车位
            //                     continue;
            //                 }

            //                 if (psd2vcu.FusionSlotInfo[icnt].slotStatusType == 3 && !recommend_exist) { //非占用 且不存在推荐车位
            //                     psd2vcu.FusionSlotInfo[icnt].slotStatusType = 7; 
            //                     RECOMMEND_ID = psd2vcu.FusionSlotInfo[icnt].slotLabel;
            //                     recommend_exist = true;
            //                 }
            //                 else if (psd2vcu.FusionSlotInfo[icnt].slotStatusType == 3 && recommend_exist && near_ID <= 4){ //非占用 且已存在推荐车位。走不到这条else
            //                     psd2vcu.FusionSlotInfo[icnt].displayLabel = near_ID;
            //                     near_ID++;
            //                 }
            //             }
            //         }
            //     }
            // // *************************************

                

                // 推荐车位作为final_ID
                final_ID = RecommendSelectID(final_select_ID,RECOMMEND_ID);
            }
            else if (final_select_ID == 0 && !is_Still) { //当没有点选ID且运动，保留RD原状态
                LOGD("RECOMMEND2: not still, NO Recommend!")
                // saved_RECOMMEND_ID = -1;
                RECOMMEND_ID = 0;
                final_select_ID = 0;
                final_ID = 0;
                recommend_exist = false;
                already_has_recommend_slot = false;
                for (int i = 0; i < slotlist_size; i++) {
                    psd2vcu.FusionSlotInfo[i].displayLabel = 0;
                    if (psd2vcu.FusionSlotInfo[i].slotStatusType == 4) { //占用的保持占用
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
                    }
                    else if (psd2vcu.FusionSlotInfo[i].slotStatusType == 6){ //unavailable的保持unavailable
                        psd2vcu.FusionSlotInfo[i].slotStatusType == 6;
                    }
                    else { //剩下的回到available
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 设置为AVAILABLE状态
                    }
                }
            }

            else if (final_select_ID != 0 && is_Still) { //当有点选车位且静止，使用点选ID
                LOGD("RECOMMEND3: still, Select!")
                // saved_RECOMMEND_ID = -1;
                RECOMMEND_ID = 0;
                final_ID = final_select_ID;
                already_has_recommend_slot = false;
                for (int i = 0; i < slotlist_size; i++) {
                    psd2vcu.FusionSlotInfo[i].displayLabel = 0;
                    //找到目标车位ID 且 非占用
                    if (psd2vcu.FusionSlotInfo[i].slotLabel == final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType != 4) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 5; // 设置为SELECTED状态
                    } 
                    //找到目标车位ID 且 占用
                    if (psd2vcu.FusionSlotInfo[i].slotLabel == final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType == 4) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
                    } 
                    //剩下的非选中车位，占用的保持占用
                    else if (psd2vcu.FusionSlotInfo[i].slotLabel != final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType == 4) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
                    }
                    //剩下的非选中车位，unavailable的保持unavailable
                    else if (psd2vcu.FusionSlotInfo[i].slotLabel != final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType == 6) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 6; // 设置为unavailable状态
                    }
                    //剩下的非选中车位，不占用,不unavailable的回到available
                    else if (psd2vcu.FusionSlotInfo[i].slotLabel != final_ID && psd2vcu.FusionSlotInfo[i].slotStatusType != 4) {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 设置为AVAILABLE状态
                    }
                }
            }

            else{ //当有点选车位且运动，清除所有ID。@TODO 前后距离超过一定值
                LOGD("RECOMMEND4: no still, no recommend, no select")
                // saved_RECOMMEND_ID = -1;
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
                    if (psd2vcu.FusionSlotInfo[i].slotStatusType == 4) { //占用的保持占用
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 设置为OCCUPIED状态
                    }
                    else if (psd2vcu.FusionSlotInfo[i].slotStatusType == 6){ //unavailable的保持unavailable
                        psd2vcu.FusionSlotInfo[i].slotStatusType == 6;
                    }
                    else { //不占用的回到available
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 设置为AVAILABLE状态
                    }
                }
            }
        }

        // For Test VCU slot lists
        for (int icnt = 0; icnt < slotlist_size; icnt++){
            LOGD("[PSD2VCUSLOTLIST] apa_status: %d, slotsize: %d, TYPE: %d, STATUS: %d, ID: %d, displayID: %d (%f,%f) (%f,%f) (%f,%f) (%f,%f), timestamp: %llu",
            apa_status,
            slotlist_size,
            psd2vcu.FusionSlotInfo[icnt].slotType,
            psd2vcu.FusionSlotInfo[icnt].slotStatusType,
            psd2vcu.FusionSlotInfo[icnt].slotLabel,
            psd2vcu.FusionSlotInfo[icnt].displayLabel,
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
        // // 固定点选车位
        // for (int i = 0; i < slotlist_size; i++) {
        //     if (psd2vcu.FusionSlotInfo[i].slotLabel == final_select_ID) {
        //         psd2vcu.FusionSlotInfo[i].fusionSlotType = 1; //@TODO 作为点选flag，下个版本有新接口后更换
        //     } 
        // }
        if (apa_status != 1){
            LOGD("[RECOMMENDSELECTID] HMI %d, VCU %d, final select %d, recommend: %d, final_ID %d",HMI_temp_ID,VCU_select_ID_ON,final_select_ID,RECOMMEND_ID,final_ID);

            // //固定psd2vcu
            // FixedStillVCUSlots(psd2vcu,is_Still);

            EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
        }
    }

    else { //泊入过程中显示所有车位
        LOGD("[SELECT_SLOT]The apa staus is 5!");
        memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector));
        psd2vcu.slotNum = slotlist_size;
        if (psd2vcu.slotNum > 0){
            int i = 0;
            LOGD("PSD2VCU apa_status: %d, outputslot_fused size: %d",apa_status,outputSlot_FUSED.slots_in_cur_frame.size());
            
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
                    psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) ) / MM_TO_M; //mm 转 m , VCU坐标系上x右y
                    psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[0].x / MM_TO_M; //后轴中心转前保中心
                    psd2vcu.FusionSlotInfo[i].pt[0].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[1].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) ) / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[1].x/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[2].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[2].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[3].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[3].x/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
                    if (psd_m_output.rectInfo.iSodType == 1){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                    }else {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                    }
                    // VCU显示障碍物
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
                }
                //右侧
                else{
                    psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  )/ MM_TO_M; //mm 转 m
                    psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[0].x / MM_TO_M; //后轴中心转前保中心
                    psd2vcu.FusionSlotInfo[i].pt[0].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[1].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[1].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[2].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[2].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                    psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[3].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR)  )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[3].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
                    if (psd_m_output.rectInfo.iSodType == 1){
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 4; // 被占用
                    }else {
                        psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 无占用
                    }
                    // VCU显示障碍物
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
                }

                // // 点选车位VCU显示， 作为点选flag，下个版本有新接口后更换
                // for (int i = 0; i < slotlist_size; i++) {
                //     if (psd2vcu.FusionSlotInfo[i].fusionSlotType == 1) {
                //         psd2vcu.FusionSlotInfo[i].slotStatusType = 5; // 设置为选中的状态
                //     } 
                // }
                i++;
            }
        }
        for (int i = 0; i < slotlist_size; i++){
            LOGD("[PSD2VCUSLOTLIST] IN GUIDANCE, Slot#%d, type: %d, (%f,%f) (%f,%f) (%f,%f) (%f,%f), timestamp: %llu",
            psd2vcu.FusionSlotInfo[i].slotLabel,
            psd2vcu.FusionSlotInfo[i].slotStatusType,
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

    // 泊入过程中只显示目标车位
    // else{
    //     LOGD("[SELECT_SLOT]The apa staus is 5!");

    //     memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector));
    //     psd2vcu.FusionSlotInfo[0].slotLabel = final_ID; //ID
    //     psd2vcu.FusionSlotInfo[0].displayLabel = 0;
    //     Sfus::Sfsuion2DecPlan temp_psd2planning;
    //     temp_psd2planning = psd2planning;
        
    //     if (dr_first){
    //         dr_cul_x = pose_globaldata.coord.x;
    //         dr_cul_y = pose_globaldata.coord.y;
    //         dr_cul_theta = pose_globaldata.yaw;

    //         dr_first = false;
    //     }

    //     LOGD("TEST_2025_01:(%d,%d,%f)", dr_cul_x, dr_cul_y, dr_cul_theta);
    //     LOGD("TEST_2025_01: global dr(%d,%d,%f)", pose_globaldata.coord.x,  pose_globaldata.coord.y,  pose_globaldata.yaw);
        
    //     Slot2Global(temp_psd2planning, dr_cul_x, dr_cul_y, dr_cul_theta);
    //     Slot2Local(temp_psd2planning, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
       
    //     // rotatePoint(psd2vcu, pose_globaldata);
    //     if (psd2planning.targetSlot.slotCorners.cornerA.x <= 0 ||  psd2planning.targetSlot.slotCorners.cornerB.x <= 0){
    //         // psd2vcu.FusionSlotInfo[0].pt[0].x = (temp_psd2planning.targetSlot.slotCorners.cornerB.y - VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[0].x = (temp_psd2planning.targetSlot.slotCorners.cornerB.y - (VEHICLE_LENGTH / 2 - REAR_AXLE_CENTER_VEHICLE_REAR)) / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[0].y = temp_psd2planning.targetSlot.slotCorners.cornerB.x / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[0].z = 0;

    //         psd2vcu.FusionSlotInfo[0].pt[1].x = (temp_psd2planning.targetSlot.slotCorners.cornerA.y - (VEHICLE_LENGTH / 2 - REAR_AXLE_CENTER_VEHICLE_REAR)) / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[1].y = temp_psd2planning.targetSlot.slotCorners.cornerA.x / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[1].z = 0;

    //         psd2vcu.FusionSlotInfo[0].pt[2].x = (temp_psd2planning.targetSlot.slotCorners.cornerD.y - (VEHICLE_LENGTH / 2 - REAR_AXLE_CENTER_VEHICLE_REAR)) / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[2].y = temp_psd2planning.targetSlot.slotCorners.cornerD.x / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[2].z = 0;

    //         psd2vcu.FusionSlotInfo[0].pt[3].x = (temp_psd2planning.targetSlot.slotCorners.cornerC.y - (VEHICLE_LENGTH / 2 - REAR_AXLE_CENTER_VEHICLE_REAR)) / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[3].y = temp_psd2planning.targetSlot.slotCorners.cornerC.x / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[3].z = 0;

    //         psd2vcu.FusionSlotInfo[0].slotStatusType = 5;
    //         psd2vcu.FusionSlotInfo[0].backInAvailableFlag = 1;
    //         psd2vcu.FusionSlotInfo[0].parkInHeadInSoftButtonCurrentValue = 1;
    //         LOGD("2025_01:left slot");
    //     }else{
    //         psd2vcu.FusionSlotInfo[0].pt[0].x = (temp_psd2planning.targetSlot.slotCorners.cornerB.y - (VEHICLE_LENGTH / 2 - REAR_AXLE_CENTER_VEHICLE_REAR)) / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[0].y = temp_psd2planning.targetSlot.slotCorners.cornerB.x / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[0].z = 0;

    //         psd2vcu.FusionSlotInfo[0].pt[1].x = (temp_psd2planning.targetSlot.slotCorners.cornerC.y - (VEHICLE_LENGTH / 2 - REAR_AXLE_CENTER_VEHICLE_REAR))/ 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[1].y = temp_psd2planning.targetSlot.slotCorners.cornerC.x / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[1].z = 0;

    //         psd2vcu.FusionSlotInfo[0].pt[2].x = (temp_psd2planning.targetSlot.slotCorners.cornerD.y - (VEHICLE_LENGTH / 2 - REAR_AXLE_CENTER_VEHICLE_REAR)) / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[2].y = temp_psd2planning.targetSlot.slotCorners.cornerD.x / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[2].z = 0;

    //         psd2vcu.FusionSlotInfo[0].pt[3].x = (temp_psd2planning.targetSlot.slotCorners.cornerA.y - (VEHICLE_LENGTH / 2 - REAR_AXLE_CENTER_VEHICLE_REAR)) / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[3].y = temp_psd2planning.targetSlot.slotCorners.cornerA.x / 1000.0;
    //         psd2vcu.FusionSlotInfo[0].pt[3].z = 0;

    //         psd2vcu.FusionSlotInfo[0].slotStatusType = 5;
    //         psd2vcu.FusionSlotInfo[0].backInAvailableFlag = 1;
    //         psd2vcu.FusionSlotInfo[0].parkInHeadInSoftButtonCurrentValue = 1;
    //         LOGD("2025_01:right slot");
    //     }
    //     for (int icnt = 0; icnt < 4; icnt++){
    //         LOGD("[TEST slot pt](%f,%f)",psd2vcu.FusionSlotInfo[0].pt[icnt].x, psd2vcu.FusionSlotInfo[0].pt[icnt].y);
    //     }
    //      LOGD("[SELECT_SLOT] selected slot:(%f,%f),(%f,%f),(%f,%f),(%f,%f)", psd2vcu.FusionSlotInfo[0].pt[0].x,psd2vcu.FusionSlotInfo[0].pt[0].y,
    //                                                                      psd2vcu.FusionSlotInfo[0].pt[1].x,psd2vcu.FusionSlotInfo[0].pt[1].y,
    //                                                                      psd2vcu.FusionSlotInfo[0].pt[2].x,psd2vcu.FusionSlotInfo[0].pt[2].y,
    //                                                                      psd2vcu.FusionSlotInfo[0].pt[3].x,psd2vcu.FusionSlotInfo[0].pt[3].y);
                                                                    
    //     LOGD("[SELECT_SLOT] SEND VCU TARGET SLOT!!!!");
    //     if (apa_status != 1){

    //         LOGD("[RECOMMENDSELECTID] HMI %d, VCU %d, final select %d, recommend: %d, final_ID %d",HMI_temp_ID,VCU_select_ID_ON,final_select_ID,RECOMMEND_ID,final_ID);
    //         EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
    //     }
    // }






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
            }
            else{
                if (psd_m_output.rectInfo.label >= 1000 && psd_m_output.rectInfo.label < 10000){
                    psd2location.fusionSlotInfo[j].fusionSlotType = 0;
                }
                else{
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

            LOGD("[PSD2APAHANDLE] TOTAL SLOT NUM: %d, Slot#%d, slottype: %d, fusionslottype: %d (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                    psd2location.slotNum,
                    psd2location.fusionSlotInfo[j].slotLabel,
                    psd2location.fusionSlotInfo[j].slotType,
                    psd2location.fusionSlotInfo[j].fusionSlotType,
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
    if (final_ID){
        psd2apahandel_targetID.targetSlotLabel = final_ID;
        if (parkout_flag != 1){
            EMC_psd_fusion_process_SetFieldSlotlabel(psd2apahandel_targetID);
        }
        LOGD("[PSD2APAHANDLE] target slot id: %d",final_ID);
    }


    //***********************************PLANNING/HMI 发送车位列表
    //check psd output to planning(1 slot list)  
    //规划暂时不用车位列表，需等待预规划模块ready后。暂时用于HMI显示车位列表
    int planning_slotnum = outputSlot_FUSED.slots_in_cur_frame.size();
    LOGD("slot list size:%d",planning_slotnum);
    int k = 0;
    for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame){
        if (k >= slotlist_size || k >= 50){
            std::cout<<"die in planning and size is:"<< slotlist_size << std::endl;
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
        if (angleDegrees > 80  || angleDegrees < 100)
        {
            psd2planning.SfusionSrchSlots[k].slotType = slottype_rd2decplan(psd_m_output.rectInfo.PStype);
        }
        else if (angleDegrees <= 80){
            psd2planning.SfusionSrchSlots[k].slotType = Sfus::SLOTTYP_RFOBL;
        }
        else{
            psd2planning.SfusionSrchSlots[k].slotType = Sfus::SLOTTYP_OBL;
        }
        //*******************
        
        if (psd2planning.SfusionSrchSlots[k].slotID >= 1000 && psd2planning.SfusionSrchSlots[k].slotID < 10000){
            psd2planning.SfusionSrchSlots[k].slotSource = Sfus::SLOTSRC_VIS;
        } else if (psd2planning.SfusionSrchSlots[k].slotID >= 10000){
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

    //***********************************PLANNING 发送目标车位
    //check psd output to planning(2 target slot)
    //拿到目标车位ID后，发送目标车位信息给planning
    if (final_ID > 0 && apa_status != 5 && parkout_flag != 1){ //进入guidance后固定目标车位角点
        for (int i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size();++i){
            if (final_ID == outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label){
                psd2planning.targetSlot.slotCorners.cornerA.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x; 
                psd2planning.targetSlot.slotCorners.cornerA.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                psd2planning.targetSlot.slotCorners.cornerB.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x;
                psd2planning.targetSlot.slotCorners.cornerB.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y;
                psd2planning.targetSlot.slotCorners.cornerC.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].x;
                psd2planning.targetSlot.slotCorners.cornerC.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].y;
                psd2planning.targetSlot.slotCorners.cornerD.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x;
                psd2planning.targetSlot.slotCorners.cornerD.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y;
                // *******************正逆鱼骨
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
                if (angleDegrees > 80  || angleDegrees < 100)
                {
                    psd2planning.targetSlot.slotType = slottype_rd2decplan(outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.PStype);
                }
                else if (angleDegrees <= 80){
                    psd2planning.SfusionSrchSlots[k].slotType = Sfus::SLOTTYP_RFOBL;
                }
                else{
                    psd2planning.SfusionSrchSlots[k].slotType = Sfus::SLOTTYP_OBL;
                }
                //*******************
                psd2planning.targetSlot.stopper_Dis = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.StopperDistance;
                if (final_ID >= 1000 && final_ID < 10000){
                    psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
                }
                else if (final_ID >= 10000){
                    psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_USS;
                }
                else{
                    psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
                }
            }
        }
    }
    if (parkout_flag != 1){
        if (apa_status == 1 || apa_status == 6 || apa_status == 7 || apa_status == 0){
            memset(&psd2planning, 0, sizeof(Sfus::Sfsuion2DecPlan));
            EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);
        }
        else{
            LOGD("[PSD2PLANNING] TIMESTAMP: %llu, APASTATUS: %d, TARGET SLOT type: %d, source: %d, stopper dis: %f, (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
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
            EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);
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
            psd2perception.slotSource = psd2planning.targetSlot.slotSource;
            psd2perception.timeStamp = (current1970_ms >= 0) ? static_cast<uint64_t>(current1970_ms) : 0;
        }
        LOGD("[PSD2PERCEPTION] TIMESTAMP: %llu, APASTATUS: %d, TARGET SLOT type: %d, source: %d (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
        psd2perception.timeStamp,
        apa_status,
        psd2perception.slotType,
        psd2perception.slotSource,
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
        final_ID = 0;
        HMI_temp_ID = 0;
    }
    if (final_ID > 0){
        psd2statemachine.aps_apaParkType = slottype_decplan2statemachine(psd2planning.targetSlot.slotType);
        if (final_ID >= 10000){
            psd2statemachine.aps_apaParkFusionType = 1;
        }
        else{
            psd2statemachine.aps_apaParkFusionType = 0;
        }
        psd2statemachine.aps_apaNarrowSlot = 0; //@TODO 窄车位
        if (slotlist_size > 0){
            psd2statemachine.aps_apaAvailableSlot = 1;
            psd2statemachine.aps_apaHighlightSlot = 1;
        }
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

    //***********************************USS 发送目标车位ID
    short targetUssSlotID = 0;

    if (final_ID >= 10000) {
        if (final_ID > SHRT_MAX) {  // SHRT_MAX 来自 <limits.h>
            LOGD("Error: final_ID exceeds short range!\n");
        } else {
            targetUssSlotID = (short) final_ID;
            S2S_MCore_Bridge_SetSigtargetUssSlotLabel(&targetUssSlotID);
        }
    }

    //***********************************Control 发送限位块信息
    memset(&psd2control, 0, sizeof(APAControlBumpInput));
    if (final_ID != 0){
        for (int i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size();++i){
            if (final_ID == outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label){
                for (int j = 0; j < 2; ++j) {
                    psd2control.apc_LimitBarX[j] = static_cast<tInt16>(outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.StopperX[j]);
                    psd2control.apc_LimitBarY[j] = static_cast<tInt16>(outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.StopperY[j]);
                }
            }
        }
    }
    LOGD("[PSD2CONTROL]LimitBar for final_ID #%d: (%d, %d), (%d, %d)", 
       final_ID,
       psd2control.apc_LimitBarX[0], psd2control.apc_LimitBarY[0],
       psd2control.apc_LimitBarX[1], psd2control.apc_LimitBarY[1]);

    S2S_MCore_Bridge_SetSigAPAControlBumpInput(&psd2control);



    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOGD("[TIMECOST]Timetrigger100 time is: %d",elapsed.count());

    RETURN_NOERROR;
}


