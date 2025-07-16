#include "utils.h"
#include "save_to_json.h"

namespace PSDConfigUtils {

void ClearRD(rd::QuadParkingSlots& rd_info) {
    rd_info = rd::QuadParkingSlots{};
    LOGD("CLEAR rd_info");
}

void ClearDR(Loc::App2emap_DR& dr_pose, padVehiclePose& pose_globaldata) {
    dr_pose = Loc::App2emap_DR{};       
    pose_globaldata = padVehiclePose{};  
    LOGD("CLEAR dr_pose and pose_globaldata");
}

void ClearOBS(Fus::PkEmapObs& obs_info_get) {
    obs_info_get = Fus::PkEmapObs{};
    LOGD("CLEAR obs_info_get");
}

void ClearUSS(UssIf_stPLVOutputInfo_t& uss_info, UssIf_stPLVOutputInfo_t& uss_restruct) {
    uss_info = UssIf_stPLVOutputInfo_t{};
    uss_restruct = UssIf_stPLVOutputInfo_t{};
    LOGD("CLEAR uss_info");
}

void ClearParkingSlots(std::vector<padVisionSlotCoord>& singleframeslots, unsigned long long& singleframeslotsID, apaSlotListInfo& outputSlot_VIS, apaSlotListInfo& outputSlot_USS, apaSlotListInfo& outputSlot_FUSED, int& parkout_flag) {
    singleframeslots.clear();
    singleframeslotsID = 0;
    outputSlot_VIS.slots_in_cur_frame.clear();
    outputSlot_USS.slots_in_cur_frame.clear();
    outputSlot_FUSED.slots_in_cur_frame.clear();
    parkout_flag = 0;
    LOGD("CLEAR singleframeslots, size: %d, parkout_flag = %d", singleframeslots.size(), parkout_flag);
}

bool CheckTimeSync(uint64_t current1970_ms, uint64_t rd_timestamp, uint64_t dr_timestamp) {
    if (std::llabs(current1970_ms - rd_timestamp) > 1500 || std::llabs(current1970_ms - dr_timestamp) > 1500) {
        LOGW("[TIMESYNC] current: %llu, RD timestamp: %llu, DR timestamp: %llu, Time synchronization not met, skipping execution.", current1970_ms, rd_timestamp, dr_timestamp);
        return false;
    }
    LOGD("[TIMESYNC] current: %llu, RD timestamp: %llu, DR timestamp: %llu, Time synchronization achieved!", current1970_ms, rd_timestamp, dr_timestamp);
    return true;
}

bool CheckRDFrameTimestamp(uint64_t currentTimestamp) {
    static uint64_t previousTimestamp = 0;  // 存储上一个周期的时间戳

    if (currentTimestamp - previousTimestamp > 5) {
        LOGW("[RDDelay] **************PASS************** currentTimestamp: %llu, previousTimestamp: %llu",currentTimestamp, previousTimestamp);
        // 更新上一个时间戳
        previousTimestamp = currentTimestamp;
        return true;
    }
    else{
        LOGW("[RDDelay] **************DELAY************** currentTimestamp: %llu, previousTimestamp: %llu",currentTimestamp, previousTimestamp);
        // 更新上一个时间戳
        previousTimestamp = currentTimestamp;
        return false;
    }
}

void LogSlotInfo(const apaSlotListInfo& slots, const std::string& slotType) {
    for (const auto& psd_m_output : slots.slots_in_cur_frame) {
        if (slotType == "ORIGIN VISSLOTS") {
            LOGD("[%s] TOTAL SLOT NUM: %d, Slot#%d, type: %d, occ: %d, StopDis: %f, StopLoc: %d, Stopper: (%d, %d) (%d, %d). (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                 slotType.c_str(),
                 slots.slots_in_cur_frame.size(),
                 psd_m_output.rectInfo.label,
                 psd_m_output.rectInfo.PStype,
                 psd_m_output.rectInfo.iSodType,
                 psd_m_output.rectInfo.StopperDistance,
                 psd_m_output.rectInfo.StopperLocation,
                 psd_m_output.rectInfo.StopperX[0],
                 psd_m_output.rectInfo.StopperY[0],
                 psd_m_output.rectInfo.StopperX[1],
                 psd_m_output.rectInfo.StopperY[1],
                 psd_m_output.rectInfo.pt[0].x,
                 psd_m_output.rectInfo.pt[0].y,
                 psd_m_output.rectInfo.pt[1].x,
                 psd_m_output.rectInfo.pt[1].y,
                 psd_m_output.rectInfo.pt[2].x,
                 psd_m_output.rectInfo.pt[2].y,
                 psd_m_output.rectInfo.pt[3].x,
                 psd_m_output.rectInfo.pt[3].y);
        } else if (slotType == "ORIGIN USSSLOTS") {
            LOGD("[%s] TOTAL SLOT NUM: %d, Slot#%d, type: %d, SOD: %d, DownSlotSOD: %d, iMinOtherSideDist: %d, iRoadEdgeDist: %d, (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                 slotType.c_str(),
                 slots.slots_in_cur_frame.size(),
                 psd_m_output.rectInfo.label,
                 psd_m_output.rectInfo.PStype,
                 psd_m_output.rectInfo.iSodType,
                 psd_m_output.rectInfo.iDownSlotSOD,
                 psd_m_output.rectInfo.iMinOtherSideDist,
                 psd_m_output.rectInfo.iRoadEdgeDist,
                 psd_m_output.rectInfo.pt[0].x,
                 psd_m_output.rectInfo.pt[0].y,
                 psd_m_output.rectInfo.pt[1].x,
                 psd_m_output.rectInfo.pt[1].y,
                 psd_m_output.rectInfo.pt[2].x,
                 psd_m_output.rectInfo.pt[2].y,
                 psd_m_output.rectInfo.pt[3].x,
                 psd_m_output.rectInfo.pt[3].y);
        } else if (slotType == "FUSIONSLOTS") {
            LOGD("[%s] TOTAL SLOT NUM: %d, Slot#%d, type: %d, occ: %d, StopDis: %f, StopLoc: %d, Material: %d, NotToRelease: %d, ParkInSlot: %d, Stopper: (%d, %d) (%d, %d). (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                 slotType.c_str(),
                 slots.slots_in_cur_frame.size(),
                 psd_m_output.rectInfo.label,
                 psd_m_output.rectInfo.PStype,
                 psd_m_output.rectInfo.iSodType,
                 psd_m_output.rectInfo.StopperDistance,
                 psd_m_output.rectInfo.StopperLocation,
                 psd_m_output.rectInfo.iMaterial,
                 psd_m_output.rectInfo.NotToRelease,
                 psd_m_output.rectInfo.ParkInSlot,
                 psd_m_output.rectInfo.StopperX[0],
                 psd_m_output.rectInfo.StopperY[0],
                 psd_m_output.rectInfo.StopperX[1],
                 psd_m_output.rectInfo.StopperY[1],
                 psd_m_output.rectInfo.pt[0].x,
                 psd_m_output.rectInfo.pt[0].y,
                 psd_m_output.rectInfo.pt[1].x,
                 psd_m_output.rectInfo.pt[1].y,
                 psd_m_output.rectInfo.pt[2].x,
                 psd_m_output.rectInfo.pt[2].y,
                 psd_m_output.rectInfo.pt[3].x,
                 psd_m_output.rectInfo.pt[3].y);
        }
    }
}

void LogWorldSlotInfo(const apaSlotListInfo& slots, const std::string& slotType) {
    for (const auto& psd_m_output : slots.WorldoutRect) {
        if (slotType == "ORIGIN VISSLOTS") {
            LOGD("[%s World] TOTAL SLOT NUM: %d, Slot#%d, type: %d, occ: %d, StopDis: %f, StopLoc: %d, (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                 slotType.c_str(),
                 slots.WorldoutRect.size(),
                 psd_m_output.rectInfo.label,
                 psd_m_output.rectInfo.PStype,
                 psd_m_output.rectInfo.iSodType,
                 psd_m_output.rectInfo.StopperDistance,
                 psd_m_output.rectInfo.StopperLocation,
                 psd_m_output.rectInfo.pt[0].x,
                 psd_m_output.rectInfo.pt[0].y,
                 psd_m_output.rectInfo.pt[1].x,
                 psd_m_output.rectInfo.pt[1].y,
                 psd_m_output.rectInfo.pt[2].x,
                 psd_m_output.rectInfo.pt[2].y,
                 psd_m_output.rectInfo.pt[3].x,
                 psd_m_output.rectInfo.pt[3].y);
        } else if (slotType == "ORIGIN USSSLOTS") {
            LOGD("[%s World] TOTAL SLOT NUM: %d, Slot#%d, type: %d, SOD: %d, DownSlotSOD: %d, iMinOtherSideDist: %d, iRoadEdgeDist: %d, (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                 slotType.c_str(),
                 slots.WorldoutRect.size(),
                 psd_m_output.rectInfo.label,
                 psd_m_output.rectInfo.PStype,
                 psd_m_output.rectInfo.iSodType,
                 psd_m_output.rectInfo.iDownSlotSOD,
                 psd_m_output.rectInfo.iMinOtherSideDist,
                 psd_m_output.rectInfo.iRoadEdgeDist,
                 psd_m_output.rectInfo.pt[0].x,
                 psd_m_output.rectInfo.pt[0].y,
                 psd_m_output.rectInfo.pt[1].x,
                 psd_m_output.rectInfo.pt[1].y,
                 psd_m_output.rectInfo.pt[2].x,
                 psd_m_output.rectInfo.pt[2].y,
                 psd_m_output.rectInfo.pt[3].x,
                 psd_m_output.rectInfo.pt[3].y);
        } else if (slotType == "FUSIONSLOTS") {
            LOGD("[%s World] TOTAL SLOT NUM: %d, Slot#%d, type: %d, occ: %d, StopDis: %f, StopLoc: %d, Material: %d, NotToRelease: %d, (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                 slotType.c_str(),
                 slots.WorldoutRect.size(),
                 psd_m_output.rectInfo.label,
                 psd_m_output.rectInfo.PStype,
                 psd_m_output.rectInfo.iSodType,
                 psd_m_output.rectInfo.StopperDistance,
                 psd_m_output.rectInfo.StopperLocation,
                 psd_m_output.rectInfo.iMaterial,
                 psd_m_output.rectInfo.NotToRelease,
                 psd_m_output.rectInfo.pt[0].x,
                 psd_m_output.rectInfo.pt[0].y,
                 psd_m_output.rectInfo.pt[1].x,
                 psd_m_output.rectInfo.pt[1].y,
                 psd_m_output.rectInfo.pt[2].x,
                 psd_m_output.rectInfo.pt[2].y,
                 psd_m_output.rectInfo.pt[3].x,
                 psd_m_output.rectInfo.pt[3].y);
        }
    }
}

void ClearSelectRecommendSlot(int& HMI_temp_ID, int& HMI_select_ID, int& VCU_select_ID_ON, int& final_select_ID, int& RECOMMEND_ID, int& final_ID, int apa_status) {
    HMI_temp_ID = 0;
    HMI_select_ID = 0;
    VCU_select_ID_ON = 0;
    final_select_ID = 0;
    RECOMMEND_ID = 0;
    final_ID = 0;
    LOGD("[STATUSSELECT] in ClearSelectRecommendSlot HMI %d, VCU %d, final select %d", HMI_temp_ID, VCU_select_ID_ON, final_select_ID);
}



float CalcDistance(const POINT_I& a, const POINT_I& b) {
    float dx = static_cast<float>(a.x - b.x);
    float dy = static_cast<float>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

bool LoadFromFile(const std::string& filename) {
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        DEBUG = false;
        FARAWAY_FILTER = false;
        LOGD("无法打开配置文件: %s, DEBUG: %d, FARAWAY_FILTER: %d", filename.c_str(), DEBUG, FARAWAY_FILTER);
        return false;
    }

    try {
        json j;
        inFile >> j;

        // 解析文件路径
        j.at("debug").at("save_to_json").get_to(DEBUG);

        j.at("calib").at("FARAWAY_FILTER").get_to(FARAWAY_FILTER);
        auto FARAWAY_SLOTS_LEFT_RANGE = j.at("calib").at("FARAWAY_SLOTS_LEFT");
        FARAWAY_SLOTS_LEFT[0] = FARAWAY_SLOTS_LEFT_RANGE[0];
        FARAWAY_SLOTS_LEFT[1] = FARAWAY_SLOTS_LEFT_RANGE[1];
        auto FARAWAY_SLOTS_RIGHT_RANGE = j.at("calib").at("FARAWAY_SLOTS_RIGHT");
        FARAWAY_SLOTS_RIGHT[0] = FARAWAY_SLOTS_RIGHT_RANGE[0];
        FARAWAY_SLOTS_RIGHT[1] = FARAWAY_SLOTS_RIGHT_RANGE[1];
        j.at("calib").at("FARAWAY_SLOTS_REAR").get_to(FARAWAY_SLOTS_REAR);
        j.at("calib").at("FARAWAY_SLOTS_FRONT").get_to(FARAWAY_SLOTS_FRONT);

        j.at("calib").at("ANGEL_FILTER").get_to(ANGEL_FILTER);
        j.at("calib").at("ANGEL_FILTER_LIMIT").get_to(ANGEL_FILTER_LIMIT);

        j.at("calib").at("VCU_TOO_SMALL_FILTER").get_to(VCU_TOO_SMALL_FILTER);
        j.at("calib").at("VCU_TOO_SMALL").get_to(VCU_TOO_SMALL);

        j.at("calib").at("PARALLEL_VECTOR_FILTER").get_to(PARALLEL_VECTOR_FILTER);
        j.at("calib").at("PARALLEL_VECTOR_LIMIT").get_to(PARALLEL_VECTOR_LIMIT);

        j.at("calib").at("NARROWSLOT_THRESHOLD").get_to(NARROWSLOT_THRESHOLD);
    }
    catch (json::exception& e) {
        LOGD("配置文件解析错误: %s", e.what());
        return false;
    }
    inFile.close();
    return true;
}

void Slot2Global(Sfus::Sfsuion2DecPlan &slot, const float &x, const float &y, const float &yaw) {
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

void Slot2Local(Sfus::Sfsuion2DecPlan &slot, const float &x, const float &y, const float &yaw) {
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

POINT_I Local2Global(const POINT_I& pt_local, const float& x, const float& y, const float& yaw) {
    float theta = yaw * acos(-1) / 180.0;
    POINT_I pt_global;
    pt_global.x = pt_local.x * cos(theta) + pt_local.y * sin(theta) + x;
    pt_global.y = pt_local.y * cos(theta) - pt_local.x * sin(theta) + y;
    return pt_global;
}


}