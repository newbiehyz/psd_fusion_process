#include "utils.h"

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
            LOGD("[%s] TOTAL SLOT NUM: %d, Slot#%d, type: %d, occ: %d, StopDis: %f, StopLoc: %d, (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                 slotType.c_str(),
                 slots.slots_in_cur_frame.size(),
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
            LOGD("[%s RESTRUCT] TOTAL SLOT NUM: %d, Slot#%d, type: %d, occ: %d, StopDis: %f, StopLoc: %d, Material: %d, (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                 slotType.c_str(),
                 slots.slots_in_cur_frame.size(),
                 psd_m_output.rectInfo.label,
                 psd_m_output.rectInfo.PStype,
                 psd_m_output.rectInfo.iSodType,
                 psd_m_output.rectInfo.StopperDistance,
                 psd_m_output.rectInfo.StopperLocation,
                 psd_m_output.rectInfo.iMaterial,
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

