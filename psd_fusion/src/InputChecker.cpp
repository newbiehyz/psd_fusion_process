#include "InputChecker.h"
#include "utils.h"
#include "psd2vcu.h"
#include "PSD_FusionModuleIF.h"

extern Loc::App2emap_DR previous_dr_pose;
extern std::vector<apaSlotInfo> g_singleframe_locked_slots;
extern PSD_FusionModuleIF PSD_FusionModuleIFrunable;
extern Sfus::FusionSlotInfovector psd2vcu;
extern StatusDecFusionInput psd2statemachine;
extern bool target_slot_already_updated_once;

void ClearHistoricalSlots(int apa_status) {
    if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7) {
        rd::QuadParkingSlots rd_info{};
        Loc::App2emap_DR dr_pose{};
        padVehiclePose pose_globaldata{};
        Fus::PkEmapObs obs_info_get{};
        UssIf_stPLVOutputInfo_t uss_info{};
        UssIf_stPLVOutputInfo_t uss_info_restruct{};

        PSDConfigUtils::ClearRD(rd_info);
        PSDConfigUtils::ClearDR(dr_pose, pose_globaldata);
        PSDConfigUtils::ClearOBS(obs_info_get);
        PSDConfigUtils::ClearUSS(uss_info, uss_info_restruct);
        PSD_FusionModuleIFrunable.ClearSlotsMap();
    }
}

void ClearHistoricalSlotsOnceSearch(int apa_status, bool& has_cleared_once) {
    if (apa_status == 2 && !has_cleared_once) {
        rd::QuadParkingSlots rd_info{};
        Loc::App2emap_DR dr_pose{};
        padVehiclePose pose_globaldata{};
        Fus::PkEmapObs obs_info_get{};
        UssIf_stPLVOutputInfo_t uss_info{};
        UssIf_stPLVOutputInfo_t uss_info_restruct{};

        PSDConfigUtils::ClearRD(rd_info);
        PSDConfigUtils::ClearDR(dr_pose, pose_globaldata);
        PSDConfigUtils::ClearOBS(obs_info_get);
        PSDConfigUtils::ClearUSS(uss_info, uss_info_restruct);
        PSD_FusionModuleIFrunable.ClearSlotsMap();

        g_singleframe_locked_slots.clear();
        memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector));
        EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
        memset(&psd2statemachine, 0, sizeof(StatusDecFusionInput));
        S2S_MCore_Bridge_SetSigStatusDecFusionInput(&psd2statemachine);

        target_slot_already_updated_once = false;
        has_cleared_once = true;
    }
}

void HandleMirrorFold(std::vector<padVisionSlotCoord>& singleframeslots, unsigned long long& slotID, rd::QuadParkingSlots& rd_info) {
    rd_info.frameTimeStampNs = 0;
    rd_info.quadParkingSlotList.clear();
    singleframeslots.clear();
    slotID = 0;
}

void UpdateParkoutAndStill(int apa_status, const Loc::App2emap_DR& dr_pose, Loc::App2emap_DR& prev_pose, int& parkout_flag, int& is_still) {
    parkout_flag = (apa_status == 3) ? 1 : (apa_status == 2 ? 0 : parkout_flag);
    is_still = fabs(dr_pose.x - prev_pose.x) < 30.0 &&
               fabs(dr_pose.y - prev_pose.y) < 30.0 &&
               fabs(dr_pose.canAng - prev_pose.canAng) < 30.0 ? 1 : 0;

    prev_pose = dr_pose;
}

void CheckSlotStatus(const std::vector<padVisionSlotCoord>& singleframeslots,
                          const apaSlotListInfo& vis, const apaSlotListInfo& uss, const apaSlotListInfo& fused) {
    LOGD("[CHECK SIZE] singleframeslots size: %d", singleframeslots.size());
    LOGD("[CHECK SIZE] vis: %d, uss: %d, fused: %d",
         vis.slots_in_cur_frame.size(),
         uss.slots_in_cur_frame.size(),
         fused.slots_in_cur_frame.size());
}
