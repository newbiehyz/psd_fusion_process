#pragma once

#include <cmath>  // for std::llabs
#include <vector>
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"

bool CheckTimeSync(uint64_t current1970_ms, uint64_t rd_timestamp, uint64_t dr_timestamp);

bool CheckRDFrameTimestamp(uint64_t currentTimestamp);

void ClearRD(rd::QuadParkingSlots& rd_info);

void ClearDR(Loc::App2emap_DR& dr_pose, padVehiclePose& pose_globaldata);

void ClearOBS(Fus::PkEmapObs& obs_info_get);

void ClearUSS(UssIf_stPLVOutputInfo_t& uss_info, UssIf_stPLVOutputInfo_t& uss_restruct);

void ClearParkingSlots(std::vector<padVisionSlotCoord>& singleframeslots, 
                       unsigned long long& singleframeslotsID, 
                       apaSlotListInfo& outputSlot_VIS, 
                       apaSlotListInfo& outputSlot_USS, 
                       apaSlotListInfo& outputSlot_FUSED, 
                       int& parkout_flag);

void ClearAllData(rd::QuadParkingSlots& rd_info,
                  Loc::App2emap_DR& dr_pose,
                  padVehiclePose& pose_globaldata,
                  Fus::PkEmapObs& obs_info_get,
                  UssIf_stPLVOutputInfo_t& uss_info,
                  UssIf_stPLVOutputInfo_t& uss_restruct,
                  std::vector<padVisionSlotCoord>& singleframeslots,
                  unsigned long long& singleframeslotsID,
                  apaSlotListInfo& outputSlot_VIS,
                  apaSlotListInfo& outputSlot_USS,
                  apaSlotListInfo& outputSlot_FUSED,
                  int& parkout_flag);

void LogSlotInfo(const apaSlotListInfo& slots, const std::string& slotType);

void LogWorldSlotInfo(const apaSlotListInfo& slots, const std::string& slotType);

void ClearSelectRecommendSlot(int& HMI_temp_ID, int& HMI_select_ID, int& VCU_select_ID_ON, int& final_select_ID, int& RECOMMEND_ID, int& final_ID, int apa_status);

float CalcDistance(const POINT_I& a, const POINT_I& b);



