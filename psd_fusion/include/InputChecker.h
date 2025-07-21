#ifndef INPUT_CHECKER_H
#define INPUT_CHECKER_H


#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"

void ClearHistoricalSlots(int apa_status);
void ClearHistoricalSlotsOnceSearch(int apa_status, bool& has_cleared_once);
void HandleMirrorFold(std::vector<padVisionSlotCoord>& singleframeslots, unsigned long long& slotID, rd::QuadParkingSlots& rd_info);
void UpdateParkoutAndStill(int apa_status, const Loc::App2emap_DR& dr_pose, Loc::App2emap_DR& prev_pose, int& parkout_flag, int& is_still);
void CheckSlotStatus(const std::vector<padVisionSlotCoord>& singleframeslots,
                          const apaSlotListInfo& vis, const apaSlotListInfo& uss, const apaSlotListInfo& fused);




































#endif