#pragma once

#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"
#include "PSD_FusionModuleIF.h"
#include <vector>

extern int slotlist_size;

void ProcessMapInfoSlot(const Loc::MapInfo& map_info,
                                    const padVehiclePose& pose_globaldata,
                                    apaSlotListInfo& outputSlot_VIS,
                                    PSD_FusionModuleIF& fusionModule);


void ProcessUssSlots(UssIf_stPLVOutputInfo_t& uss_info,
                     UssIf_stPLVOutputInfo_t& uss_info_restruct,
                     apaSlotListInfo& outputSlot_USS,
                     apaSlotListInfo& outputSlot_VIS,
                     apaSlotListInfo& outputSlot_FUSED,
                     slotfusion& fusionslot);
