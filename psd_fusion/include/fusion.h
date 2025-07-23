#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"
#include "iou.h"

class slotfusion
{
  public:
    slotfusion();
    ~slotfusion();
    
  public:
    void mergeSlotLists(const apaSlotListInfo &outputSlot_USS, apaSlotListInfo &outputSlot_VIS,apaSlotListInfo &outputSlot_FUSION); 
    void postprocessUSSslots(UssIf_stPLVOutputInfo_t &total_uss_slot);
    void fillVisonstruct(const UssIf_stPLVOutputInfo_t &total_uss_slot, apaSlotListInfo &uss_slots);
    void clearInvalidUSSslots(apaSlotListInfo& uss_slots);
  private:
    bool deleteinvalidslot(UssIf_stSlotProperty_t uss_slot);
    double calculateOverlap(const APA_SPACE::SApaPSRect& rect1, const APA_SPACE::SApaPSRect& rect2);
    double calculateIntersectionArea(const APA_SPACE::SApaPSRect& rect1, const APA_SPACE::SApaPSRect& rect2);
    double calculateArea(const APA_SPACE::SApaPSRect& rect);
    vector<apaSlotInfo>::iterator existed_in_psinfo(const apaSlotInfo& rect_new, apaSlotListInfo& vison_slot_list, bool& mis_detect_flag);

};

