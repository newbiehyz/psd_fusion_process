#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"

class slotfusion
{
  public:
    slotfusion();
    ~slotfusion();
    
  public:
    void mergeSlotLists(const apaSlotListInfo &outputSlot_USS,const apaSlotListInfo &outputSlot_VIS,apaSlotListInfo &outputSlot_FUSION); 
    // void mergeUSSleftandright(UssIf_stSlotInfo_t &total_uss_slot, const UssIf_stPLVOutputInfo_t& userData);
    void postprocessUSSslots(UssIf_stPLVOutputInfo_t &total_uss_slot);
    void fillVisonstruct(const UssIf_stSlotInfo_t &total_uss_slot, apaSlotListInfo &uss_slots);
  private:
    double calculateOverlap(const APA_SPACE::SApaPSRect& rect1, const APA_SPACE::SApaPSRect& rect2);
    double calculateIntersectionArea(const APA_SPACE::SApaPSRect& rect1, const APA_SPACE::SApaPSRect& rect2);
    double calculateArea(const APA_SPACE::SApaPSRect& rect);

};

