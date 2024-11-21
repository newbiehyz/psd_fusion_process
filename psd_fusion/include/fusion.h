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
    void mergeSlotLists(const apaSlotListInfo &outputSlot_USS,const apaSlotListInfo &outputSlot_VIS,apaSlotListInfo &outputSlot_FUSION); 
    double calculateOverlap(const APA_SPACE::SApaPSRect& rect1, const APA_SPACE::SApaPSRect& rect2);
    double calculateIntersectionArea(const APA_SPACE::SApaPSRect& rect1, const APA_SPACE::SApaPSRect& rect2);
    double calculateArea(const APA_SPACE::SApaPSRect& rect);

};

