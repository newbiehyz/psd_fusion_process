#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"

float computeDistance(const Sfus::SlotPoint& p1, const Sfus::SlotPoint& p2);

void filterSlotPosition(Sfus::FusionSlotInfo& slot, const Sfus::FusionSlotInfo& prevSlot);

void normalizeSlotSize(Sfus::FusionSlotInfo& slot);

void PSD2VCUSlotsFilter(Sfus::FusionSlotInfovector& psd2vcu, const Sfus::FusionSlotInfovector& prevPsd2vcu);
