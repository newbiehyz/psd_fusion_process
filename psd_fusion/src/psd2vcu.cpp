#include "psd2vcu.h"


std::vector<Sfus::FusionSlotInfo> fixedSlots;

void FixedStillVCUSlots(Sfus::FusionSlotInfovector& psd2vcu, int is_Still) {
    if (is_Still == 1) {
        if (fixedSlots.empty()) {
            // Store the initial values when is_Still first becomes 1
            fixedSlots.assign(psd2vcu.FusionSlotInfo, psd2vcu.FusionSlotInfo + psd2vcu.slotNum);
        } else {
            // Keep the values fixed
            for (int i = 0; i < psd2vcu.slotNum; ++i) {
                for (int j = 0; j < 4; ++j) {
                    psd2vcu.FusionSlotInfo[i].pt[j].x = fixedSlots[i].pt[j].x;
                    psd2vcu.FusionSlotInfo[i].pt[j].y = fixedSlots[i].pt[j].y;
                }
            }
        }
    } else {
        // Reset the stored values when is_Still != 1
        fixedSlots.clear();
    }
}