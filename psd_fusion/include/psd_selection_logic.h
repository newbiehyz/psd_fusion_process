#pragma once

#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"

namespace PSDSelectionLogic {
    // HMI和VCU选择逻辑
    int HMIVCUSelect(int &hmi_temp, const int &hmi_select, const int &vcu_select, int &final_select_ID);
    
    // 推荐选择逻辑
    int RecommendSelectID(const int &final_select, const int &recommend);
    
    // 泊出状态判断
    int IsParkOut(int apastatus);
    
    // 静止状态判断
    int IsStill(const Loc::App2emap_DR drpose, Loc::App2emap_DR& previous_drpose);
}