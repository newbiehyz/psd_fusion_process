#include "psd_selection_logic.h"
#include <cmath>

namespace PSDSelectionLogic {

int HMIVCUSelect(int &hmi_temp, const int &hmi_select, const int &vcu_select, int &final_select_ID) {
    LOGD("[HMIVCUSELECT IN] HMI:%d HMI temp:%d VCU:%d", hmi_select, hmi_temp, vcu_select);
    
    // HMI 部分
    // 中间变量保存HMI发送的 [0 - ID - 0]，一秒内发送五次
    if (hmi_select) {
        hmi_temp = hmi_select;
    }
    
    // VCU 部分，已从GET获取
    // VCU接收的点选车位 与 HMI接收的点选车位 二选一
    if (vcu_select != 0 && hmi_temp == 0) {
        final_select_ID = vcu_select;
    }
    else if (vcu_select == 0 && hmi_temp != 0) {
        final_select_ID = hmi_temp;
    }
    else if (vcu_select != 0 && hmi_temp != 0 && (vcu_select == hmi_temp)) {
        final_select_ID = hmi_temp;
    }
    else if (vcu_select == 0 && hmi_temp == 0) {
        final_select_ID = 0;
    }
    else {
        final_select_ID = vcu_select;
    }
    
    // 结合HMI和VCU，得到final_select_ID
    return final_select_ID;
}

int RecommendSelectID(const int &final_select, const int &recommend) {
    if (final_select) {
        return final_select;
    }
    else {
        return recommend;
    }
}

int IsParkOut(int apastatus) {
    static int parkout_flag_internal = 0;

    if (apastatus == 3) {
        parkout_flag_internal = 1;
    } else if (apastatus == 2) {
        parkout_flag_internal = 0;
    }
    return parkout_flag_internal;
}

int IsStill(const Loc::App2emap_DR drpose, Loc::App2emap_DR& previous_drpose) {
    static int no_change_count = 0;
    float epsilon = 30.0; // 设置阈值，可以根据需要调整
    bool has_changed = false; // 比较 drpose 和 previous_drpose 是否变化
    int still_threshold = 5; // 静止阈值，连续多少次没有变化算静止
    
    LOGD("[STILL] dr: x:%f, y:%f, yaw: %f,previous: x:%f, y:%f, yaw:%f",
        drpose.x, drpose.y, drpose.canAng,
        previous_drpose.x, previous_drpose.y, previous_drpose.canAng);

    if (fabs(drpose.x - previous_drpose.x) > epsilon ||
        fabs(drpose.y - previous_drpose.y) > epsilon ||
        fabs(drpose.canAng - previous_drpose.canAng) > epsilon) {
        has_changed = true;
    }
    
    LOGD("[STILL] has_changed:%d", has_changed);

    // 如果没有变化，增加连续无变化计数
    if (!has_changed) {
        no_change_count++;
    }
    else {
        no_change_count = 0;  // 有变化时重置计数器
    }
    
    LOGD("[STILL] no_change_count:%d", no_change_count);

    // 如果连续still_threshold次没有变化，则认为是静止状态
    if (no_change_count >= still_threshold) {
        previous_drpose = drpose; // 更新 previous_drpose 为当前的 drpose
        return 1;  // is_Still = 1
    }

    previous_drpose = drpose; // 更新 previous_drpose 为当前的 drpose
    return 0;  // is_Still = 0
}

} // namespace PSDSelectionLogic