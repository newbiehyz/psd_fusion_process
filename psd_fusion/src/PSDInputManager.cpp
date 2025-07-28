#include "PSDInputManager.h"
#include <fstream>
#include <sstream>
#include <cmath>

// 静态成员变量定义
int PSDInputManager::still_count_ = 0;

bool PSDInputManager::loadIPMCameraIdFromCSV(const std::string& filename, 
                                            std::vector<std::vector<int>>& image) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        LOGE("Failed to open IPM camera ID file: %s", filename.c_str());
        return false;
    }

    std::string line;
    int row = 0;
    
    while (std::getline(file, line) && row < image.size()) {
        std::stringstream ss(line);
        std::string value;
        int col = 0;
        
        while (std::getline(ss, value, ',') && col < image[row].size()) {
            try {
                image[row][col] = std::stoi(value);
                ++col;
            } catch (const std::exception& e) {
                LOGW("Failed to convert value at row %d, col %d: %s", row, col, value.c_str());
                image[row][col] = 0; // 默认值
                ++col;
            }
        }
        ++row;
    }
    
    file.close();
    LOGD("Successfully loaded IPM camera ID image from: %s", filename.c_str());
    return true;
}

int PSDInputManager::processHMIVCUSelection(int& hmi_temp, 
                                           const int& hmi_select, 
                                           const int& vcu_select) {
    auto& configManager = PSDConfigGlobalManager::getInstance();
    
    LOGD("[INPUT SELECTION] HMI select: %d, HMI temp: %d, VCU select: %d", 
         hmi_select, hmi_temp, vcu_select);

    // HMI部分：保存临时选择ID
    if (hmi_select != 0) {
        hmi_temp = hmi_select;
        configManager.setHMISelectID(hmi_select);
    }
    
    // VCU部分：直接保存选择ID
    if (vcu_select != 0) {
        configManager.setVCUSelectID(vcu_select);
    }
    
    // 选择优先级逻辑：VCU > HMI
    int final_select_ID = 0;
    
    if (vcu_select != 0 && hmi_temp == 0) {
        // 只有VCU选择
        final_select_ID = vcu_select;
    } 
    else if (vcu_select == 0 && hmi_temp != 0) {
        // 只有HMI选择
        final_select_ID = hmi_temp;
    } 
    else if (vcu_select != 0 && hmi_temp != 0) {
        // 两者都有选择
        if (vcu_select == hmi_temp) {
            // 选择相同，使用该ID
            final_select_ID = hmi_temp;
        } else {
            // 选择不同，优先使用VCU
            final_select_ID = vcu_select;
        }
    }
    // 两者都没有选择时，final_select_ID 保持为0
    
    configManager.setFinalSelectID(final_select_ID);
    
    LOGD("[INPUT SELECTION] Final selection ID: %d", final_select_ID);
    return final_select_ID;
}

int PSDInputManager::processRecommendSelection(const int& final_select, const int& recommend) {
    auto& configManager = PSDConfigGlobalManager::getInstance();
    
    int result = (final_select != 0) ? final_select : recommend;
    
    configManager.setFinalID(result);
    
    LOGD("[RECOMMEND SELECTION] Final select: %d, Recommend: %d, Result: %d", 
         final_select, recommend, result);
    
    return result;
}

int PSDInputManager::determineParkOutFlag(int apa_status) {
    auto& configManager = PSDConfigGlobalManager::getInstance();
    auto state = configManager.getGlobalState();
    
    // 根据APA状态判断泊出标志
    if (apa_status == 3) {
        state.parkout_flag = 1;  // 泊出状态
    } else if (apa_status == 2) {
        state.parkout_flag = 0;  // 搜索状态，非泊出
    }
    // 其他状态保持原有标志
    
    configManager.updateGlobalState(state);
    
    LOGD("[PARK OUT] APA status: %d, Park out flag: %d", apa_status, state.parkout_flag);
    return state.parkout_flag;
}

int PSDInputManager::detectVehicleStillState(const Loc::App2emap_DR& current_dr_pose, 
                                            Loc::App2emap_DR& previous_dr_pose) {
    auto& configManager = PSDConfigGlobalManager::getInstance();
    previous_dr_pose = configManager.getPreviousDRPose();
    
    bool has_significant_change = false;
    
    LOGD("[STILL DETECTION] Current: (%.1f, %.1f, %.1f°), Previous: (%.1f, %.1f, %.1f°)",
         current_dr_pose.x, current_dr_pose.y, current_dr_pose.canAng,
         previous_dr_pose.x, previous_dr_pose.y, previous_dr_pose.canAng);

    // 检查位置和角度变化是否超过阈值
    if (std::fabs(current_dr_pose.x - previous_dr_pose.x) > STILL_POSITION_THRESHOLD ||
        std::fabs(current_dr_pose.y - previous_dr_pose.y) > STILL_POSITION_THRESHOLD ||
        std::fabs(current_dr_pose.canAng - previous_dr_pose.canAng) > STILL_ANGLE_THRESHOLD) {
        has_significant_change = true;
    }
    
    LOGD("[STILL DETECTION] Has significant change: %d", has_significant_change);

    // 更新静止计数
    if (!has_significant_change) {
        still_count_++;
    } else {
        still_count_ = 0;  // 有变化时重置计数器
    }
    
    LOGD("[STILL DETECTION] Still count: %d", still_count_);

    // 判断是否静止
    int is_still_result = (still_count_ >= STILL_COUNT_THRESHOLD) ? 1 : 0;
    
    // 更新上一次的DR位置到配置管理器
    configManager.setPreviousDRPose(current_dr_pose);
    previous_dr_pose = current_dr_pose; // 同时更新传入的引用
    
    // 更新静止状态到全局状态
    auto state = configManager.getGlobalState();
    state.is_Still = is_still_result;
    configManager.updateGlobalState(state);
    
    LOGD("[STILL DETECTION] Vehicle is still: %d", is_still_result);
    return is_still_result;
}