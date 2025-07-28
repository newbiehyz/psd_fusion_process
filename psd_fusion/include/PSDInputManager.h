#pragma once

#include "psd_fusion_process_header.h"
#include "apa_define.h"
#include "PSDConfigGlobalManager.h"
#include <vector>
#include <string>

/**
 * @brief PSD输入管理器
 * 负责处理用户输入选择、车辆状态检测等输入相关逻辑
 */
class PSDInputManager {
public:
    PSDInputManager() = default;
    ~PSDInputManager() = default;

    /**
     * @brief 从CSV文件加载IPM相机ID图像数据
     * @param filename CSV文件路径
     * @param image 输出的图像数据
     * @return 是否成功加载
     */
    static bool loadIPMCameraIdFromCSV(const std::string& filename, 
                                       std::vector<std::vector<int>>& image);

    /**
     * @brief 处理HMI和VCU的车位选择逻辑
     * @param hmi_temp HMI临时选择ID（可修改）
     * @param hmi_select HMI选择ID
     * @param vcu_select VCU选择ID
     * @return 最终选择的车位ID
     */
    int processHMIVCUSelection(int& hmi_temp, 
                              const int& hmi_select, 
                              const int& vcu_select);

    /**
     * @brief 处理推荐和选择逻辑
     * @param final_select 最终选择ID
     * @param recommend 推荐ID
     * @return 最终确定的车位ID
     */
    int processRecommendSelection(const int& final_select, const int& recommend);

    /**
     * @brief 判断是否为泊出状态
     * @param apa_status APA状态
     * @return 泊出标志
     */
    int determineParkOutFlag(int apa_status);

    /**
     * @brief 检测车辆是否静止
     * @param current_dr_pose 当前DR位置
     * @param previous_dr_pose 上一次DR位置（引用，会被更新）
     * @return 是否静止 (1=静止, 0=运动)
     */
    int detectVehicleStillState(const Loc::App2emap_DR& current_dr_pose, 
                               Loc::App2emap_DR& previous_dr_pose);

private:
    // 静止检测相关常量
    static constexpr float STILL_POSITION_THRESHOLD = 30.0f;  // 位置变化阈值(mm)
    static constexpr float STILL_ANGLE_THRESHOLD = 30.0f;     // 角度变化阈值(degree)
    static constexpr int STILL_COUNT_THRESHOLD = 5;           // 静止计数阈值
    
    // 静止检测状态
    static int still_count_;
};