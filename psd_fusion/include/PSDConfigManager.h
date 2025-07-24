#pragma once


#include <mutex>
#include <memory>
#include "psd_fusion_process_header.h"
#include "apa_define.h"

// 车辆标定参数结构体
struct VehicleCalibration {
    static constexpr float VEHICLE_LENGTH = 5259.9f;
    static constexpr float REAR_AXLE_CENTER_VEHICLE_REAR = 1136.7f;
    static constexpr float MM_TO_M = 1000.0f;
};

// 配置参数结构体
struct PSDConfig {
    bool FARAWAY_FILTER = false;
    float FARAWAY_SLOTS_LEFT[2] = {-99999.0f, 0.0f};
    float FARAWAY_SLOTS_RIGHT[2] = {0.0f, 99999.0f};
    float FARAWAY_SLOTS_REAR = -99999.0f;
    float FARAWAY_SLOTS_FRONT = 99999.0f;
    
    bool ANGEL_FILTER = false;
    float ANGEL_FILTER_LIMIT = -99999.0f;
    
    bool VCU_TOO_SMALL_FILTER = false;
    float VCU_TOO_SMALL = -99999.0f;
    
    bool PARALLEL_VECTOR_FILTER = false;
    float PARALLEL_VECTOR_LIMIT = -99999.0f;
    
    float NARROWSLOT_THRESHOLD = -99999.0f;
};

// 全局状态结构体
struct PSDGlobalState {
    // 输入状态
    int apa_status = 0;
    int park_request = 0;
    int search_interrupt = 0;
    int is_Still = 0;
    
    // 选择相关状态
    int HMI_select_ID = 0;
    int HMI_temp_ID = 0;
    int VCU_select_ID_ON = 0;
    int RECOMMEND_ID = 0;
    int final_select_ID = 0;
    int final_ID = 0;
    bool recommend_exist = false;
    bool already_has_recommend_slot = false;
    
    // 车位相关状态
    int available_slot_flag_to_statemachine = 0;
    bool isNarrow = false;
    
    // 其他状态
    int parkout_flag = 0;
    int mirror_fold_flag_ahead = 0;
    int mirror_fold_flag = 0;
    
    // 目标车位更新相关
    bool target_slot_already_updated_once = false;
    bool target_slot_in_range = false;
};

// 目标车位记忆结构体
struct TargetSlotMemory {
    Sfus::SfusionSlotType slot_type_before_update;
    POINT_I search_target_center = {0, 0};
    POINT_I search_target_center_world = {0, 0};
    POINT_I new_target_center = {0, 0};
    POINT_I new_target_center_world = {0, 0};
    POINT_I world_slot_memory[4];
    
    static constexpr float MAX_SLOT_MOVE_DIST_MM = 1500.0f;
};

// 单例
class PSDConfigManager {
private:
    PSDConfigManager() = default;
    
    mutable std::mutex config_mutex_;
    mutable std::mutex state_mutex_;
    mutable std::mutex memory_mutex_;
    
    PSDConfig config_;
    PSDGlobalState global_state_;
    TargetSlotMemory target_memory_;
    Loc::App2emap_DR previous_dr_pose_;
    
public:
    // 单例模式
    static PSDConfigManager& getInstance() {
        static PSDConfigManager instance;
        return instance;
    }
    
    // 禁用拷贝构造和赋值
    PSDConfigManager(const PSDConfigManager&) = delete;
    PSDConfigManager& operator=(const PSDConfigManager&) = delete;
    
    // 配置相关方法
    bool loadConfigFromFile(const std::string& filename);
    PSDConfig getConfig() const;
    void updateConfig(const PSDConfig& config);
    
    // 状态相关方法
    PSDGlobalState getGlobalState() const;
    void updateGlobalState(const PSDGlobalState& state);
    
    // 个别状态访问器
    void setApaStatus(int status);
    int getApaStatus() const;
    
    void setHMISelectID(int id);
    int getHMISelectID() const;
    
    void setVCUSelectID(int id);
    int getVCUSelectID() const;
    
    void setFinalSelectID(int id);
    int getFinalSelectID() const;
    
    void setRecommendID(int id);
    int getRecommendID() const;
    
    void setFinalID(int id);
    int getFinalID() const;
    
    // 目标车位记忆相关方法
    TargetSlotMemory getTargetMemory() const;
    void updateTargetMemory(const TargetSlotMemory& memory);
    void setTargetSlotUpdatedOnce(bool updated);
    bool isTargetSlotUpdatedOnce() const;
    
    // DR pose相关
    void setPreviousDRPose(const Loc::App2emap_DR& pose);
    Loc::App2emap_DR getPreviousDRPose() const;
    
    // 重置方法
    void resetState();
    void resetTargetMemory();
    void resetForNewSession();
    
    // 车辆标定参数访问
    static constexpr float getVehicleLength() { return VehicleCalibration::VEHICLE_LENGTH; }
    static constexpr float getRearAxleCenterVehicleRear() { return VehicleCalibration::REAR_AXLE_CENTER_VEHICLE_REAR; }
    static constexpr float getMmToM() { return VehicleCalibration::MM_TO_M; }
};