#include "PSDConfigManager.h"
#include "json.hpp"
#include <fstream>
#include <iostream>

using json = nlohmann::json;

bool PSDConfigManager::loadConfigFromFile(const std::string& filename) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        config_.FARAWAY_FILTER = false;
        std::cout << "无法打开配置文件: " << filename << std::endl;
        return false;
    }
    
    try {
        json j;
        inFile >> j;
        
        j.at("calib").at("FARAWAY_FILTER").get_to(config_.FARAWAY_FILTER);
        
        auto FARAWAY_SLOTS_LEFT_RANGE = j.at("calib").at("FARAWAY_SLOTS_LEFT");
        config_.FARAWAY_SLOTS_LEFT[0] = FARAWAY_SLOTS_LEFT_RANGE[0];
        config_.FARAWAY_SLOTS_LEFT[1] = FARAWAY_SLOTS_LEFT_RANGE[1];
        
        auto FARAWAY_SLOTS_RIGHT_RANGE = j.at("calib").at("FARAWAY_SLOTS_RIGHT");
        config_.FARAWAY_SLOTS_RIGHT[0] = FARAWAY_SLOTS_RIGHT_RANGE[0];
        config_.FARAWAY_SLOTS_RIGHT[1] = FARAWAY_SLOTS_RIGHT_RANGE[1];
        
        j.at("calib").at("FARAWAY_SLOTS_REAR").get_to(config_.FARAWAY_SLOTS_REAR);
        j.at("calib").at("FARAWAY_SLOTS_FRONT").get_to(config_.FARAWAY_SLOTS_FRONT);
        j.at("calib").at("ANGEL_FILTER").get_to(config_.ANGEL_FILTER);
        j.at("calib").at("ANGEL_FILTER_LIMIT").get_to(config_.ANGEL_FILTER_LIMIT);
        j.at("calib").at("VCU_TOO_SMALL_FILTER").get_to(config_.VCU_TOO_SMALL_FILTER);
        j.at("calib").at("VCU_TOO_SMALL").get_to(config_.VCU_TOO_SMALL);
        j.at("calib").at("PARALLEL_VECTOR_FILTER").get_to(config_.PARALLEL_VECTOR_FILTER);
        j.at("calib").at("PARALLEL_VECTOR_LIMIT").get_to(config_.PARALLEL_VECTOR_LIMIT);
        j.at("calib").at("NARROWSLOT_THRESHOLD").get_to(config_.NARROWSLOT_THRESHOLD);
        
    } catch (json::exception& e) {
        std::cout << "配置文件解析错误: " << e.what() << std::endl;
        return false;
    }
    
    inFile.close();
    return true;
}

PSDConfig PSDConfigManager::getConfig() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void PSDConfigManager::updateConfig(const PSDConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;
}

PSDGlobalState PSDConfigManager::getGlobalState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return global_state_;
}

void PSDConfigManager::updateGlobalState(const PSDGlobalState& state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    global_state_ = state;
}

void PSDConfigManager::setApaStatus(int status) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    global_state_.apa_status = status;
}

int PSDConfigManager::getApaStatus() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return global_state_.apa_status;
}

void PSDConfigManager::setHMISelectID(int id) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    global_state_.HMI_select_ID = id;
}

int PSDConfigManager::getHMISelectID() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return global_state_.HMI_select_ID;
}

void PSDConfigManager::setVCUSelectID(int id) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    global_state_.VCU_select_ID_ON = id;
}

int PSDConfigManager::getVCUSelectID() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return global_state_.VCU_select_ID_ON;
}

void PSDConfigManager::setFinalSelectID(int id) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    global_state_.final_select_ID = id;
}

int PSDConfigManager::getFinalSelectID() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return global_state_.final_select_ID;
}

void PSDConfigManager::setRecommendID(int id) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    global_state_.RECOMMEND_ID = id;
}

int PSDConfigManager::getRecommendID() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return global_state_.RECOMMEND_ID;
}

void PSDConfigManager::setFinalID(int id) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    global_state_.final_ID = id;
}

int PSDConfigManager::getFinalID() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return global_state_.final_ID;
}

TargetSlotMemory PSDConfigManager::getTargetMemory() const {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    return target_memory_;
}

void PSDConfigManager::updateTargetMemory(const TargetSlotMemory& memory) {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    target_memory_ = memory;
}

void PSDConfigManager::setTargetSlotUpdatedOnce(bool updated) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    global_state_.target_slot_already_updated_once = updated;
}

bool PSDConfigManager::isTargetSlotUpdatedOnce() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return global_state_.target_slot_already_updated_once;
}

void PSDConfigManager::setPreviousDRPose(const Loc::App2emap_DR& pose) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    previous_dr_pose_ = pose;
}

Loc::App2emap_DR PSDConfigManager::getPreviousDRPose() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return previous_dr_pose_;
}

void PSDConfigManager::resetState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    global_state_ = PSDGlobalState{}; // 重置为默认值
}

void PSDConfigManager::resetTargetMemory() {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    target_memory_ = TargetSlotMemory{}; // 重置为默认值
}

void PSDConfigManager::resetForNewSession() {
    resetState();
    resetTargetMemory();
}