#include "PSDOutputManager.h"
#include "math.hpp"


extern StatusDecFusionInput psd2statemachine;

POINT_I Local2Global(const POINT_I& pt_local, const float& x, const float& y, const float& yaw)
{
    float theta = yaw * acos(-1) / 180.0;
    POINT_I pt_global;
    pt_global.x = pt_local.x * cos(theta) + pt_local.y * sin(theta) + x;
    pt_global.y = pt_local.y * cos(theta) - pt_local.x * sin(theta) + y;
    return pt_global;
}

float CalcDistance(const POINT_I& a, const POINT_I& b) {
    float dx = static_cast<float>(a.x - b.x);
    float dy = static_cast<float>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}


void PSDOutputManager::sendVCUSlotList(const apaSlotListInfo& outputSlot_FUSED,
                                      int apa_status,
                                      const PSDGlobalState& currentState,
                                      int is_Still,
                                      unsigned long long current1970_ms) {
    if (apa_status == 2) {
        // SEARCH阶段
        handleSearchPhaseVCU(outputSlot_FUSED, currentState, is_Still, current1970_ms);
    } else {
        // 泊入过程中显示所有车位
        handleNonSearchPhaseVCU(outputSlot_FUSED, current1970_ms);
    }
}

void PSDOutputManager::handleSearchPhaseVCU(const apaSlotListInfo& outputSlot_FUSED,
                                           const PSDGlobalState& currentState,
                                           int is_Still,
                                           unsigned long long current1970_ms) {
    LOGD("VCU display for SEARCH");

    memset(&vcuSlotOutput_, 0, sizeof(Sfus::FusionSlotInfovector));
    int slotlist_size = outputSlot_FUSED.slots_in_cur_frame.size();
    vcuSlotOutput_.slotNum = slotlist_size;

    if (vcuSlotOutput_.slotNum > 0) {
        LOGD("PSD2VCU apa_status: 2, outputslot_fused size: %d", outputSlot_FUSED.slots_in_cur_frame.size());
        
        // 填充车位基本信息
        for (int i = 0; i < slotlist_size && i < 50; ++i) {
            fillBasicSlotInfo(outputSlot_FUSED.slots_in_cur_frame[i], 
                            vcuSlotOutput_.FusionSlotInfo[i], i, current1970_ms);
            
            // 应用车位不释放策略
            applySlotReleaseFilters(outputSlot_FUSED.slots_in_cur_frame[i],
                                  vcuSlotOutput_.FusionSlotInfo[i],
                                  currentState.RECOMMEND_ID);
        }

        // 执行推荐逻辑
        std::vector<Sfus::FusionSlotInfo> vcuSlots(vcuSlotOutput_.FusionSlotInfo, 
                                                  vcuSlotOutput_.FusionSlotInfo + slotlist_size);
        SlotRecommend(currentState.final_select_ID, is_Still, 
                            currentState.RECOMMEND_ID, vcuSlots);

        // 复制回输出结构
        for (int i = 0; i < slotlist_size; ++i) {
            vcuSlotOutput_.FusionSlotInfo[i] = vcuSlots[i];
        }

        // 设置 slotSelectedFlag - 根据 ParkInSlot 标记
        int selected_label = -1;
        for (const auto& slot : outputSlot_FUSED.slots_in_cur_frame) {
            if (slot.rectInfo.ParkInSlot == 1) {
                selected_label = slot.rectInfo.label;
                break;
            }
        }
        
        for (int i = 0; i < vcuSlotOutput_.slotNum; ++i) {
            if (vcuSlotOutput_.FusionSlotInfo[i].slotLabel == selected_label) {
                vcuSlotOutput_.FusionSlotInfo[i].slotSelectedFlag = 1;
            } else {
                vcuSlotOutput_.FusionSlotInfo[i].slotSelectedFlag = 0;
            }
        }

        // 输出日志
        for (int i = 0; i < slotlist_size; ++i) {
            LOGD("[PSD2VCUSLOTLIST] apa_status: 2, slotsize: %d, TYPE: %d, STATUS: %d, ID: %d, displayID: %d, SelectedFlag: %d (%f,%f) (%f,%f) (%f,%f) (%f,%f), timestamp: %llu",
                slotlist_size,
                vcuSlotOutput_.FusionSlotInfo[i].slotType,
                vcuSlotOutput_.FusionSlotInfo[i].slotStatusType,
                vcuSlotOutput_.FusionSlotInfo[i].slotLabel,
                vcuSlotOutput_.FusionSlotInfo[i].displayLabel,
                vcuSlotOutput_.FusionSlotInfo[i].slotSelectedFlag,
                vcuSlotOutput_.FusionSlotInfo[i].pt[0].x, vcuSlotOutput_.FusionSlotInfo[i].pt[0].y,
                vcuSlotOutput_.FusionSlotInfo[i].pt[1].x, vcuSlotOutput_.FusionSlotInfo[i].pt[1].y,
                vcuSlotOutput_.FusionSlotInfo[i].pt[2].x, vcuSlotOutput_.FusionSlotInfo[i].pt[2].y,
                vcuSlotOutput_.FusionSlotInfo[i].pt[3].x, vcuSlotOutput_.FusionSlotInfo[i].pt[3].y,
                vcuSlotOutput_.FusionSlotInfo[i].timeStamp);
        }

        EMC_psd_fusion_process_SetFieldFusionSlotInfovector(vcuSlotOutput_);
    }
}

void PSDOutputManager::handleNonSearchPhaseVCU(const apaSlotListInfo& outputSlot_FUSED,
                                              unsigned long long current1970_ms) {
    LOGD("VCU display For NON-SEARCH");
    
    memset(&vcuSlotOutput_, 0, sizeof(Sfus::FusionSlotInfovector));
    int slotlist_size = outputSlot_FUSED.slots_in_cur_frame.size();
    vcuSlotOutput_.slotNum = slotlist_size;

    if (vcuSlotOutput_.slotNum > 0) {
        LOGD("PSD2VCU apa_status: %d, outputslot_fused size: %d", 5, outputSlot_FUSED.slots_in_cur_frame.size());

        for (int i = 0; i < slotlist_size && i < 50; ++i) {
            const auto& slot = outputSlot_FUSED.slots_in_cur_frame[i];
            auto& vcuSlot = vcuSlotOutput_.FusionSlotInfo[i];

            vcuSlot.slotLabel = slot.rectInfo.label;
            vcuSlot.displayLabel = 0;

            // 使用配置管理器中的车辆参数
            float vehicle_length = PSDConfigManager::getVehicleLength();
            float rear_axle_center = PSDConfigManager::getRearAxleCenterVehicleRear();
            float mm_to_m = PSDConfigManager::getMmToM();

            // 角点转换
            for (int j = 0; j < 4; ++j) {
                vcuSlot.pt[j].x = (slot.rectInfo.pt[j].y - (vehicle_length - rear_axle_center)) / mm_to_m;
                vcuSlot.pt[j].y = slot.rectInfo.pt[j].x / mm_to_m;
                vcuSlot.pt[j].z = 0;
            }

            // 状态判断
            if (slot.rectInfo.ParkInSlot == 1) {
                vcuSlot.slotStatusType = 5;
                vcuSlot.slotSelectedFlag = 1;
            } else {
                vcuSlot.slotSelectedFlag = 0;
                vcuSlot.slotStatusType = (slot.rectInfo.iSodType == 1) ? 4 : 3;
            }

            // 障碍物属性设置
            vcuSlot.stopperInSlot = slot.rectInfo.StopperInSlot;
            vcuSlot.lockInSlot = slot.rectInfo.LockInSlot;
            if (slot.rectInfo.iSodType == 1 && slot.rectInfo.LockInSlot == 1) {
                vcuSlot.slotInnerObType = 3;
                vcuSlot.lockLocation = 3;
            }
            if (slot.rectInfo.iSodType == 1 && slot.rectInfo.OBSInSlot == 1) {
                vcuSlot.slotInnerObType = 2;
            }
            if (slot.rectInfo.iSodType == 1 && slot.rectInfo.OBSInSlot != 1 && 
                slot.rectInfo.LockInSlot != 1 && slot.rectInfo.StopperInSlot != 1) {
                vcuSlot.slotInnerObType = 1;
            }

            vcuSlot.backInAvailableFlag = 1;
            vcuSlot.parkInHeadInSoftButtonCurrentValue = 1;
            vcuSlot.timeStamp = (current1970_ms >= 0) ? static_cast<uint64_t>(current1970_ms) : 0;
        }

        // 输出日志
        for (int i = 0; i < slotlist_size; ++i) {
            LOGD("[PSD2VCUSLOTLIST] IN GUIDANCE, Slot#%d, type: %d, Selected: %d, (%f,%f) (%f,%f) (%f,%f) (%f,%f), timestamp: %llu",
                vcuSlotOutput_.FusionSlotInfo[i].slotLabel,
                vcuSlotOutput_.FusionSlotInfo[i].slotStatusType,
                vcuSlotOutput_.FusionSlotInfo[i].slotSelectedFlag,
                vcuSlotOutput_.FusionSlotInfo[i].pt[0].x, vcuSlotOutput_.FusionSlotInfo[i].pt[0].y,
                vcuSlotOutput_.FusionSlotInfo[i].pt[1].x, vcuSlotOutput_.FusionSlotInfo[i].pt[1].y,
                vcuSlotOutput_.FusionSlotInfo[i].pt[2].x, vcuSlotOutput_.FusionSlotInfo[i].pt[2].y,
                vcuSlotOutput_.FusionSlotInfo[i].pt[3].x, vcuSlotOutput_.FusionSlotInfo[i].pt[3].y,
                vcuSlotOutput_.FusionSlotInfo[i].timeStamp);
        }
    }

    EMC_psd_fusion_process_SetFieldFusionSlotInfovector(vcuSlotOutput_);
}

void PSDOutputManager::fillBasicSlotInfo(const apaSlotInfo& slot, 
                                        Sfus::FusionSlotInfo& vcuSlot,
                                        int index,
                                        unsigned long long current1970_ms) {
    vcuSlot.slotLabel = slot.rectInfo.label;
    vcuSlot.slotType = slot.rectInfo.PStype;

    // 角点转换 - 使用配置管理器中的车辆参数
    float vehicle_length = PSDConfigManager::getVehicleLength();
    float rear_axle_center = PSDConfigManager::getRearAxleCenterVehicleRear();
    float mm_to_m = PSDConfigManager::getMmToM();

    for (int j = 0; j < 4; ++j) {
        vcuSlot.pt[j].x = (slot.rectInfo.pt[j].y - (vehicle_length - rear_axle_center)) / mm_to_m;
        vcuSlot.pt[j].y = slot.rectInfo.pt[j].x / mm_to_m;
        vcuSlot.pt[j].z = 0;
    }

    // 状态判断
    if (slot.rectInfo.iSodType == 1) {
        vcuSlot.slotStatusType = 4; // 被占用
    } else {
        vcuSlot.slotStatusType = 3; // 无占用
    }

    // Unavailable 车位
    if (slot.rectInfo.NotToRelease == 1) {
        vcuSlot.slotStatusType = 6;
    }

    // 障碍物属性
    vcuSlot.stopperInSlot = slot.rectInfo.StopperInSlot;
    vcuSlot.lockInSlot = slot.rectInfo.LockInSlot;
    
    if (slot.rectInfo.iSodType == 1 && slot.rectInfo.LockInSlot == 1) {
        vcuSlot.slotInnerObType = 3;
        vcuSlot.lockLocation = 3;
    }
    if (slot.rectInfo.iSodType == 1 && slot.rectInfo.OBSInSlot == 1) {
        vcuSlot.slotInnerObType = 2;
    }
    if (slot.rectInfo.iSodType == 1 && slot.rectInfo.OBSInSlot != 1 && 
        slot.rectInfo.LockInSlot != 1 && slot.rectInfo.StopperInSlot != 1) {
        vcuSlot.slotInnerObType = 1;
    }

    vcuSlot.backInAvailableFlag = 1;
    vcuSlot.parkInHeadInSoftButtonCurrentValue = 1;
    vcuSlot.timeStamp = (current1970_ms >= 0) ? static_cast<uint64_t>(current1970_ms) : 0;
}

void PSDOutputManager::applySlotReleaseFilters(const apaSlotInfo& slot,
                                              Sfus::FusionSlotInfo& vcuSlot,
                                              int currentRecommendID) {
    // 应用距离范围过滤
    applyDistanceFilter(vcuSlot, vcuSlot);
    
    // 应用角度过滤
    applyAngleFilter(slot, vcuSlot, vcuSlot);
    
    // 应用宽度过滤
    applyWidthFilter(vcuSlot, vcuSlot);
    
    // 应用水平车位向量距离过滤
    applyParallelVectorFilter(slot, vcuSlot, vcuSlot);

    // 如果是推荐车位，设置为推荐状态
    if (slot.rectInfo.iSodType != 1 && slot.rectInfo.label == currentRecommendID) {
        vcuSlot.slotStatusType = 7;
    }
}

void PSDOutputManager::applyDistanceFilter(const Sfus::FusionSlotInfo& vcuSlot,
                                          Sfus::FusionSlotInfo& filteredSlot) {
    auto& configManager = PSDConfigManager::getInstance();
    auto config = configManager.getConfig();
    
    LOGD("[VCU NOTRELEASE1 range] FARAWAY_FILTER: %d, Rear-Front: [%f, %f], Left: [%f, %f], Right:[%f, %f]",
        config.FARAWAY_FILTER, config.FARAWAY_SLOTS_REAR, config.FARAWAY_SLOTS_FRONT,
        config.FARAWAY_SLOTS_LEFT[0], config.FARAWAY_SLOTS_LEFT[1],
        config.FARAWAY_SLOTS_RIGHT[0], config.FARAWAY_SLOTS_RIGHT[1]);

    if (config.FARAWAY_FILTER) {
        // 车位中心点
        float center_x = 0.0f, center_y = 0.0f;
        for (int j = 0; j < 4; ++j) {
            center_x += vcuSlot.pt[j].x;
            center_y += vcuSlot.pt[j].y;
        }
        center_x /= 4.0f;
        center_y /= 4.0f;

        // 判断中心点是否在矩形范围内
        const float FARAWAY_DEADZONE = 0.1f; // 死区（0.2m）
        float rear_limit = config.FARAWAY_SLOTS_REAR + FARAWAY_DEADZONE;
        float front_limit = config.FARAWAY_SLOTS_FRONT - FARAWAY_DEADZONE;
        float left1 = config.FARAWAY_SLOTS_LEFT[0] + FARAWAY_DEADZONE;
        float left2 = config.FARAWAY_SLOTS_LEFT[1] - FARAWAY_DEADZONE;
        float right1 = config.FARAWAY_SLOTS_RIGHT[0] + FARAWAY_DEADZONE;
        float right2 = config.FARAWAY_SLOTS_RIGHT[1] - FARAWAY_DEADZONE;

        bool out_of_x_range = (center_x <= rear_limit || center_x >= front_limit);
        bool out_of_y_range = !((center_y >= left1 && center_y <= left2) ||
                                (center_y >= right1 && center_y <= right2));

        if (vcuSlot.slotType == 0 || vcuSlot.slotType == 2){ // 垂直or斜列
            if (out_of_x_range || out_of_y_range) {
                filteredSlot.slotStatusType = 4;
            }
        }

        LOGD("[VCU NOTRELEASE1 range] ID: %d, center(%.1f,%.1f), x_out: %d, y_out: %d, VCU status: %d",
            filteredSlot.slotLabel,
            center_x,
            center_y,
            out_of_x_range,
            out_of_y_range,
            filteredSlot.slotStatusType);
    }
}

void PSDOutputManager::applyAngleFilter(const apaSlotInfo& slot,
                                       const Sfus::FusionSlotInfo& vcuSlot,
                                       Sfus::FusionSlotInfo& filteredSlot) {
    auto& configManager = PSDConfigManager::getInstance();
    auto config = configManager.getConfig();
    
    LOGD("[VCU NOTRELEASE2 anglelimit] ANGLE_FILTER: %d, ANGEL_FILTER_LIMIT:%f", 
         config.ANGEL_FILTER, config.ANGEL_FILTER_LIMIT);

    if (config.ANGEL_FILTER && slot.rectInfo.PStype != 2) {
        float ax = 4.0f, ay = 0.0f; // 向量1 (-4.0)指向(0,0)
        float bx = vcuSlot.pt[1].x - vcuSlot.pt[0].x;
        float by = vcuSlot.pt[1].y - vcuSlot.pt[0].y; // 向量2 A指向B

        // 夹角计算
        float dot = ax * bx + ay * by;
        float normA = std::sqrt(ax * ax + ay * ay);
        float normB = std::sqrt(bx * bx + by * by);
        if (normA * normB < 1e-6f) return;
        
        float cos_theta = dot / (normA * normB);
        cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
        float angle_deg = std::acos(cos_theta) * 180.0f / M_PI;

        // 死区（5度）
        const float ANGEL_DEADZONE = 5.0f;
        if (angle_deg > config.ANGEL_FILTER_LIMIT + ANGEL_DEADZONE) {
            filteredSlot.slotStatusType = 4;
        }

        LOGD("[VCU NOTRELEASE2 anglelimit] ID: %d, angle_deg: %.f", filteredSlot.slotLabel, angle_deg);
    } else if (slot.rectInfo.PStype == 2) {
        LOGD("[VCU NOTRELEASE2 anglelimit] ID: %d, DIAGONAL SLOT NO LIMIT.", filteredSlot.slotLabel);
    }
}

void PSDOutputManager::applyWidthFilter(const Sfus::FusionSlotInfo& vcuSlot,
                                       Sfus::FusionSlotInfo& filteredSlot) {
    auto& configManager = PSDConfigManager::getInstance();
    auto config = configManager.getConfig();
    
    LOGD("[VCU NOTRELEASE3 abnarrow] VCU_TOO_SMALL_FILTER: %d, VCU_TOO_SMALL: %.3f", 
         config.VCU_TOO_SMALL_FILTER, config.VCU_TOO_SMALL);

    if (config.VCU_TOO_SMALL_FILTER) {
        float VCU_AB = sqrt(pow(vcuSlot.pt[0].x - vcuSlot.pt[1].x, 2) +
                           pow(vcuSlot.pt[0].y - vcuSlot.pt[1].y, 2));
        
        // 死区（0.1m）
        const float TOO_SMALL_DEADZONE = 0.05f;
        if (VCU_AB <= config.VCU_TOO_SMALL - TOO_SMALL_DEADZONE) {
            filteredSlot.slotStatusType = 4;
        }
        
        LOGD("[VCU NOTRELEASE3 abnarrow] ID: %d, VCU_AB: %.3f, VCU occupied: %d",
             filteredSlot.slotLabel, VCU_AB, filteredSlot.slotStatusType);
    }
}

void PSDOutputManager::applyParallelVectorFilter(const apaSlotInfo& slot,
                                                const Sfus::FusionSlotInfo& vcuSlot,
                                                Sfus::FusionSlotInfo& filteredSlot) {
    auto& configManager = PSDConfigManager::getInstance();
    auto config = configManager.getConfig();
    
    LOGD("[VCU NOTRELEASE4 parallel] PARALLEL_VECTOR_FILTER: %d, PARALLEL_VECTOR_LIMIT: %f", 
         config.PARALLEL_VECTOR_FILTER, config.PARALLEL_VECTOR_LIMIT);

    if (config.PARALLEL_VECTOR_FILTER && slot.rectInfo.PStype == 1) {
        POINT_F VCU_car_rear_axle_center = {-4.064f, 0.0f};

        float vector_carrearaxlecenter2parallelAD = (
            (vcuSlot.pt[3].y - vcuSlot.pt[0].y) * VCU_car_rear_axle_center.x
            - (vcuSlot.pt[3].x - vcuSlot.pt[0].x) * VCU_car_rear_axle_center.y
            + vcuSlot.pt[3].x * vcuSlot.pt[0].y
            - vcuSlot.pt[3].y * vcuSlot.pt[0].x
        ) / sqrt(
            pow(vcuSlot.pt[3].y - vcuSlot.pt[0].y, 2)
            + pow(vcuSlot.pt[3].x - vcuSlot.pt[0].x, 2)
        );
        
        // 死区（0.2m）
        const float PARALLEL_VECTOR_DEADZONE = 0.1f;
        if (vcuSlot.pt[0].y >= 0) { // 右侧
            if (vector_carrearaxlecenter2parallelAD <= config.PARALLEL_VECTOR_LIMIT - PARALLEL_VECTOR_DEADZONE) {
                filteredSlot.slotStatusType = 4;
            }
        } else {
            if (vector_carrearaxlecenter2parallelAD >= config.PARALLEL_VECTOR_LIMIT - PARALLEL_VECTOR_DEADZONE) {
                filteredSlot.slotStatusType = 4;
            }
        }
        
        LOGD("[VCU NOTRELEASE4 parallel] PARALLEL VECTOR: %f, ID: %d, set to %d", 
             vector_carrearaxlecenter2parallelAD, filteredSlot.slotLabel, filteredSlot.slotStatusType);
    } else if (slot.rectInfo.PStype != 1) {
        LOGD("[VCU NOTRELEASE4 parallel] NOT PARALLEL SLOT.");
    }
}

void PSDOutputManager::SlotRecommend(int final_select_ID,
                                            int is_Still,
                                            int currentRecommendID,
                                            std::vector<Sfus::FusionSlotInfo>& vcuSlots) {
    // 清除旧的推荐信息
    for (auto& slot : vcuSlots) {
        if (slot.slotStatusType == 7) {
            slot.slotStatusType = 3;
        }
        slot.displayLabel = 0;
    }

    // 获取可用车位列表
    std::vector<Sfus::FusionSlotInfo> vcu_available_slots;
    for (const auto& slot : vcuSlots) {
        if (slot.slotStatusType == 3) {
            vcu_available_slots.push_back(slot);
        }
    }
    LOGD("vcu_available_slots size: %d", vcu_available_slots.size());

    // 更新全局状态中的available_slot_flag_to_statemachine
    auto& configManager = PSDConfigManager::getInstance();
    auto currentState = configManager.getGlobalState();
    if (vcu_available_slots.size() > 0) {
        currentState.available_slot_flag_to_statemachine = 1;
    } else {
        currentState.available_slot_flag_to_statemachine = 0;
    }
    configManager.updateGlobalState(currentState);

    // 推荐逻辑
    if (final_select_ID == 0 && is_Still) {
        // 状态1：当没有点选ID且静止，使用推荐ID
        LOGD("RECOMMEND1: still, Start Recommend!");
        
        POINT_F VCU_car_pose = {-4.0f, 0.0f};
        auto closest_slots = math::findClosesParkingSpots(VCU_car_pose, vcu_available_slots, 10);
        
        const int max_recommend_num = 3;
        
        // Step 1：设置 closest_slots[0] 对应 slot 的 slotStatusType 为 7
        if (!closest_slots.empty()) {
            int targetLabel = closest_slots[0].slotLabel;
            
            // 更新RECOMMEND_ID到配置管理器
            auto& configManager = PSDConfigManager::getInstance();
            auto currentState = configManager.getGlobalState();
            currentState.RECOMMEND_ID = targetLabel;
            
            for (auto& slot : vcuSlots) {
                if (slot.slotLabel == targetLabel) {
                    slot.slotStatusType = 7;
                    break;
                }
            }
            
            // 推荐车位作为final_ID - 调用RecommendSelectID逻辑
            currentState.final_ID = (final_select_ID != 0) ? final_select_ID : currentState.RECOMMEND_ID;
            configManager.updateGlobalState(currentState);
        }
        
        // Step 2：设置推荐车位的 displayLabel
        for (int idx = 1; idx <= max_recommend_num && idx < closest_slots.size(); ++idx) {
            int targetLabel = closest_slots[idx].slotLabel;
            for (auto& slot : vcuSlots) {
                if (slot.slotLabel == targetLabel) {
                    slot.displayLabel = idx;
                    break;
                }
            }
        }
    } else if (final_select_ID == 0 && !is_Still) {
        // 状态2：当没有点选ID且运动，保留RD原状态
        LOGD("RECOMMEND2: not still, NO Recommend!");
        
        // 清除所有状态
        auto& configManager = PSDConfigManager::getInstance();
        auto currentState = configManager.getGlobalState();
        currentState.RECOMMEND_ID = 0;
        currentState.final_select_ID = 0;
        currentState.final_ID = 0;
        currentState.recommend_exist = false;
        currentState.already_has_recommend_slot = false;
        currentState.available_slot_flag_to_statemachine = 0;
        configManager.updateGlobalState(currentState);

        clearRecommendAndSelectState(vcuSlots);

    } else if (final_select_ID != 0 && is_Still) {
        // 状态3：当有点选车位且静止，使用点选ID
        LOGD("RECOMMEND3: still, Select!");
        
        // 设置final_ID为点选ID
        auto& configManager = PSDConfigManager::getInstance();
        auto currentState = configManager.getGlobalState();
        currentState.RECOMMEND_ID = 0;
        currentState.final_ID = final_select_ID;
        currentState.already_has_recommend_slot = false;
        configManager.updateGlobalState(currentState);
        
        for (auto& slot : vcuSlots) {
            slot.displayLabel = 0;
            if (slot.slotLabel == final_select_ID && slot.slotStatusType != 4) {
                slot.slotStatusType = 5; // SELECTED状态
            } else if (slot.slotLabel == final_select_ID && slot.slotStatusType == 4) {
                slot.slotStatusType = 4; // OCCUPIED状态
            } else if (slot.slotLabel != final_select_ID && slot.slotStatusType == 4) {
                slot.slotStatusType = 4; // OCCUPIED状态
            } else if (slot.slotLabel != final_select_ID && slot.slotStatusType == 6) {
                slot.slotStatusType = 6; // unavailable状态
            } else if (slot.slotLabel != final_select_ID && slot.slotStatusType != 4) {
                slot.slotStatusType = 3; // AVAILABLE状态
            }
        }
    } else {
        // 状态4：当有点选车位且运动，清除所有ID
        LOGD("RECOMMEND4: no still, no recommend, no select");
        
        // 清除所有状态
        auto& configManager = PSDConfigManager::getInstance();
        auto currentState = configManager.getGlobalState();
        currentState.HMI_temp_ID = 0;
        currentState.HMI_select_ID = 0;
        currentState.VCU_select_ID_ON = 0;
        currentState.final_select_ID = 0;
        currentState.RECOMMEND_ID = 0;
        currentState.final_ID = 0;
        currentState.recommend_exist = false;
        currentState.already_has_recommend_slot = false;
        currentState.available_slot_flag_to_statemachine = 0;
        configManager.updateGlobalState(currentState);
        
        clearRecommendAndSelectState(vcuSlots);
    }
}

void PSDOutputManager::clearRecommendAndSelectState(std::vector<Sfus::FusionSlotInfo>& vcuSlots) {
    for (auto& slot : vcuSlots) {
        slot.displayLabel = 0;
        if (slot.slotStatusType == 4) {
            slot.slotStatusType = 4; // 占用的保持占用
        } else if (slot.slotStatusType == 6) {
            slot.slotStatusType = 6; // unavailable的保持unavailable
        } else {
            slot.slotStatusType = 3; // 不占用的回到available
        }
    }
}

void PSDOutputManager::sendAPAHandleSlotInfo(const apaSlotListInfo& outputSlot_FUSED,
                                           const PSDGlobalState& currentState,
                                           int parkout_flag) {
    // 发送车位列表给APAHANDLE
    Fsm::FusionSlotInfo2Location psd2location;
    memset(&psd2location, 0, sizeof(Fsm::FusionSlotInfo2Location));
    
    int slotlist_size = outputSlot_FUSED.slots_in_cur_frame.size();
    psd2location.slotNum = slotlist_size;
    
    if (psd2location.slotNum > 0) {
        int j = 0;
        
        for (const auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame) {
            if (j >= slotlist_size || j >= 50) {
                LOGW("[PSD2APAHANDLE] Slot index exceeded limit, size: %d", slotlist_size);
                break;
            }
            
            // 基本信息
            psd2location.fusionSlotInfo[j].slotLabel = psd_m_output.rectInfo.label;
            psd2location.fusionSlotInfo[j].slotType = slottype_rd2vcu(psd_m_output.rectInfo.PStype);
            
            // 融合车位类型判断
            if (psd_m_output.rectInfo.iMaterial == 1) {
                psd2location.fusionSlotInfo[j].fusionSlotType = 3;
            } else {
                if (psd_m_output.rectInfo.label >= 1000 && psd_m_output.rectInfo.label < 10000) {
                    psd2location.fusionSlotInfo[j].fusionSlotType = 0;
                } else {
                    psd2location.fusionSlotInfo[j].fusionSlotType = 1;
                }
            }
            
            // 车位角点坐标
            for (int k = 0; k < 4; ++k) {
                psd2location.fusionSlotInfo[j].pt[k].x = psd_m_output.rectInfo.pt[k].x;
                psd2location.fusionSlotInfo[j].pt[k].y = psd_m_output.rectInfo.pt[k].y;
            }
            
            // 后视镜折叠状态（只有第一个车位设置）
            if (j == 0) {
                psd2location.fusionSlotInfo[j].displayLabel = currentState.mirror_fold_flag;
            } else {
                psd2location.fusionSlotInfo[j].displayLabel = 0;
            }
            
            LOGD("[PSD2APAHANDLE] TOTAL SLOT NUM: %d, Slot#%d, slottype: %d, fusionslottype: %d, mirrorfold: %d (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
                 psd2location.slotNum,
                 psd2location.fusionSlotInfo[j].slotLabel,
                 psd2location.fusionSlotInfo[j].slotType,
                 psd2location.fusionSlotInfo[j].fusionSlotType,
                 psd2location.fusionSlotInfo[j].displayLabel,
                 psd2location.fusionSlotInfo[j].pt[0].x,
                 psd2location.fusionSlotInfo[j].pt[0].y,
                 psd2location.fusionSlotInfo[j].pt[1].x,
                 psd2location.fusionSlotInfo[j].pt[1].y,
                 psd2location.fusionSlotInfo[j].pt[2].x,
                 psd2location.fusionSlotInfo[j].pt[2].y,
                 psd2location.fusionSlotInfo[j].pt[3].x,
                 psd2location.fusionSlotInfo[j].pt[3].y);
            j++;
        }
        
        // 发送车位列表（泊出时不发送）
        if (parkout_flag != 1) {
            EMC_psd_fusion_process_SetFieldFusionSlotInfo2Location(psd2location);
        }
    }
    
    // 发送目标车位ID给APAHANDLE
    if (currentState.final_ID > 0) {
        Fsm::Slotlabel psd2apahandel_targetID;
        psd2apahandel_targetID.targetSlotLabel = currentState.final_ID;
        
        if (parkout_flag != 1) {
            EMC_psd_fusion_process_SetFieldSlotlabel(psd2apahandel_targetID);
        }
        
        LOGD("[PSD2APAHANDLE] target slot id: %d", currentState.final_ID);
    }
}


void PSDOutputManager::sendPlanningAndPerceptionTargetSlot(const apaSlotListInfo& outputSlot_FUSED,
                                                          PSDGlobalState& currentState,
                                                          const padVehiclePose& pose_globaldata,
                                                          int apa_status,
                                                          int parkout_flag,
                                                          unsigned long long current1970_ms,
                                                          const std::vector<std::vector<int>>& imp_camera_id_image,
                                                          Sfus::Sfsuion2DecPlan& psd2planning) {
    auto& configManager = PSDConfigManager::getInstance();
    
    int target_slot_fusionSlotType = 0;
    auto targetMemory = configManager.getTargetMemory();
    
    // 目标车位信息处理
    if (currentState.final_ID > 0 && parkout_flag != 1) {
        if (apa_status != 5) {
            // SEARCH阶段处理
            processSearchPhaseTargetSlot(outputSlot_FUSED, currentState, targetMemory, 
                                       pose_globaldata, psd2planning, target_slot_fusionSlotType);
        } else {
            // GUIDANCE阶段处理
            processGuidancePhaseTargetSlot(outputSlot_FUSED, currentState, targetMemory, 
                                         pose_globaldata, psd2planning, target_slot_fusionSlotType, 
                                         imp_camera_id_image);
        }
        
        // 更新状态到配置管理器
        configManager.updateGlobalState(currentState);
        configManager.updateTargetMemory(targetMemory);
    }
    
    // 发送给Planning模块
    sendPlanningTargetSlot(psd2planning, apa_status, parkout_flag, currentState, 
                          pose_globaldata, target_slot_fusionSlotType);
    
    // 发送给Perception模块
    sendPerceptionTargetSlot(psd2planning, apa_status, parkout_flag, currentState, 
                           current1970_ms, target_slot_fusionSlotType);
}

Sfus::_tSfusionSlotType PSDOutputManager::calculateSlotTypeFromCorners(const apaSlotInfo& slot,
                                                                      int original_slot_type) {
    // 计算AB和AD向量
    double ABx = slot.rectInfo.pt[1].x - slot.rectInfo.pt[0].x;
    double ABy = slot.rectInfo.pt[1].y - slot.rectInfo.pt[0].y;
    double ADx = slot.rectInfo.pt[3].x - slot.rectInfo.pt[0].x;
    double ADy = slot.rectInfo.pt[3].y - slot.rectInfo.pt[0].y;
    
    // 计算角度
    double dotProduct = (ABx * ADx) + (ABy * ADy);
    double magnitudeAB = sqrt(ABx * ABx + ABy * ABy);
    double magnitudeAD = sqrt(ADx * ADx + ADy * ADy);
    double cosTheta = dotProduct / (magnitudeAB * magnitudeAD);
    double angleRadians = acos(cosTheta);
    double angleDegrees = angleRadians * (180.0 / M_PI);
    
    // 根据角度判断车位类型
    if (angleDegrees > 80 && angleDegrees < 100) {
        return slottype_rd2decplan(original_slot_type);
    } else if (angleDegrees <= 80) {
        return Sfus::SLOTTYP_RFOBL;
    } else {
        return Sfus::SLOTTYP_OBL;
    }
}

bool PSDOutputManager::isNarrowSlot(const POINT_I& cornerA, const POINT_I& cornerB) {
    auto& configManager = PSDConfigManager::getInstance();
    auto config = configManager.getConfig();
    
    double dx = cornerB.x - cornerA.x;
    double dy = cornerB.y - cornerA.y;
    double AB_dist = sqrt(dx * dx + dy * dy);
    
    LOGD("AB_dist: %f, threshold: %f", AB_dist, config.NARROWSLOT_THRESHOLD);
    
    return AB_dist <= config.NARROWSLOT_THRESHOLD - 100;
}

Sfus::SfusionSlotSource PSDOutputManager::determineSlotSource(int slot_label) {
    if (slot_label >= 1000 && slot_label < 10000) {
        return Sfus::SLOTSRC_VIS;
    } else if (slot_label >= 10000) {
        return Sfus::SLOTSRC_USS;
    } else {
        return Sfus::SLOTSRC_VIS;
    }
}

int PSDOutputManager::determineFusionSlotType(const apaSlotInfo& slot) {
    if (slot.rectInfo.iMaterial == 1) {
        return 3; // 草砖
    } else if (slot.rectInfo.label >= 1000 && slot.rectInfo.label < 10000) {
        return 0; // 视觉
    } else {
        return 1; // 超声波
    }
}

bool PSDOutputManager::isTargetSlotInCameraRange(const apaSlotInfo& slot,
                                                const std::vector<std::vector<int>>& imp_camera_id_image) {
    POINT_I point_A = {slot.rectInfo.pt[0].x, slot.rectInfo.pt[0].y};
    POINT_I point_B = {slot.rectInfo.pt[1].x, slot.rectInfo.pt[1].y};
    POINT_I pointA_pixel = math::coordConvert_car_center_to_pixel(point_A);
    POINT_I pointB_pixel = math::coordConvert_car_center_to_pixel(point_B);
    
    // 检查像素坐标是否在有效范围内
    if (pointA_pixel.x <= 0 || pointA_pixel.y <= 0 || pointB_pixel.x <= 0 || pointB_pixel.y <= 0 || 
        pointA_pixel.x > 895 || pointA_pixel.y > 895 || pointB_pixel.x > 895 || pointB_pixel.y > 895) {
        return false;
    }
    
    int camera_id_A = imp_camera_id_image[pointA_pixel.y][pointA_pixel.x];
    int camera_id_B = imp_camera_id_image[pointB_pixel.y][pointB_pixel.x];
    
    LOGD("[PSD2PLANNING][CAMERA_RANGE_CHECK] pointA (%d, %d), pointB (%d, %d), camera_id_A = %d, camera_id_B = %d", 
         pointA_pixel.x, pointA_pixel.y, pointB_pixel.x, pointB_pixel.y, camera_id_A, camera_id_B);
    
    return (camera_id_A != 0 && camera_id_B != 0 && camera_id_A == camera_id_B);
}

void PSDOutputManager::processSearchPhaseTargetSlot(const apaSlotListInfo& outputSlot_FUSED,
                                                   PSDGlobalState& currentState,
                                                   TargetSlotMemory& targetMemory,
                                                   const padVehiclePose& pose_globaldata,
                                                   Sfus::Sfsuion2DecPlan& psd2planning,
                                                   int& target_slot_fusionSlotType) {
    LOGD("[PSD2PLANNING][SEARCH_PHASE] Processing target slot ID: %d", currentState.final_ID);
    
    for (size_t i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size(); ++i) {
        if (currentState.final_ID == outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label) {
            const auto& slot = outputSlot_FUSED.slots_in_cur_frame[i];
            
            // 计算车位类型
            psd2planning.targetSlot.slotType = calculateSlotTypeFromCorners(slot, slot.rectInfo.PStype);
            
            // 设置角点坐标
            psd2planning.targetSlot.slotCorners.cornerA.x = slot.rectInfo.pt[0].x;
            psd2planning.targetSlot.slotCorners.cornerA.y = slot.rectInfo.pt[0].y;
            psd2planning.targetSlot.slotCorners.cornerB.x = slot.rectInfo.pt[1].x;
            psd2planning.targetSlot.slotCorners.cornerB.y = slot.rectInfo.pt[1].y;
            psd2planning.targetSlot.slotCorners.cornerC.x = slot.rectInfo.pt[2].x;
            psd2planning.targetSlot.slotCorners.cornerC.y = slot.rectInfo.pt[2].y;
            psd2planning.targetSlot.slotCorners.cornerD.x = slot.rectInfo.pt[3].x;
            psd2planning.targetSlot.slotCorners.cornerD.y = slot.rectInfo.pt[3].y;
            
            // 更新目标车位记忆
            targetMemory.slot_type_before_update = psd2planning.targetSlot.slotType;
            
            // 计算目标车位中心点并转为世界坐标
            POINT_I ptA = {slot.rectInfo.pt[0].x, slot.rectInfo.pt[0].y};
            POINT_I ptB = {slot.rectInfo.pt[1].x, slot.rectInfo.pt[1].y};
            POINT_I ptC = {slot.rectInfo.pt[2].x, slot.rectInfo.pt[2].y};
            POINT_I ptD = {slot.rectInfo.pt[3].x, slot.rectInfo.pt[3].y};
            
            targetMemory.search_target_center.x = (ptA.x + ptB.x + ptC.x + ptD.x) / 4;
            targetMemory.search_target_center.y = (ptA.y + ptB.y + ptC.y + ptD.y) / 4;
            targetMemory.search_target_center_world = Local2Global(targetMemory.search_target_center, 
                                                                  pose_globaldata.coord.x, 
                                                                  pose_globaldata.coord.y, 
                                                                  pose_globaldata.yaw);
            
            // 保存世界坐标系角点记忆
            targetMemory.world_slot_memory[0] = Local2Global(ptA, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
            targetMemory.world_slot_memory[1] = Local2Global(ptB, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
            targetMemory.world_slot_memory[2] = Local2Global(ptC, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
            targetMemory.world_slot_memory[3] = Local2Global(ptD, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
            
            LOGD("[PSD2PLANNING][SEARCH_PHASE] world_slot_memory: A(%d,%d), B(%d,%d), C(%d,%d), D(%d,%d)",
                 targetMemory.world_slot_memory[0].x, targetMemory.world_slot_memory[0].y,
                 targetMemory.world_slot_memory[1].x, targetMemory.world_slot_memory[1].y,
                 targetMemory.world_slot_memory[2].x, targetMemory.world_slot_memory[2].y,
                 targetMemory.world_slot_memory[3].x, targetMemory.world_slot_memory[3].y);
            
            // 判定是否狭窄车位
            currentState.isNarrow = isNarrowSlot(ptA, ptB);
            
            // 设置车位来源和其他属性
            psd2planning.targetSlot.stopper_Dis = slot.rectInfo.StopperDistance;
            psd2planning.targetSlot.slotSource = determineSlotSource(currentState.final_ID);
            target_slot_fusionSlotType = determineFusionSlotType(slot);
            
            break;
        }
    }
}

void PSDOutputManager::processGuidancePhaseTargetSlot(const apaSlotListInfo& outputSlot_FUSED,
                                                     PSDGlobalState& currentState,
                                                     TargetSlotMemory& targetMemory,
                                                     const padVehiclePose& pose_globaldata,
                                                     Sfus::Sfsuion2DecPlan& psd2planning,
                                                     int& target_slot_fusionSlotType,
                                                     const std::vector<std::vector<int>>& imp_camera_id_image) {
    LOGD("[PSD2PLANNING][GUIDANCE_PHASE] world_slot_memory: A(%d,%d), B(%d,%d), C(%d,%d), D(%d,%d)",
         targetMemory.world_slot_memory[0].x, targetMemory.world_slot_memory[0].y,
         targetMemory.world_slot_memory[1].x, targetMemory.world_slot_memory[1].y,
         targetMemory.world_slot_memory[2].x, targetMemory.world_slot_memory[2].y,
         targetMemory.world_slot_memory[3].x, targetMemory.world_slot_memory[3].y);
    
    auto& configManager = PSDConfigManager::getInstance();
    
    for (size_t i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size(); ++i) {
        if (currentState.final_ID == outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label) {
            const auto& slot = outputSlot_FUSED.slots_in_cur_frame[i];
            
            // 更新限位块距离
            psd2planning.targetSlot.stopper_Dis = slot.rectInfo.StopperDistance;
            
            // 只在特定条件下更新一次目标车位
            if (!configManager.isTargetSlotUpdatedOnce() && targetMemory.slot_type_before_update == 1) {
                
                // 计算车位类型
                psd2planning.targetSlot.slotType = calculateSlotTypeFromCorners(slot, slot.rectInfo.PStype);
                
                // 判定是否狭窄车位
                POINT_I ptA = {slot.rectInfo.pt[0].x, slot.rectInfo.pt[0].y};
                POINT_I ptB = {slot.rectInfo.pt[1].x, slot.rectInfo.pt[1].y};
                currentState.isNarrow = isNarrowSlot(ptA, ptB);
                
                // 设置车位来源和材质类型
                psd2planning.targetSlot.slotSource = determineSlotSource(currentState.final_ID);
                target_slot_fusionSlotType = determineFusionSlotType(slot);
                
                // 锁定更新前的车位类型
                if (targetMemory.slot_type_before_update != 0) {
                    psd2planning.targetSlot.slotType = targetMemory.slot_type_before_update;
                }
                
                // 检查目标车位是否在相机范围内
                currentState.target_slot_in_range = isTargetSlotInCameraRange(slot, imp_camera_id_image);
                
                LOGD("[PSD2PLANNING][GUIDANCE_PHASE] target slot in range: %d", currentState.target_slot_in_range);
                
                if (currentState.target_slot_in_range) {
                    // 计算新目标车位中心点
                    POINT_I ptC = {slot.rectInfo.pt[2].x, slot.rectInfo.pt[2].y};
                    POINT_I ptD = {slot.rectInfo.pt[3].x, slot.rectInfo.pt[3].y};
                    
                    targetMemory.new_target_center.x = (ptA.x + ptB.x + ptC.x + ptD.x) / 4;
                    targetMemory.new_target_center.y = (ptA.y + ptB.y + ptC.y + ptD.y) / 4;
                    targetMemory.new_target_center_world = Local2Global(targetMemory.new_target_center, 
                                                                       pose_globaldata.coord.x, 
                                                                       pose_globaldata.coord.y, 
                                                                       pose_globaldata.yaw);
                    
                    // 检查移动距离是否在允许范围内
                    float target_slot_diff = CalcDistance(targetMemory.search_target_center_world, 
                                                         targetMemory.new_target_center_world);
                    LOGD("[PSD2PLANNING][GUIDANCE_PHASE][TARGET_SLOT_CHECK] world center moved: %f, threshold: %f", 
                         target_slot_diff, TargetSlotMemory::MAX_SLOT_MOVE_DIST_MM);
                    
                    if (target_slot_diff < TargetSlotMemory::MAX_SLOT_MOVE_DIST_MM) {
                        // 更新目标车位角点
                        psd2planning.targetSlot.slotCorners.cornerA.x = slot.rectInfo.pt[0].x;
                        psd2planning.targetSlot.slotCorners.cornerA.y = slot.rectInfo.pt[0].y;
                        psd2planning.targetSlot.slotCorners.cornerB.x = slot.rectInfo.pt[1].x;
                        psd2planning.targetSlot.slotCorners.cornerB.y = slot.rectInfo.pt[1].y;
                        psd2planning.targetSlot.slotCorners.cornerC.x = slot.rectInfo.pt[2].x;
                        psd2planning.targetSlot.slotCorners.cornerC.y = slot.rectInfo.pt[2].y;
                        psd2planning.targetSlot.slotCorners.cornerD.x = slot.rectInfo.pt[3].x;
                        psd2planning.targetSlot.slotCorners.cornerD.y = slot.rectInfo.pt[3].y;
                        
                        configManager.setTargetSlotUpdatedOnce(true);
                    }
                }
            }
            break;
        }
    }
}

void PSDOutputManager::sendPlanningTargetSlot(Sfus::Sfsuion2DecPlan& psd2planning,
                                             int apa_status,
                                             int parkout_flag,
                                             const PSDGlobalState& currentState,
                                             const padVehiclePose& pose_globaldata,
                                             int target_slot_fusionSlotType) {
    auto& configManager = PSDConfigManager::getInstance();
    
    if (parkout_flag != 1) {
        if (apa_status == 1 || apa_status == 6 || apa_status == 7 || apa_status == 0) {
            // 清除状态
            configManager.setTargetSlotUpdatedOnce(false);
            memset(&psd2planning, 0, sizeof(Sfus::Sfsuion2DecPlan));
            EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);
        } else {
            // 发送目标车位信息给Planning
            LOGD("[PSD2PLANNING] UPDATED: %d, TIMESTAMP: %llu, APASTATUS: %d, TARGET SLOT type: %d, source: %d, stopper dis: %f, (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
                 configManager.isTargetSlotUpdatedOnce(),
                 psd2planning.timeStamp,
                 apa_status,
                 psd2planning.targetSlot.slotType,
                 psd2planning.targetSlot.slotSource,
                 psd2planning.targetSlot.stopper_Dis,
                 psd2planning.targetSlot.slotCorners.cornerA.x,
                 psd2planning.targetSlot.slotCorners.cornerA.y,
                 psd2planning.targetSlot.slotCorners.cornerB.x,
                 psd2planning.targetSlot.slotCorners.cornerB.y,
                 psd2planning.targetSlot.slotCorners.cornerC.x,
                 psd2planning.targetSlot.slotCorners.cornerC.y,
                 psd2planning.targetSlot.slotCorners.cornerD.x,
                 psd2planning.targetSlot.slotCorners.cornerD.y);
            LOGD("[PSD2PLANNING] target_slot_fusionSlotType: %d", target_slot_fusionSlotType);
            
            EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);
            
            // 输出世界坐标系信息（仅在GUIDANCE阶段且在范围内时）
            if (currentState.target_slot_in_range && apa_status == 5) {
                POINT_I ptA = {static_cast<int>(psd2planning.targetSlot.slotCorners.cornerA.x), 
                              static_cast<int>(psd2planning.targetSlot.slotCorners.cornerA.y)};
                POINT_I ptB = {static_cast<int>(psd2planning.targetSlot.slotCorners.cornerB.x), 
                              static_cast<int>(psd2planning.targetSlot.slotCorners.cornerB.y)};
                POINT_I ptC = {static_cast<int>(psd2planning.targetSlot.slotCorners.cornerC.x), 
                              static_cast<int>(psd2planning.targetSlot.slotCorners.cornerC.y)};
                POINT_I ptD = {static_cast<int>(psd2planning.targetSlot.slotCorners.cornerD.x), 
                              static_cast<int>(psd2planning.targetSlot.slotCorners.cornerD.y)};
                
                POINT_I A_world = Local2Global(ptA, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I B_world = Local2Global(ptB, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I C_world = Local2Global(ptC, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                POINT_I D_world = Local2Global(ptD, pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
                
                LOGD("[PSD2PLANNING] TARGET_SLOT_WORLD A(%d,%d), B(%d,%d), C(%d,%d), D(%d,%d)",
                     A_world.x, A_world.y, B_world.x, B_world.y, C_world.x, C_world.y, D_world.x, D_world.y);
            }
        }
    }
}

void PSDOutputManager::sendPerceptionTargetSlot(const Sfus::Sfsuion2DecPlan& psd2planning,
                                               int apa_status,
                                               int parkout_flag,
                                               const PSDGlobalState& currentState,
                                               unsigned long long current1970_ms,
                                               int target_slot_fusionSlotType) {
    if (parkout_flag != 1) {
        Sfus::SfusionSlots psd2perception;
        memset(&psd2perception, 0, sizeof(Sfus::SfusionSlots));
        
        if (psd2planning.targetSlot.slotCorners.cornerA.x != 0) {
            // 设置角点坐标
            psd2perception.slotCorners.cornerA.x = psd2planning.targetSlot.slotCorners.cornerA.x;
            psd2perception.slotCorners.cornerA.y = psd2planning.targetSlot.slotCorners.cornerA.y;
            psd2perception.slotCorners.cornerB.x = psd2planning.targetSlot.slotCorners.cornerB.x;
            psd2perception.slotCorners.cornerB.y = psd2planning.targetSlot.slotCorners.cornerB.y;
            psd2perception.slotCorners.cornerC.x = psd2planning.targetSlot.slotCorners.cornerC.x;
            psd2perception.slotCorners.cornerC.y = psd2planning.targetSlot.slotCorners.cornerC.y;
            psd2perception.slotCorners.cornerD.x = psd2planning.targetSlot.slotCorners.cornerD.x;
            psd2perception.slotCorners.cornerD.y = psd2planning.targetSlot.slotCorners.cornerD.y;
            
            psd2perception.slotType = psd2planning.targetSlot.slotType;
            psd2perception.slotSource = psd2planning.targetSlot.slotSource;
            
            // 设置目标位置类型（基于融合车位类型）
            switch (target_slot_fusionSlotType) {
                case 0:
                    psd2perception.targetPosType = Sfus::POSHEADING_NULL;
                    break;
                case 1:
                    psd2perception.targetPosType = Sfus::POSHEADING_AB;
                    break;
                case 3:
                    psd2perception.targetPosType = Sfus::POSHEADING_CD;
                    break;
                default:
                    psd2perception.targetPosType = Sfus::POSHEADING_NULL;
                    break;
            }
            
            psd2perception.timeStamp = (current1970_ms >= 0) ? static_cast<uint64_t>(current1970_ms) : 0;
            psd2perception.flag_valid = currentState.mirror_fold_flag; // 后视镜折叠状态
        }
        
        LOGD("[PSD2PERCEPTION] TIMESTAMP: %llu, APASTATUS: %d, TARGET SLOT type: %d, targetPosType: %d, source: %d, mirrorfold: %d (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
             psd2perception.timeStamp,
             apa_status,
             psd2perception.slotType,
             psd2perception.targetPosType,
             psd2perception.slotSource,
             psd2perception.flag_valid,
             psd2perception.slotCorners.cornerA.x,
             psd2perception.slotCorners.cornerA.y,
             psd2perception.slotCorners.cornerB.x,
             psd2perception.slotCorners.cornerB.y,
             psd2perception.slotCorners.cornerC.x,
             psd2perception.slotCorners.cornerC.y,
             psd2perception.slotCorners.cornerD.x,
             psd2perception.slotCorners.cornerD.y);
        
        EMC_psd_fusion_process_SetFieldSfusionSlots(psd2perception);
    }
}


void PSDOutputManager::sendStateMachineInfo(int slotlist_size,
                                           int apa_status,
                                           PSDGlobalState& currentState,
                                           Sfus::_tSfusionSlotType target_slot_type) {
    auto& configManager = PSDConfigManager::getInstance();
    
    // 设置车位数量
    stateMachineOutput_.aps_apaParkPlaceNum = slotlist_size;

    // 处理特定状态下的清零逻辑
    if (apa_status == 1 || apa_status == 6 || apa_status == 7) {
        stateMachineOutput_.aps_apaParkType = 0;
        stateMachineOutput_.aps_apaParkPlaceNum = 0;
        stateMachineOutput_.aps_apaHighlightSlot = 0;
        
        // 清零状态
        currentState.final_ID = 0;
        currentState.HMI_temp_ID = 0;
        configManager.updateGlobalState(currentState);
    }
    
    // 根据是否有目标车位设置状态
    if (currentState.final_ID > 0) { 
        // 有点选或推荐
        stateMachineOutput_.aps_apaParkType = slottype_decplan2statemachine(target_slot_type);
        
        // 设置融合类型
        if (currentState.final_ID >= 10000) {
            stateMachineOutput_.aps_apaParkFusionType = 1;
        } else {
            stateMachineOutput_.aps_apaParkFusionType = 0;
        }

        // 设置高亮车位标志
        if (slotlist_size > 0) {
            stateMachineOutput_.aps_apaHighlightSlot = 1;
        }

        stateMachineOutput_.aps_apaAvailableSlot = 1;
        
        // 设置窄车位标志
        stateMachineOutput_.aps_apaNarrowSlot = currentState.isNarrow ? 1 : 0;

    } else { 
        // 无点选或推荐
        stateMachineOutput_.aps_apaParkType = 0;
        stateMachineOutput_.aps_apaHighlightSlot = 0;
        stateMachineOutput_.aps_apaAvailableSlot = currentState.available_slot_flag_to_statemachine;
        stateMachineOutput_.aps_apaNarrowSlot = 0;
    }
    
    LOGD("[PSD2STATEMACHINE] SELECT ID: %d, ParkType = %d, ParkFusionType: %d, NarrowSlot: %d, ParkPlaceNum: %d, AvailableSlot: %d, HighlightSlot: %d",
         currentState.final_ID,
         stateMachineOutput_.aps_apaParkType,
         stateMachineOutput_.aps_apaParkFusionType,
         stateMachineOutput_.aps_apaNarrowSlot,
         stateMachineOutput_.aps_apaParkPlaceNum,
         stateMachineOutput_.aps_apaAvailableSlot,
         stateMachineOutput_.aps_apaHighlightSlot);
    
    // 发送给StateMachine模块
    S2S_MCore_Bridge_SetSigStatusDecFusionInput(&stateMachineOutput_);
}

void PSDOutputManager::sendUSSTargetSlotID(int final_ID) {
    short targetUssSlotID = 0;

    if (final_ID >= 10000) {
        if (final_ID > SHRT_MAX) {
            LOGD("[PSD2USS] Error: final_ID %d exceeds short range!", final_ID);
        } else {
            targetUssSlotID = static_cast<short>(final_ID);
            S2S_MCore_Bridge_SetSigtargetUssSlotLabel(&targetUssSlotID);
            LOGD("[PSD2USS] Sent target USS slot ID: %d", targetUssSlotID);
        }
    } else {
        LOGD("[PSD2USS] final_ID %d is not USS slot (< 10000), no USS target sent", final_ID);
    }
}




void PSDOutputManager::sendControlBumpInfo(const apaSlotListInfo& outputSlot_FUSED, 
                                          int final_ID, 
                                          int parkout_flag) {
    APAControlBumpInput bumpInfo;
    memset(&bumpInfo, 0, sizeof(APAControlBumpInput));

    if (final_ID != 0) {
        // 查找目标车位的限位块信息
        if (!findTargetSlotBumpInfo(outputSlot_FUSED, final_ID, bumpInfo)) {
            // 找不到目标车位，设置默认值
            setDefaultBumpInfo(bumpInfo);
        }
    } else if (parkout_flag == 1) {
        // 泊出时，限位块设置为默认值
        setDefaultBumpInfo(bumpInfo);
    }

    LOGD("[PSD2CONTROL] LimitBar for target slot: (%d, %d), (%d, %d)", 
         bumpInfo.apc_LimitBarX[0], bumpInfo.apc_LimitBarY[0],
         bumpInfo.apc_LimitBarX[1], bumpInfo.apc_LimitBarY[1]);

    // 发送给Control模块
    S2S_MCore_Bridge_SetSigAPAControlBumpInput(&bumpInfo);
}




bool PSDOutputManager::findTargetSlotBumpInfo(const apaSlotListInfo& outputSlot_FUSED,
                                             int final_ID,
                                             APAControlBumpInput& bumpInfo) {
    for (const auto& slot : outputSlot_FUSED.slots_in_cur_frame) {
        if (slot.rectInfo.label == final_ID) {
            // 找到目标车位，复制限位块坐标
            for (int j = 0; j < 2; ++j) {
                bumpInfo.apc_LimitBarX[j] = static_cast<tInt16>(slot.rectInfo.StopperX[j]);
                bumpInfo.apc_LimitBarY[j] = static_cast<tInt16>(slot.rectInfo.StopperY[j]);
            }

            // 若限位块均为(0,0)，则设置为默认值
            if (bumpInfo.apc_LimitBarX[0] == 0 && bumpInfo.apc_LimitBarY[0] == 0 &&
                bumpInfo.apc_LimitBarX[1] == 0 && bumpInfo.apc_LimitBarY[1] == 0) {
                setDefaultBumpInfo(bumpInfo);
            }
            return true;
        }
    }
    return false;
}

void PSDOutputManager::setDefaultBumpInfo(APAControlBumpInput& bumpInfo) {
    // 设置为(-20000,-20000)表示无限位块
    bumpInfo.apc_LimitBarX[0] = -20000;
    bumpInfo.apc_LimitBarY[0] = -20000;
    bumpInfo.apc_LimitBarX[1] = -20000;
    bumpInfo.apc_LimitBarY[1] = -20000;
}