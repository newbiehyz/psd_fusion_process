#include "PSDOutputManager.h"
#include "math.hpp"

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