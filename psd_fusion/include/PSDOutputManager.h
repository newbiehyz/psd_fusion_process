#pragma once

#include "psd_fusion_process_header.h"
#include "apa_define.h"
#include "PSDConfigManager.h"

/**
 * @brief PSD输出管理器
 * 负责管理所有下游输出，包括VCU、Planning、Control、StateMachine等
 */
class PSDOutputManager {
public:
    PSDOutputManager() = default;
    ~PSDOutputManager() = default;

    /**
     * @brief 发送限位块信息给Control模块
     * @param outputSlot_FUSED 融合车位信息
     * @param final_ID 目标车位ID
     * @param parkout_flag 泊出标志
     */
    void sendControlBumpInfo(const apaSlotListInfo& outputSlot_FUSED, 
                           int final_ID, 
                           int parkout_flag);

    /**
     * @brief 发送车位列表给VCU模块
     * @param outputSlot_FUSED 融合车位信息
     * @param apa_status APA状态
     * @param currentState 全局状态
     * @param is_Still 是否静止
     * @param current1970_ms 当前时间戳
     */
    void sendVCUSlotList(const apaSlotListInfo& outputSlot_FUSED,
                        int apa_status,
                        const PSDGlobalState& currentState, 
                        int is_Still,
                        unsigned long long current1970_ms);

private:
    /**
     * @brief 查找目标车位中的限位块信息
     * @param outputSlot_FUSED 融合车位信息
     * @param final_ID 目标车位ID
     * @param bumpInfo 输出的限位块信息
     * @return 是否找到目标车位
     */
    bool findTargetSlotBumpInfo(const apaSlotListInfo& outputSlot_FUSED,
                               int final_ID,
                               APAControlBumpInput& bumpInfo);

    /**
     * @brief 设置默认的限位块信息（无限位块或泊出时）
     * @param bumpInfo 输出的限位块信息
     */
    void setDefaultBumpInfo(APAControlBumpInput& bumpInfo);

    // VCU相关私有方法
    /**
     * @brief 处理SEARCH阶段的VCU输出
     */
    void handleSearchPhaseVCU(const apaSlotListInfo& outputSlot_FUSED,
                             const PSDGlobalState& currentState,
                             int is_Still,
                             unsigned long long current1970_ms);

    /**
     * @brief 处理非SEARCH阶段的VCU输出（泊入过程中）
     */
    void handleNonSearchPhaseVCU(const apaSlotListInfo& outputSlot_FUSED,
                                 unsigned long long current1970_ms);

    /**
     * @brief 填充基本车位信息到VCU格式
     */
    void fillBasicSlotInfo(const apaSlotInfo& slot, 
                          Sfus::FusionSlotInfo& vcuSlot,
                          int index,
                          unsigned long long current1970_ms);

    /**
     * @brief 应用车位不释放策略
     */
    void applySlotReleaseFilters(const apaSlotInfo& slot,
                               Sfus::FusionSlotInfo& vcuSlot,
                               int currentRecommendID);

    /**
     * @brief 距离范围过滤
     */
    void applyDistanceFilter(const Sfus::FusionSlotInfo& vcuSlot,
                           Sfus::FusionSlotInfo& filteredSlot);

    /**
     * @brief 角度过滤
     */
    void applyAngleFilter(const apaSlotInfo& slot,
                        const Sfus::FusionSlotInfo& vcuSlot,
                        Sfus::FusionSlotInfo& filteredSlot);

    /**
     * @brief 车位宽度过滤
     */
    void applyWidthFilter(const Sfus::FusionSlotInfo& vcuSlot,
                        Sfus::FusionSlotInfo& filteredSlot);

    /**
     * @brief 水平车位向量距离过滤
     */
    void applyParallelVectorFilter(const apaSlotInfo& slot,
                                 const Sfus::FusionSlotInfo& vcuSlot,
                                 Sfus::FusionSlotInfo& filteredSlot);

    /**
     * @brief 执行推荐逻辑
     */
    void SlotRecommend(int final_select_ID,
                             int is_Still,
                             int currentRecommendID,
                             std::vector<Sfus::FusionSlotInfo>& vcuSlots);

    /**
     * @brief 清除推荐和选择状态
     */
    void clearRecommendAndSelectState(std::vector<Sfus::FusionSlotInfo>& vcuSlots);

private:
    Sfus::FusionSlotInfovector vcuSlotOutput_;  // VCU输出缓存
};