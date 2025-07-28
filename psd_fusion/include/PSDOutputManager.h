#pragma once

#include "psd_fusion_process_header.h"
#include "apa_define.h"
#include "PSDConfigGlobalManager.h"

POINT_I Local2Global(const POINT_I& pt_local, const float& x, const float& y, const float& yaw);
float CalcDistance(const POINT_I& a, const POINT_I& b);

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

    /**
     * @brief 发送车位列表和目标车位ID给APAHandle模块
     * @param outputSlot_FUSED 融合车位信息
     * @param currentState 全局状态
     * @param parkout_flag 泊出标志
     */
    void sendAPAHandleSlotInfo(const apaSlotListInfo& outputSlot_FUSED,
                              const PSDGlobalState& currentState,
                              int parkout_flag);

    /**
     * @brief 发送目标车位信息给Planning和Perception模块
     * @param outputSlot_FUSED 融合车位信息
     * @param currentState 全局状态（可修改）
     * @param pose_globaldata 车辆位姿信息
     * @param apa_status APA状态
     * @param parkout_flag 泊出标志
     * @param current1970_ms 当前时间戳
     * @param ipm_camera_id_image IPM相机ID图像
     * @param psd2planning Planning输出结构（输入输出参数）
     */
    void sendPlanningAndPerceptionTargetSlot(const apaSlotListInfo& outputSlot_FUSED,
                                            PSDGlobalState& currentState,
                                            const padVehiclePose& pose_globaldata,
                                            int apa_status,
                                            int parkout_flag,
                                            unsigned long long current1970_ms,
                                            const std::vector<std::vector<int>>& ipm_camera_id_image,
                                            Sfus::Sfsuion2DecPlan& psd2planning);


    /**
     * @brief 发送状态信息给StateMachine模块
     * @param slotlist_size 车位列表大小
     * @param apa_status APA状态
     * @param currentState 全局状态（可修改）
     * @param target_slot_type 目标车位类型
     */
    void sendStateMachineInfo(int slotlist_size,
                            int apa_status,
                            PSDGlobalState& currentState,
                            Sfus::_tSfusionSlotType target_slot_type);

    /**
     * @brief 发送目标车位ID给USS模块
     * @param final_ID 目标车位ID
     */
    void sendUSSTargetSlotID(int final_ID);

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

    // Planning和Perception相关私有方法
    /**
     * @brief 计算车位角度并确定车位类型
     * @param slot 车位信息
     * @param original_slot_type 原始车位类型
     * @return 计算后的车位类型
     */
    Sfus::_tSfusionSlotType calculateSlotTypeFromCorners(const apaSlotInfo& slot,
                                                       int original_slot_type);

    /**
     * @brief 判断是否为狭窄车位
     * @param cornerA 角点A
     * @param cornerB 角点B
     * @return 是否为狭窄车位
     */
    bool isNarrowSlot(const POINT_I& cornerA, const POINT_I& cornerB);

    /**
     * @brief 确定车位来源类型
     * @param slot_label 车位标签
     * @return 车位来源类型
     */
    Sfus::SfusionSlotSource determineSlotSource(int slot_label);

    /**
     * @brief 确定融合车位类型
     * @param slot 车位信息
     * @return 融合车位类型
     */
    int determineFusionSlotType(const apaSlotInfo& slot);

    /**
     * @brief 检查目标车位是否在相机范围内
     * @param slot 车位信息
     * @param imp_camera_id_image IPM相机ID图像
     * @return 是否在范围内
     */
    bool isTargetSlotInCameraRange(const apaSlotInfo& slot,
                                  const std::vector<std::vector<int>>& imp_camera_id_image);

    /**
     * @brief 处理SEARCH阶段的目标车位更新
     */
    void processSearchPhaseTargetSlot(const apaSlotListInfo& outputSlot_FUSED,
                                     PSDGlobalState& currentState,
                                     TargetSlotMemory& targetMemory,
                                     const padVehiclePose& pose_globaldata,
                                     Sfus::Sfsuion2DecPlan& psd2planning,
                                     int& target_slot_fusionSlotType);

    /**
     * @brief 处理GUIDANCE阶段的目标车位更新
     */
    void processGuidancePhaseTargetSlot(const apaSlotListInfo& outputSlot_FUSED,
                                       PSDGlobalState& currentState,
                                       TargetSlotMemory& targetMemory,
                                       const padVehiclePose& pose_globaldata,
                                       Sfus::Sfsuion2DecPlan& psd2planning,
                                       int& target_slot_fusionSlotType,
                                       const std::vector<std::vector<int>>& imp_camera_id_image);

    /**
     * @brief 发送目标车位信息给Planning模块
     */
    void sendPlanningTargetSlot(Sfus::Sfsuion2DecPlan& psd2planning,
                               int apa_status,
                               int parkout_flag,
                               const PSDGlobalState& currentState,
                               const padVehiclePose& pose_globaldata,
                               int target_slot_fusionSlotType);

    /**
     * @brief 发送目标车位信息给Perception模块
     */
    void sendPerceptionTargetSlot(const Sfus::Sfsuion2DecPlan& psd2planning,
                                 int apa_status,
                                 int parkout_flag,
                                 const PSDGlobalState& currentState,
                                 unsigned long long current1970_ms,
                                 int target_slot_fusionSlotType);

    


private:
    Sfus::FusionSlotInfovector vcuSlotOutput_;  // VCU输出缓存
    StatusDecFusionInput stateMachineOutput_;
};



