#ifndef PSD_FUSION_LOGIC_H
#define PSD_FUSION_LOGIC_H

#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "fusion.h"
#include "psd_fusion_process_base.h" // 如果这些函数需要访问基类数据或方法
#include "PSD_FusionModuleIF.h"
#include "save_to_json.h" // 根据实际需要包含其他头文件
#include "utils.h"
#include "psd_selection_logic.h"

// 声明外部变量
extern int apa_status;
extern int park_request;
extern const int search_interrupt;
extern int is_Still;
extern Loc::App2emap_DR previous_dr_pose;
extern std::vector<apaSlotInfo> g_singleframe_locked_slots;

extern slotfusion fusionslot;
extern StatusDecFusionInput psd2statemachine;
extern Sfus::Sfsuion2DecPlan psd2planning;
extern Sfus::FusionSlotInfovector psd2vcu;
extern Fsm::FusionSlotInfo2Location psd2location;
extern APAControlBumpInput psd2control;

extern int HMI_select_ID;
extern int HMI_temp_ID;
extern int VCU_select_ID_ON;
extern int RECOMMEND_ID;
extern int final_select_ID;
extern int final_ID;
extern bool recommend_exist;
extern bool already_has_recommend_slot;
extern bool in_release_range_last_frame;
extern int stable_frame_count;
extern const int STABLE_THRESHOLD;
extern int available_slot_flag_to_statemachine;
extern bool isNarrow;
extern int parkout_flag;
extern int mirror_fold_flag_ahead;
extern int mirror_fold_flag;
extern bool target_slot_already_updated_once;
extern bool target_slot_in_range;
extern Sfus::SfusionSlotType slot_type_before_update;
extern POINT_I search_target_center;
extern POINT_I search_target_center_world;
extern POINT_I new_target_center;
extern POINT_I new_target_center_world;
extern const float MAX_SLOT_MOVE_DIST_MM;
extern POINT_I world_slot_memory[4];
extern std::vector<std::vector<int>> ipm_camera_id_image;

void load_image_from_csv(const std::string& filename, std::vector<std::vector<int>>& image);

void ProcessFarawayFilter(int index, Sfus::FusionSlotInfovector& vcu_data, const apaSlotInfo& slot_output);
void ProcessAngleFilter(int index, Sfus::FusionSlotInfovector& vcu_data, const apaSlotInfo& slot_output);
void ProcessWidthFilter(int index, Sfus::FusionSlotInfovector& vcu_data, const apaSlotInfo& slot_output);
void ProcessParallelFilter(int index, Sfus::FusionSlotInfovector& vcu_data, const apaSlotInfo& slot_output);
void SetObstacleProperties(int index, Sfus::FusionSlotInfovector& vcu_data, const apaSlotInfo& slot_output);
void ProcessRecommendationLogic(apaSlotInfo& selected_slot_in_world, Sfus::FusionSlotInfovector& psd2vcu, int slotlist_size, bool is_Still, apaSlotListInfo& outputSlot_FUSED, PSD_FusionModuleIF& psd_fusion_module_if);
void ProcessNonSearchVCUDisplay(uint64_t current1970_ms, Sfus::FusionSlotInfovector& psd2vcu, int slotlist_size, int apa_status, apaSlotListInfo& outputSlot_FUSED, PSD_FusionModuleIF& psd_fusion_module_if);

void ProcessPSD2VCU(int apa_status, uint64_t current1970_ms, apaSlotInfo& selected_slot_in_world, Sfus::FusionSlotInfovector& psd2vcu, int slotlist_size, bool is_Still, apaSlotListInfo& outputSlot_FUSED, PSD_FusionModuleIF& psd_fusion_module_if);
void ProcessPSD2APAHandle(uint64_t current1970_ms, Fsm::FusionSlotInfo2Location& psd2location, int slotlist_size, apaSlotListInfo& outputSlot_FUSED, int mirror_fold_flag, int parkout_flag);
void ProcessPSD2APAHandleTargetID(int final_ID, int parkout_flag);
void ProcessPSD2Planning(uint64_t current1970_ms, Sfus::Sfsuion2DecPlan& psd2planning, int slotlist_size, apaSlotListInfo& outputSlot_FUSED);
void ProcessPSD2PlanningTargetSlot(uint64_t current1970_ms, int& target_slot_fusionSlotType, int final_ID, int apa_status, int parkout_flag, Sfus::Sfsuion2DecPlan& psd2planning, apaSlotListInfo& outputSlot_FUSED, const padVehiclePose& pose_globaldata, std::vector<std::vector<int>>& ipm_camera_id_image);
void ProcessGuidanceTargetSlotUpdate(int& target_slot_fusionSlotType, int final_ID, apaSlotListInfo& outputSlot_FUSED, Sfus::Sfsuion2DecPlan& psd2planning, const padVehiclePose& pose_globaldata, std::vector<std::vector<int>>& ipm_camera_id_image);
void ProcessPSD2Perception(uint64_t current1970_ms, int target_slot_fusionSlotType, int parkout_flag, Sfus::Sfsuion2DecPlan& psd2planning, int mirror_fold_flag);
void ProcessPSD2StateMachine(int final_ID, int slotlist_size, int apa_status, StatusDecFusionInput& psd2statemachine, Sfus::Sfsuion2DecPlan& psd2planning, int available_slot_flag_to_statemachine);
void ProcessPSD2USS(int final_ID);
void ProcessPSD2Control(int final_ID, int parkout_flag, apaSlotListInfo& outputSlot_FUSED, APAControlBumpInput& psd2control);

#endif // PSD_FUSION_LOGIC_H