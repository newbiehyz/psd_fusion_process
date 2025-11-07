#ifndef EMC_PSD_FUSION_PROCESS_TEMPLATE_INCLUDE
#define EMC_PSD_FUSION_PROCESS_TEMPLATE_INCLUDE

#include <thread>
#include "psd_fusion_process_base.h"
#include "apa_define.h"
#include "fusion.h"
#include "save_to_json.h"
#include "PSD_FusionModuleIF.h"

void mergeSlotLists(const apaSlotListInfo &outputSlot_USS,const apaSlotListInfo &outputSlot_VIS,apaSlotListInfo &outputSlot_FUSION); 
int slottype_uss2rd(UssIf_enmSlotType_t uss_type);
int slottype_rd2vcu(int rd_type);
Sfus::_tSfusionSlotType slottype_rd2decplan(int rd_type);
int slottype_decplan2statemachine(Sfus::_tSfusionSlotType decplan_type);
Sfus::SlotBottomType slotbottomtype_uss2decplan(int uss_bottom_type);


class cpsd_fusion_process: public cpsd_fusion_process_base
{
public:
    cpsd_fusion_process();
    virtual ~cpsd_fusion_process() override;

public:
    virtual tResult Init() override;
    virtual tResult Term() override;
    virtual tResult Start() override;
    virtual tResult Stop() override;
public:	
    virtual tResult TimeTrigger_thread_50ms_1() override;
    virtual tResult TimeTrigger_thread_50ms_2() override;
    virtual tResult OnVehicleCanData(const VehicleCanData& userData) override;
    virtual tResult OnStatusDecOutput(const StatusDecOutput& userData) override;
    virtual tResult OnUssIf_stPLVOutputInfo(const UssIf_stPLVOutputInfo_t& userData) override;
    virtual tResult OnAPAControlPlanOutput(const APAControlPlanOutput& userData) override;
    virtual tResult OnStatusDecFusionOutput(const StatusDecFusionOutput& userData) override;
    virtual tResult OnAPAControlDebugOutput(const APAControlDebugOutput& userData) override;
    virtual tResult OnStatusDec2FusionDebug(const StatusDec2FusionDebug& userData) override;
    virtual tResult OnStateMachine_Output(const StateMachine_Output& userData) override;
    virtual tResult OnApp2emap_DR(const Loc::App2emap_DR& userData) override;
    virtual tResult OnMapInfo(const Loc::MapInfo& userData) override;
    virtual tResult OnObstacles(const od::Obstacles& userData) override;
    virtual tResult OnSApaPSInfo(const rd::SApaPSInfo& userData) override;
    virtual tResult OnQuadParkingSlots(const rd::QuadParkingSlots& userData) override;
    virtual tResult OnImage(const rd::Image& userData) override;
    virtual tResult OnHMI_InputInfo(const HMI_InputInfo& userData) override;
    virtual tResult OnSelectSlot(const Sfus::SelectSlot& userData) override;
    virtual tResult OnParkInHeadInSwitch(const Sfus::ParkInHeadInSwitch& userData) override;
    virtual tResult OnSelectSlot2(const Sfus::SelectSlot& userData) override;
    virtual tResult OnEmapWorkMode(const Fus::EmapWorkMode& userData) override;
    virtual tResult OnFusionTimeStamp(const Fus::FusionTimeStamp& userData) override;
    virtual tResult OnPkEmapObs(const Fus::PkEmapObs& userData) override;
    virtual tResult OnEmapSlotVector(const Fus::EmapSlotVector& userData) override;
    virtual tResult OnStableEmapObs(const Fus::StableEmapObs& userData) override;
    virtual tResult OnFusionSlotInfo(const Fus::FusionSlotInfo& userData) override;
    virtual tResult OnParkInHeadInSwitch2(const Sfus::ParkInHeadInSwitch& userData) override;
    virtual tResult OnSelectSlot3(const Sfus::SelectSlot& userData) override;
    virtual tResult OnHMI_InputInfo2(const HMI_InputInfo& userData) override;
    virtual tResult OnDecPlan2Emap(const Pla::DecPlan2Emap& userData) override;
    virtual tResult OnPlan2Psd(const Pla::Plan2Psd& userData) override;
    virtual tResult TimeTrigger_thread_100ms_1() override;
    
private:
    bool LoadFromFile(const std::string& filename);
    void Slot2Global(Sfus::Sfsuion2DecPlan &slot, const float &x, const float &y, const float &yaw);
    void Slot2Local(Sfus::Sfsuion2DecPlan &slot, const float &x, const float &y, const float &yaw);
    int HMIVCUSelect(int &hmi_temp, const int &hmi_select, const int &vcu_select);
    int RecommendSelectID(const int &final_select, const int &recommend);
    int IsParkOut(int apastatus);
    int IsStill(const Loc::App2emap_DR dr_pose, Loc::App2emap_DR& previous_dr_pose);
    void adjustPSD2PLANNINGRectOrder(Sfus::SlotCorners &slotCorners);

public:
    apaSlotListInfo outputSlot_FUSED;
    apaSlotListInfo outputSlot_VIS;   
    apaSlotListInfo outputSlot_USS;
    bool DEBUG;
    PSD_FusionModuleIF PSD_FusionModuleIFrunable;
    int slotlist_size;
    bool dr_first = true;
    int dr_cul_x = 0;
    int dr_cul_y = 0;
    float dr_cul_theta = 0.0;
    std::vector<std::vector<int>> ipm_camera_id_image;
	

    std::mutex _dr_mutex;
    std::mutex _map_mutex;
    
};

#endif
