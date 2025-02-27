#ifndef EMC_PSD_FUSION_PROCESS_TEMPLATE_INCLUDE
#define EMC_PSD_FUSION_PROCESS_TEMPLATE_INCLUDE
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

class cpsd_fusion_process: public cpsd_fusion_process_base
{
public:
    cpsd_fusion_process();
    ~cpsd_fusion_process() override;

public:
    tResult Init() override;
    tResult Term() override;
    tResult Start() override;
    tResult Stop() override;
public:	
    tResult TimeTrigger_thread_50ms_1() override;
    tResult TimeTrigger_thread_50ms_2() override;
    tResult OnVehicleCanData(const VehicleCanData& userData) override;
    tResult OnStatusDecOutput(const StatusDecOutput& userData) override;
    tResult OnUssIf_stPLVOutputInfo(const UssIf_stPLVOutputInfo_t& userData) override;
    tResult OnAPAControlPlanOutput(const APAControlPlanOutput& userData) override;
    tResult OnStatusDecFusionOutput(const StatusDecFusionOutput& userData) override;
    tResult OnAPAControlDebugOutput(const APAControlDebugOutput& userData) override;
    tResult OnStatusDec2FusionDebug(const StatusDec2FusionDebug& userData) override;
    tResult OnApp2emap_DR(const Loc::App2emap_DR& userData) override;
    tResult OnMapInfo(const Loc::MapInfo& userData) override;
    tResult OnObstacles(const od::Obstacles& userData) override;
    tResult OnSApaPSInfo(const rd::SApaPSInfo& userData) override;
    tResult OnQuadParkingSlots(const rd::QuadParkingSlots& userData) override;
    tResult OnImage(const rd::Image& userData) override;
    tResult OnDecPlan2Emap(const Pla::DecPlan2Emap& userData) override;
    tResult OnHMI_InputInfo(const HMI_InputInfo& userData) override;
    tResult OnSelectSlot(const Sfus::SelectSlot& userData) override;
    tResult OnParkInHeadInSwitch(const Sfus::ParkInHeadInSwitch& userData) override;
    tResult OnSelectSlot2(const Sfus::SelectSlot& userData) override;
    tResult OnEmapWorkMode(const Fus::EmapWorkMode& userData) override;
    tResult OnFusionTimeStamp(const Fus::FusionTimeStamp& userData) override;
    tResult OnVagueEmapGrid(const Fus::VagueEmapGrid& userData) override;
    tResult OnPreciseEmapGrid(const Fus::PreciseEmapGrid& userData) override;
    tResult OnPkEmapObs(const Fus::PkEmapObs& userData) override;
    tResult OnEmapSlotVector(const Fus::EmapSlotVector& userData) override;
    tResult OnStableEmapObs(const Fus::StableEmapObs& userData) override;
    tResult OnFusionSlotInfo(const Fus::FusionSlotInfo& userData) override;
    tResult OnParkInHeadInSwitch2(const Sfus::ParkInHeadInSwitch& userData) override;
    tResult OnSelectSlot3(const Sfus::SelectSlot& userData) override;
    tResult OnHMI_InputInfo2(const HMI_InputInfo& userData) override;

private:
    bool LoadFromFile(const std::string& filename);
    void Slot2Global(Sfus::Sfsuion2DecPlan &slot, const float &x, const float &y, const float &yaw);
    void Slot2Local(Sfus::Sfsuion2DecPlan &slot, const float &x, const float &y, const float &yaw);
    int HMIVCUSelect(int &hmi_temp, const int &hmi_select, const int &vcu_select);
    int RecommendSelectID(const int &final_select, const int &recommend);
    int IsParkOut(int apastatus);
    int IsStill(const Loc::App2emap_DR dr_pose, Loc::App2emap_DR& previous_dr_pose);


public:
    apaSlotListInfo outputSlot_FUSED;
    apaSlotListInfo outputSlot_VIS;   
    apaSlotListInfo outputSlot_USS;
    bool DEBUG = false;
    PSD_FusionModuleIF PSD_FusionModuleIFrunable;
    int slotlist_size;
    bool dr_first = true;
    int dr_cul_x = 0;
    int dr_cul_y = 0;
    float dr_cul_theta = 0.0;
     
};

#endif
