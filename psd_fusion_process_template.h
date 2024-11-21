#ifndef EMC_PSD_FUSION_PROCESS_TEMPLATE_INCLUDE
#define EMC_PSD_FUSION_PROCESS_TEMPLATE_INCLUDE
#include "psd_fusion_process_base.h"
#include "apa_define.h"
#include "fusion.h"
#include "save_to_json.h"


void mergeSlotLists(const apaSlotListInfo &outputSlot_USS,const apaSlotListInfo &outputSlot_VIS,apaSlotListInfo &outputSlot_FUSION); 
int slottype_uss2rd(UssIf_enmSlotType_t uss_type);
int slottype_rd2vcu(int rd_type);
Sfus::_tSfusionSlotType slottype_rd2decplan(int rd_type);

class cpsd_fusion_process: public cpsd_fusion_process_base
{
public:
    cpsd_fusion_process();
    ~cpsd_fusion_process();

public:
    tResult Init();
    tResult Term();
    tResult Start();
    tResult Stop();

public:	
    tResult ThreadTrigger_thread();
    tResult TimeTrigger_Timer50();
    tResult TimeTrigger_Timer100();
    tResult OnVehicleCanData(const VehicleCanData& userData);
    tResult OnStatusDecOutput(const StatusDecOutput& userData);
    tResult OnUssIf_stPLVOutputInfo(const UssIf_stPLVOutputInfo_t& userData);
    tResult OnAPAControlPlanOutput(const APAControlPlanOutput& userData);
    tResult OnStatusDecFusionOutput(const StatusDecFusionOutput& userData);
    tResult OnAPAControlDebugOutput(const APAControlDebugOutput& userData);
    tResult OnStatusDec2FusionDebug(const StatusDec2FusionDebug& userData);
    tResult OnEmapWorkMode(const Fus::EmapWorkMode& userData);
    tResult OnFusionTimeStamp(const Fus::FusionTimeStamp& userData);
    tResult OnVagueEmapGrid(const Fus::VagueEmapGrid& userData);
    tResult OnPreciseEmapGrid(const Fus::PreciseEmapGrid& userData);
    tResult OnPkEmapObs(const Fus::PkEmapObs& userData);
    tResult OnEmapSlotVector(const Fus::EmapSlotVector& userData);
    tResult OnStableEmapObs(const Fus::StableEmapObs& userData);
    tResult OnFusionSlotInfo(const Fus::FusionSlotInfo& userData);
    tResult OnApp2emap_DR(const Loc::App2emap_DR& userData);
    tResult OnMapInfo(const Loc::MapInfo& userData);
    tResult OnObstacles(const od::Obstacles& userData);
    tResult OnSApaPSInfo(const rd::SApaPSInfo& userData);
    tResult OnQuadParkingSlots(const rd::QuadParkingSlots& userData);
    tResult OnImage(const rd::Image& userData);
    tResult OnDecPlan2Emap(const Pla::DecPlan2Emap& userData);
    tResult OnHMI_InputInfo(const HMI_InputInfo& userData);
    tResult OnSelectSlot(const Sfus::SelectSlot& userData);
    tResult OnParkInHeadInSwitch(const Sfus::ParkInHeadInSwitch& userData);
    tResult OnSelectSlot2(const Sfus::SelectSlot& userData);

 public:
    apaSlotListInfo outputSlot_FUSED;
    apaSlotListInfo outputSlot_VIS;   
};

#endif
