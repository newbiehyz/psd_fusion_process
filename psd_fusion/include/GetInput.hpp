#ifndef PARKINGSYSTEM_HPP
#define PARKINGSYSTEM_HPP

#include <vector>
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"

class GetInput {
public:
    GetInput();

    ~GetInput();

    void GetRDInfo(rd::QuadParkingSlots& rd_info, unsigned long long& singleframeslotsID, std::vector<padVisionSlotCoord>& singleframeslots);
    void GetDRInfo(int& apa_status, Loc::App2emap_DR& dr_pose, Loc::App2emap_DR& previous_dr_pose, padVehiclePose& pose_globaldata, bool& is_Still, int& still_count);
    void GetPerception(Fus::PkEmapObs& obs_info_get);
    void GetStateMachine(StatusDecOutput& statemachine_info, int& apa_status);
    void ClearExistedInput(unsigned long long& singleframeslotsID, std::vector<padVisionSlotCoord>& singleframeslots, padVehiclePose& pose_globaldata, int& apa_status);


    void GetAllInput();

    rd::QuadParkingSlots rd_info;
    unsigned long long singleframeslotsID;
    std::vector<padVisionSlotCoord> singleframeslots;
    Loc::App2emap_DR dr_pose;
    padVehiclePose pose_globaldata;
    Loc::App2emap_DR previous_dr_pose;
    bool is_Still;
    int still_count;
    Fus::PkEmapObs obs_info_get;
    StatusDecOutput statemachine_info;
    int apa_status;

private:
};

#endif // PARKINGSYSTEM_HPP
