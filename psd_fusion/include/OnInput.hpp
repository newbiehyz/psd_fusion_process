#ifndef PARKINGSYSTEM_HPP
#define PARKINGSYSTEM_HPP

#include <vector>
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"

class OnInput {
public:
    OnInput();

    ~OnInput();

    void OnRDInfo(int& apa_status, rd::QuadParkingSlots& rd_info, unsigned long long& singleframeslotsID, std::vector<padVisionSlotCoord>& singleframeslots);
    void OnDRInfo(int& apa_status, Loc::App2emap_DR& dr_pose, Loc::App2emap_DR& previous_dr_pose, padVehiclePose& pose_globaldata, bool& is_Still, int& still_count);
    void OnPerception(Fus::PkEmapObs& obs_info_get);
    void OnAPAStatus(StatusDecOutput& apastatus_info, int& apa_status);
    void OnSearchParkStatus(StatusDecFusionOutput& searchpark_info, int& park_request, int& search_interrupt);
    void ClearExistedInput(unsigned long long& singleframeslotsID, std::vector<padVisionSlotCoord>& singleframeslots, padVehiclePose& pose_globaldata, int& apa_status);

    // void GetAllInput();




    rd::QuadParkingSlots rd_info;
    unsigned long long singleframeslotsID;
    std::vector<padVisionSlotCoord> singleframeslots;
    Loc::App2emap_DR dr_pose;
    padVehiclePose pose_globaldata;
    Loc::App2emap_DR previous_dr_pose;
    bool is_Still;
    int still_count;
    Fus::PkEmapObs obs_info_get;
    StatusDecOutput apastatus_info;
    int apa_status;
    StatusDecFusionOutput searchpark_info;
    int park_request;
    int search_interrupt;


private:


};

#endif // PARKINGSYSTEM_HPP
