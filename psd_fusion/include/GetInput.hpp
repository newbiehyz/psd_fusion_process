#ifndef PARKINGSYSTEM_HPP
#define PARKINGSYSTEM_HPP

#include <vector>
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"
#include <deque>


struct DRPoseWithTime {
    Loc::App2emap_DR dr_pose;
    unsigned long long timestamp;
};

class GetInput {
public:
    GetInput();

    ~GetInput();

    void GetRDInfo(int& apa_status, rd::QuadParkingSlots& rd_info, unsigned long long& singleframeslotsID, std::vector<padVisionSlotCoord>& singleframeslots);
    
    void GetDRInfo(int& apa_status, Loc::App2emap_DR& dr_pose, Loc::App2emap_DR& previous_dr_pose, padVehiclePose& pose_globaldata, bool& is_Still, int& still_count);
    void UpdateDRPoseBuffer(const Loc::App2emap_DR& dr_pose);
    bool GetMatchedDRPose(unsigned long long rd_timestamp, Loc::App2emap_DR& matched_pose);

    void GetPerception(Fus::PkEmapObs& obs_info_get);
    void GetAPAStatus(StatusDecOutput& apastatus_info, int& apa_status);
    void GetSearchParkStatus(StatusDecFusionOutput& searchpark_info, int& park_request, int& search_interrupt);

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
    StatusDecOutput apastatus_info;
    int apa_status;
    StatusDecFusionOutput searchpark_info;
    int park_request;
    int search_interrupt;

    std::deque<DRPoseWithTime> dr_pose_buffer;
    const size_t MAX_BUFFER_SIZE = 20;
    const size_t MAX_RD_DR_ALLOWANCE = 100;


private:


};

#endif // PARKINGSYSTEM_HPP
