#include "OnInput.hpp"
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"

extern bool DEBUG;
extern SaveFileToJson filetojson;
extern int STILL_THRESHOLD;

// 构造函数
OnInput::OnInput() : 
    is_Still(true), 
    still_count(0), 
    apa_status(0)
{

}

// 析构函数
OnInput::~OnInput() {

}

void OnInput::OnRDInfo(int& apa_status, rd::QuadParkingSlots& rd_info, unsigned long long& singleframeslotsID, std::vector<padVisionSlotCoord>& singleframeslots) {

    //frameid
    LOGD("[INPUT rd_info timestampNs] J5 SEND timestampNs: %llu", rd_info.frameTimeStampNs);
    singleframeslotsID = rd_info.frameTimeStampNs;

    //singleframeslot
    if (!rd_info.quadParkingSlotList.empty()) {
        LOGD("[INPUT rd_info singleframeslots] J5 SEND RD output slots size: %d",rd_info.quadParkingSlotList.size());
        for (const auto& parkingSlot : rd_info.quadParkingSlotList) {
            LOGD("[INPUT rd_info singleframeslots] J5 SEND slottype(chuizhi0shuiping1xiexiang2): %d, filtered(0unccupied): %d, label(0qita1caozhuan2jixie): %d, tl:(%f,%f), bl:(%f,%f), tr:(%f,%f), br:(%f,%f)",
            parkingSlot.slotType,
            parkingSlot.filtered,
            parkingSlot.label,
            parkingSlot.tl.x,parkingSlot.tl.y,parkingSlot.bl.x,parkingSlot.bl.y,
            parkingSlot.tr.x,parkingSlot.tr.y,parkingSlot.br.x,parkingSlot.br.y);

            padVisionSlotCoord oneslot;
            oneslot.bayType = (parkingSlot.slotType == 0) ? 0x00 : (parkingSlot.slotType == 1) ? 0x01 : (parkingSlot.slotType == 2) ? 0x02 : 0xFF;
            
            //左右判断,按规划/定位ABCD顺序输出车位角点
            if (parkingSlot.tl.x < 448 && parkingSlot.tr.x < 448) {
                oneslot.slotSide = 0x01; //x小于图像中心，判断为左
                oneslot.a.x = int(parkingSlot.tr.x);
                oneslot.a.y = int(parkingSlot.tr.y);
                oneslot.b.x = int(parkingSlot.tl.x);
                oneslot.b.y = int(parkingSlot.tl.y);
                oneslot.c.x = int(parkingSlot.bl.x);
                oneslot.c.y = int(parkingSlot.bl.y);
                oneslot.d.x = int(parkingSlot.br.x);
                oneslot.d.y = int(parkingSlot.br.y);
                oneslot.occupy = parkingSlot.filtered;
                oneslot.material = parkingSlot.label;
                // LOGD("[INPUT rd_info singleframeslots] S32G RECEIVE LEFT SLOTS tl:(%d,%d), tr:(%d,%d), br:(%d,%d), bl:(%d,%d)",oneslot.b.x,oneslot.b.y,oneslot.a.x,oneslot.a.y,
            // oneslot.d.x,oneslot.d.y,oneslot.c.x,oneslot.c.y);
            } else {
                oneslot.slotSide = 0x00;
                oneslot.a.x = int(parkingSlot.tl.x);
                oneslot.a.y = int(parkingSlot.tl.y);
                oneslot.b.x = int(parkingSlot.tr.x);
                oneslot.b.y = int(parkingSlot.tr.y);
                oneslot.c.x = int(parkingSlot.br.x);
                oneslot.c.y = int(parkingSlot.br.y);
                oneslot.d.x = int(parkingSlot.bl.x);
                oneslot.d.y = int(parkingSlot.bl.y);
                oneslot.occupy = parkingSlot.filtered;
                oneslot.material = parkingSlot.label;
                // LOGD("[INPUT rd_info singleframeslots] S32G RECEIVE RIGHT SLOTS tl:(%d,%d), tr:(%d,%d), br:(%d,%d), bl:(%d,%d)",oneslot.a.x,oneslot.a.y,oneslot.b.x,oneslot.b.y,
            // oneslot.c.x,oneslot.c.y,oneslot.d.x,oneslot.d.y);
            }
            if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7) {
                memset(&oneslot, 0, sizeof(padVisionSlotCoord));
            }
            singleframeslots.push_back(oneslot);
        }
    } else {
        LOGD("[INPUT rd_info singleframeslots] S32G RECEIVE NO SLOTS! frameTimeStampNs: %llu", rd_info.frameTimeStampNs);
    }
}

void OnInput::OnDRInfo(int& apa_status, Loc::App2emap_DR& dr_pose, Loc::App2emap_DR& previous_dr_pose, padVehiclePose& pose_globaldata, bool& is_Still, int& still_count) {

    LOGD("[INPUT dr_pose] J5 SEND x: %f, y: %f, yaw: %f, timestamp: %llu",dr_pose.x, dr_pose.y, dr_pose.canAng,dr_pose.timeStamp);
    pose_globaldata.coord.x = int(dr_pose.x);
    pose_globaldata.coord.y = int(dr_pose.y);
    pose_globaldata.yaw = dr_pose.canAng;

    if (dr_pose.x != previous_dr_pose.x || dr_pose.y != previous_dr_pose.y || dr_pose.canAng != previous_dr_pose.canAng) {
        is_Still = false; // 有变化，设置为运动中
        still_count = 0; // reset
    } else {
        still_count++;
    }
    if (still_count >= STILL_THRESHOLD) {
        is_Still = true;
    }
    previous_dr_pose = dr_pose;
    LOGD("is_Still: %d",is_Still);
}

void OnInput::OnPerception(Fus::PkEmapObs& obs_info_on) {

    for (int i = 0; i < 50; ++i) {
        LOGD("[INPUT obs_info] frameindex: %llu, obsid: %d, obstyp: %u, obscenter (%f,%f,%f), age: %d",
            obs_info_on.pkEmapObs[i].FrameIndex,
            obs_info_on.pkEmapObs[i].obsID,
            obs_info_on.pkEmapObs[i].obsTyp,
            obs_info_on.pkEmapObs[i].obsCenter.x,
            obs_info_on.pkEmapObs[i].obsCenter.y,
            obs_info_on.pkEmapObs[i].obsCenter.z,
            obs_info_on.pkEmapObs[i].age);
    }
}

void OnInput::OnAPAStatus(StatusDecOutput& apastatus_info, int& apa_status) {

    LOGD("[INPUT apastatus]: %d",apastatus_info.aps_apaStatusReq);
    apa_status = apastatus_info.aps_apaStatusReq;
}

void OnInput::OnSearchParkStatus(StatusDecFusionOutput& searchpark_info, int& park_request , int& search_interrupt) {

    LOGD("[INPUT searchpark_status] parking_request: %d. search_interrupt: %d",searchpark_info.aps_apaStartParkingReq,searchpark_info.aps_apaSrchInterupt);
    park_request = searchpark_info.aps_apaStartParkingReq;
    search_interrupt = searchpark_info.aps_apaSrchInterupt;
}


void OnInput::ClearExistedInput(unsigned long long& singleframeslotsID, std::vector<padVisionSlotCoord>& singleframeslots, padVehiclePose& pose_globaldata, int& apa_status) {
    if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7) {
        singleframeslotsID = 0;
        singleframeslots.clear();
        pose_globaldata.coord.x = 0;
        pose_globaldata.coord.y = 0;
        pose_globaldata.yaw = 0;
        LOGD("CLEAR singleframeslots, size: %d",singleframeslots.size());
    }
}

//@TODO GET USS

// void GetInput::GetAllInput() {
//     GetAPAStatus(apastatus_info, apa_status);
//     GetRDInfo(apa_status, rd_info, singleframeslotsID, singleframeslots);
//     GetDRInfo(apa_status, dr_pose, previous_dr_pose, pose_globaldata, is_Still, still_count);
//     GetPerception(obs_info_get);
//     GetSearchParkStatus(searchpark_info, park_request, search_interrupt);
//     ClearExistedInput(singleframeslotsID, singleframeslots, pose_globaldata, apa_status);
// }
