#include "GetInput.hpp"
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"



extern int STILL_THRESHOLD;

std::deque<DRPoseWithTime> dr_pose_buffer;
std::deque<Loc::MapInfo> map_info_buffer;
const size_t MAX_BUFFER_SIZE = 60;
const size_t MAX_RD_DR_ALLOWANCE = 200;

// 更新DR缓存池
void UpdateDRPoseBuffer(const Loc::App2emap_DR& dr_pose) {
    DRPoseWithTime dr_with_time = { dr_pose, dr_pose.timeStamp };
    dr_pose_buffer.push_back(dr_with_time);
    if (dr_pose_buffer.size() > MAX_BUFFER_SIZE) {
        dr_pose_buffer.pop_front();
    }
}

// 获取匹配的DR
// 1. 新的RD + 老的DR
// 2. 最大但小于RD的时间戳，差值不超过MAX_RD_DR_ALLOWANCE
bool GetMatchedDRPose(unsigned long long rd_timestamp, Loc::App2emap_DR& matched_pose) {
    bool found = false;
    unsigned long long closest_diff = UINT64_MAX;

    for (auto it = dr_pose_buffer.begin(); it != dr_pose_buffer.end(); ++it) {
        if (it->timestamp < rd_timestamp) {
            unsigned long long diff = rd_timestamp - it->timestamp;
            if (diff <= MAX_RD_DR_ALLOWANCE && diff < closest_diff) {
                matched_pose = it->dr_pose;
                closest_diff = diff;
                found = true;
            }
        }
    }

    return found;
}

GetInput::GetInput() {

}

GetInput::~GetInput() {

}

void GetInput::GetAPAStatus(StatusDecOutput& apastatus_info, int& apa_status) {
    S2S_MCore_Bridge_GetSigStatusDecOutput(&apastatus_info);
    LOGD("[INPUT apastatus]: %d",apastatus_info.aps_apaStatusReq);
    apa_status = apastatus_info.aps_apaStatusReq;
}

void GetInput::GetRDInfo(int& apa_status, rd::QuadParkingSlots& rd_info, unsigned long long& singleframeslotsID, std::vector<padVisionSlotCoord>& singleframeslots) {
    EMC_TROS_Bridge_Parking_GetFieldQuadParkingSlots(rd_info);

    //frameid
    LOGD("[INPUT rd_info timestampNs] J5 SEND timestampNs: %llu", rd_info.frameTimeStampNs);
    singleframeslotsID = rd_info.frameTimeStampNs;

    //singleframeslot
    if (!rd_info.quadParkingSlotList.empty()) {
        LOGD("[INPUT rd_info singleframeslots] J5 SEND RD output slots size: %d",rd_info.quadParkingSlotList.size());
        for (const auto& parkingSlot : rd_info.quadParkingSlotList) {
            LOGD("[INPUT rd_info singleframeslots] J5 SEND slottype(chuizhi0shuiping1xiexiang2): %d, filtered(0unccupied): %d, label(0qita1caozhuan2jixie): %d, confidence: %f, tl:(%f,%f), bl:(%f,%f), tr:(%f,%f), br:(%f,%f)",
            parkingSlot.slotType,
            parkingSlot.filtered,
            parkingSlot.label,
            parkingSlot.confidence,
            parkingSlot.tl.x,parkingSlot.tl.y,parkingSlot.bl.x,parkingSlot.bl.y,
            parkingSlot.tr.x,parkingSlot.tr.y,parkingSlot.br.x,parkingSlot.br.y);

            padVisionSlotCoord oneslot;
            oneslot.bayType = (parkingSlot.slotType == 0) ? 0x00 : (parkingSlot.slotType == 1) ? 0x01 : (parkingSlot.slotType == 2) ? 0x02 : 0xFF;
        
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

            if (apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7) {
                memset(&oneslot, 0, sizeof(padVisionSlotCoord));
            }
            singleframeslots.push_back(oneslot);
        }
    } else {
        LOGD("[INPUT rd_info singleframeslots] S32G RECEIVE NO SLOTS! frameTimeStampNs: %llu", rd_info.frameTimeStampNs);
    }
}

void GetInput::GetDRInfo(int& apa_status, Loc::App2emap_DR& dr_pose, Loc::App2emap_DR& previous_dr_pose, padVehiclePose& pose_globaldata, bool& is_Still, int& still_count) {
    if (apa_status != 1) {
        EMC_TROS_Bridge_Parking_GetFieldApp2emap_DR(dr_pose);

        LOGD("[INPUT dr_pose] J5 SEND x: %f, y: %f, yaw: %f, timestamp: %llu",dr_pose.x, dr_pose.y, dr_pose.canAng,dr_pose.timeStamp);
        pose_globaldata.coord.x = int(dr_pose.x);
        pose_globaldata.coord.y = int(dr_pose.y);
        pose_globaldata.yaw = dr_pose.canAng;

        // 更新缓存池
        // UpdateDRPoseBuffer(dr_pose); 
        // 打印缓存池
        int index = 0;
        for (const auto& dr_item : dr_pose_buffer) {
            LOGD("[DR BUFFER][%d] timestamp: %llu, x: %f, y: %f, yaw: %f",
                index, dr_item.timestamp, dr_item.dr_pose.x, dr_item.dr_pose.y, dr_item.dr_pose.canAng);
            ++index;
        }
    }
}

void GetInput::GetPerception(Fus::PkEmapObs& obs_info_get) {
    EMC_perception_fusion_process_GetFieldPkEmapObs(obs_info_get);

    // 打印原始的obs
    for (int i = 0; i < 50; ++i) {
        if (obs_info_get.pkEmapObs[i].FrameIndex == 0){
            continue;
        }else{
            LOGD("[INPUT obs_info] frameindex: %llu, obsid: %d, obstyp: %u, obscenter (%f,%f,%f), age: %d",
            obs_info_get.pkEmapObs[i].FrameIndex,
            obs_info_get.pkEmapObs[i].obsID,
            obs_info_get.pkEmapObs[i].obsTyp,
            obs_info_get.pkEmapObs[i].obsCenter.x,
            obs_info_get.pkEmapObs[i].obsCenter.y,
            obs_info_get.pkEmapObs[i].obsCenter.z,
            obs_info_get.pkEmapObs[i].age);
        }
    }

    // // 过滤误检的限位块（自车内）
    // if (apa_status == 5){
        
    //     Fus::PkEmapObs filtered_obs_info = obs_info_get;
    //     memset(filtered_obs_info.pkEmapObs, 0, sizeof(filtered_obs_info.pkEmapObs));
    //     int filtered_idx = 0;
    //     for (int i = 0; i < 50; ++i) {
    //         const auto& obs = obs_info_get.pkEmapObs[i];
    //         if (obs.FrameIndex == 0) continue;

    //         if (obs.obsCenter.y <= 0.5f) {
    //             filtered_obs_info.pkEmapObs[filtered_idx++] = obs;
    //         }
    //     }
    //     // 打印过滤后的obs
    //     for (int i = 0; i < filtered_idx; ++i) {
    //         const auto& obs = filtered_obs_info.pkEmapObs[i];
    //         LOGD("[INPUT obs_info FILTERED while 5] frameindex: %llu, obsid: %d, obstyp: %u, obscenter (%.3f, %.3f, %.3f), age: %d",
    //             obs.FrameIndex, obs.obsID, obs.obsTyp,
    //             obs.obsCenter.x, obs.obsCenter.y, obs.obsCenter.z,
    //             obs.age);
    //     }
    //     // 覆盖原始数据
    //     obs_info_get = filtered_obs_info;
    // }
}



void GetInput::GetSearchParkStatus(StatusDecFusionOutput& searchpark_info, int& park_request , int& search_interrupt) {
    S2S_MCore_Bridge_GetSigStatusDecFusionOutput(&searchpark_info);

    LOGD("[INPUT searchpark_status] parking_request: %d. search_interrupt: %d",searchpark_info.aps_apaStartParkingReq,searchpark_info.aps_apaSrchInterupt);
    park_request = searchpark_info.aps_apaStartParkingReq;
    search_interrupt = searchpark_info.aps_apaSrchInterupt;
}

void GetInput::GetAllInput() {
    GetAPAStatus(apastatus_info, apa_status);
    GetRDInfo(apa_status, rd_info, singleframeslotsID, singleframeslots);
    GetDRInfo(apa_status, dr_pose, previous_dr_pose, pose_globaldata, is_Still, still_count);
    GetPerception(obs_info_get);
    GetSearchParkStatus(searchpark_info, park_request, search_interrupt);
    ClearExistedInput(singleframeslotsID, singleframeslots, pose_globaldata, apa_status);
}

void GetInput::ClearAllInput() {
    rd_info = rd::QuadParkingSlots{};
    dr_pose = Loc::App2emap_DR{};       
    pose_globaldata = padVehiclePose{};  
    obs_info_get = Fus::PkEmapObs{};
}



void GetInput::ClearExistedInput(unsigned long long& singleframeslotsID, std::vector<padVisionSlotCoord>& singleframeslots, padVehiclePose& pose_globaldata, int& apa_status) {
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

