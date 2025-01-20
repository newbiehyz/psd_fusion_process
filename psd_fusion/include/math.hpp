#pragma once

#include "apa_define.h"

#define INVALID_VALUE 99999999
#define VEHICLE_LENGTH 5259.9
#define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7
namespace math{
    int CalcDistance(POINT_I a, POINT_I b)
    {
        float dis = sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
        return dis;
    }


    int CalPointAndLineDistance(const POINT_I& point, const POINT_I& pta, const POINT_I& ptb)
    {
        float threshold = 6000; // in mm
        if((point.x != INVALID_VALUE) && (point.y != INVALID_VALUE) && 
        (CalcDistance(point, pta) <= threshold || CalcDistance(point, ptb) <= threshold)) {
            return (fabs((ptb.y - pta.y) * point.x + (pta.x - ptb.x) * point.y + ((ptb.x * pta.y) - (pta.x * ptb.y)))) / (sqrt(pow(ptb.y - pta.y, 2) + pow(pta.x - ptb.x, 2)));
    
        }

        return -1;
    }

    bool CompareDistance(const std::pair<Fsm::FusionSlotInfo, float>& p1, const std::pair<Fsm::FusionSlotInfo, float>& p2){
        return p1.second < p2.second;
    }

    std::vector<Fsm::FusionSlotInfo> findClosesParkingSpots(const POINT_I& car_position, const std::vector<Fsm::FusionSlotInfo>& parking_spots, int num_closest){
        std::vector<std::pair<Fsm::FusionSlotInfo, float>> distances;

        // 计算每个车位和自车的距离
        for(const auto& spot : parking_spots){
            POINT_I point_a, point_b;
            point_a.x = spot.pt[0].x;
            point_a.y = spot.pt[0].y;
            point_b.x = spot.pt[1].x;
            point_b.y = spot.pt[1].y;
            
            float dist  = CalPointAndLineDistance(car_position, point_a, point_b);
            distances.push_back({spot, dist});
        }

        // 按距离排序
        std::sort(distances.begin(), distances.end(), CompareDistance);

        // 取出前num_closest个最近车位
        std::vector<Fsm::FusionSlotInfo> closest_spots;
        for(int i = 0; i < num_closest && i < distances.size(); ++i){
            closest_spots.push_back(distances[i].first);
        }

        return closest_spots;
    }

    POINT_I coordConvert_car_center(const POINT_I& slot)
    {
        POINT_I grand;

        float REAR_AXEL_TO_CENTER = (VEHICLE_LENGTH / 2) - REAR_AXLE_CENTER_VEHICLE_REAR;

        float x = slot.x - BIRD_VIEW_HEIGHT/2;
        float y = BIRD_VIEW_HEIGHT/2 - slot.y + (REAR_AXEL_TO_CENTER / LR_BIRD_PIXECL_2_WORLD);

        grand.x = x * LR_BIRD_PIXECL_2_WORLD;
        grand.y = y * LR_BIRD_PIXECL_2_WORLD;

        return grand;
    }

    apaSlotListInfo ConvertSingeleframe2Local(const std::vector<padVisionSlotCoord> &singleframeslot){
         apaSlotListInfo local_slots;
        apaSlotInfo rect_local;
        int singleframe_size = singleframeslot.size();
        local_slots.slots_in_cur_frame.reserve(singleframe_size);

        for(int icnt = 0; icnt < singleframeslot.size(); icnt++){
            
            POINT_I slot_a_pt, slot_b_pt, slot_c_pt, slot_d_pt;

            slot_a_pt.x = singleframeslot[icnt].a.x;
            slot_a_pt.y = singleframeslot[icnt].a.y;
            slot_b_pt.x = singleframeslot[icnt].b.x;
            slot_b_pt.y = singleframeslot[icnt].b.y;
            slot_c_pt.x = singleframeslot[icnt].c.x;
            slot_c_pt.y = singleframeslot[icnt].c.y;
            slot_d_pt.x = singleframeslot[icnt].d.x;
            slot_d_pt.y = singleframeslot[icnt].d.y;

            rect_local.rectInfo.pt[0].x = coordConvert_car_center(slot_a_pt).x;
            rect_local.rectInfo.pt[0].y = coordConvert_car_center(slot_a_pt).y;
            rect_local.rectInfo.pt[1].x = coordConvert_car_center(slot_b_pt).x;
            rect_local.rectInfo.pt[1].y = coordConvert_car_center(slot_b_pt).y;
            rect_local.rectInfo.pt[2].x = coordConvert_car_center(slot_c_pt).x;
            rect_local.rectInfo.pt[2].y = coordConvert_car_center(slot_c_pt).y;
            rect_local.rectInfo.pt[3].x = coordConvert_car_center(slot_d_pt).x;
            rect_local.rectInfo.pt[3].y = coordConvert_car_center(slot_d_pt).y;

            rect_local.rectInfo.PStype = singleframeslot[icnt].bayType;

            local_slots.slots_in_cur_frame.push_back(rect_local);
        }
        return local_slots;
    }

    bool isNeedSingleframe2Update(apaSlotInfo slot_list_a, apaSlotInfo slot_list_b){
        POINT_I slot_a, slot_b, single_slot_a, single_slot_b;

        slot_a.x = slot_list_a.rectInfo.pt[0].x;
        slot_a.y = slot_list_a.rectInfo.pt[0].y;
        slot_b.x = slot_list_a.rectInfo.pt[1].x;
        slot_b.y = slot_list_a.rectInfo.pt[1].y;

        single_slot_a.x = slot_list_b.rectInfo.pt[0].x;
        single_slot_a.y = slot_list_b.rectInfo.pt[0].y;
        single_slot_b.x = slot_list_b.rectInfo.pt[1].x;
        single_slot_b.y = slot_list_b.rectInfo.pt[1].y;

        int threadhole_a = CalcDistance_I(slot_a, single_slot_a);
        int threadhole_b = CalcDistance_I(slot_b, single_slot_b);

        // same slot
        if ((threadhole_a + threadhole_b) / 2 < 300){
            return true;
        }else{
            return false;
        }
    }
}