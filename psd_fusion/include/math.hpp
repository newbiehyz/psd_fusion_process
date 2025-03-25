#pragma once

#include "apa_define.h"
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"


#define INVALID_VALUE 99999999
#define VEHICLE_LENGTH 5259.9
#define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7
namespace math{
    int CalcDistance(POINT_I a, POINT_I b)
    {
        float dis = sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
        return dis;
    }

    float CalcDistanceF(POINT_F a, POINT_F b)
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

    bool CompareDistance(const std::pair<Sfus::FusionSlotInfo, float>& p1, const std::pair<Sfus::FusionSlotInfo, float>& p2){
        return p1.second < p2.second;
    }

    std::vector<Sfus::FusionSlotInfo> findClosesParkingSpots(const POINT_F& car_position, const std::vector<Sfus::FusionSlotInfo>& parking_spots, int num_closest){
        
        std::vector<std::pair<Sfus::FusionSlotInfo, float>> distances;

        // 计算每个车位和自车的距离
        for(const auto& spot : parking_spots){
            POINT_F point_a, point_b, point_c, point_d, slot_center;
            point_a.x = spot.pt[0].x;
            point_a.y = spot.pt[0].y;
            point_b.x = spot.pt[1].x;
            point_b.y = spot.pt[1].y;
            point_c.x = spot.pt[2].x;
            point_c.y = spot.pt[2].y;
            point_d.x = spot.pt[3].x;
            point_d.y = spot.pt[3].y;
            slot_center.x = (point_a.x + point_b.x + point_c.x + point_d.x) / 4;
            slot_center.y = (point_a.y + point_b.y + point_c.y + point_d.y) / 4;
            
            float dist = CalcDistanceF(car_position, slot_center);
            distances.push_back({spot, dist});
        }

        // 按距离排序
        std::sort(distances.begin(), distances.end(), CompareDistance);

        // Print the sorted parking spots and their distances to the car
        for (const auto& pair : distances) {
            const Sfus::FusionSlotInfo& spot = pair.first;
            const float dist = pair.second;
            POINT_F point_a, point_b, point_c, point_d, slot_center;

            point_a.x = spot.pt[0].x;
            point_a.y = spot.pt[0].y;
            point_b.x = spot.pt[1].x;
            point_b.y = spot.pt[1].y;
            point_c.x = spot.pt[2].x;
            point_c.y = spot.pt[2].y;
            point_d.x = spot.pt[3].x;
            point_d.y = spot.pt[3].y;
            
            // Calculate the center of the parking spot again for printing
            slot_center.x = (point_a.x + point_b.x + point_c.x + point_d.x) / 4;
            slot_center.y = (point_a.y + point_b.y + point_c.y + point_d.y) / 4;
            LOGD("Parking Spot Center: (%f, %f), Distance to car:  %f m",slot_center.x, slot_center.y, dist);
        }

        // 取出前num_closest个最近车位
        std::vector<Sfus::FusionSlotInfo> closest_spots;
        for(int i = 0; i < num_closest && i < distances.size(); ++i){
            closest_spots.push_back(distances[i].first);
        }

        for (int i = 0; i < closest_spots.size(); ++i){
            LOGD("closet spot no.%d, ID: %d, (%f,%f), (%f,%f), (%f,%f), (%f,%f)",
            i+1,
            closest_spots[i].slotLabel,
            closest_spots[i].pt[0].x,
            closest_spots[i].pt[0].y,
            closest_spots[i].pt[1].x,
            closest_spots[i].pt[1].y,
            closest_spots[i].pt[2].x,
            closest_spots[i].pt[2].y,
            closest_spots[i].pt[3].x,
            closest_spots[i].pt[3].y)
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
        LOGD("single_frame slot compare: outputSlot_FUSED: (%d, %d), (%d, %d)",slot_a.x,slot_a.y,slot_b.x,slot_b.y);
        LOGD("single_frame slot compare: singleframe_local: (%d, %d), (%d, %d)",single_slot_a.x,single_slot_a.y,single_slot_b.x,single_slot_b.y);

        int threadhole_a = CalcDistance(slot_a, single_slot_a);
        int threadhole_b = CalcDistance(slot_b, single_slot_b);
        LOGD("single_frame slot compare: threadhole_a: %d, threadhole_b: %d",threadhole_a, threadhole_b);

        // same slot
        if ((threadhole_a + threadhole_b) / 2 < 800){
            return true;
        }else{
            return false;
        }
    }

    void adjustOutputSlotFusedRectOrder(apaSlotListInfo& slot_list_info)
    {
        for (auto& slot : slot_list_info.slots_in_cur_frame) {
            // 将 rectInfo 的 4 个点转换为 Eigen::Vector3f
            std::array<Eigen::Vector3f, 4> corners_world;
            for (int i = 0; i < 4; ++i) {
                corners_world[i] = Eigen::Vector3f(
                    static_cast<float>(slot.rectInfo.pt[i].x),
                    static_cast<float>(slot.rectInfo.pt[i].y),
                    0.0f
                );
            }

            // 计算车位中心点 x 坐标，判断是否在原点左侧
            int center_x = 0;
            for (int i = 0; i < 4; ++i) {
                center_x += slot.rectInfo.pt[i].x;
            }
            center_x /= 4;
            bool is_left = (center_x < 0);

            // 调整角点顺序
            // 按照 x 轴排序，先排左边的两个点，再排右边的两个点
            std::sort(corners_world.begin(), corners_world.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                return a.x() < b.x();
            });

            // 左侧两个点，右侧两个点
            std::array<Eigen::Vector3f, 2> left = {corners_world[0], corners_world[1]};
            std::array<Eigen::Vector3f, 2> right = {corners_world[2], corners_world[3]};

            // 按 y 轴排序（y 值大的在前），确保 top 和 bottom
            std::sort(left.begin(), left.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                return a.y() > b.y();
            });
            std::sort(right.begin(), right.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                return a.y() > b.y();
            });

            std::array<Eigen::Vector3f, 4> ordered;
            if (is_left) {
                // 左侧车位顺序：右下，右上，左上，左下
                ordered = {right[1], right[0], left[0], left[1]}; // A, B, C, D
            } else {
                // 右侧车位顺序：左下，左上，右上，右下
                ordered = {left[1], left[0], right[0], right[1]}; // A, B, C, D
            }

            // 写回 rectInfo.pt
            for (int i = 0; i < 4; ++i) {
                slot.rectInfo.pt[i].x = static_cast<int>(ordered[i].x());
                slot.rectInfo.pt[i].y = static_cast<int>(ordered[i].y());
            }
        }
    }



}