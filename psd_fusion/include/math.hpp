#pragma once

#include "apa_define.h"

#define INVALID_VALUE 99999999
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
}