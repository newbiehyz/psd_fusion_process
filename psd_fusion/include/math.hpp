#pragma once

#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include <cmath> // For std::sqrt, std::fabs, std::pow, M_PI
#include <vector>
#include <utility> // For std::pair
#include <algorithm> // For std::sort, std::min
#include <array> // For std::array if used in Eigen functions
// 如果你的Eigen库路径正确配置，可能还需要类似下面的包含
// #include <Eigen/Dense> // For Eigen::Vector3f, Eigen::Matrix3f etc.

// 这些宏定义通常也应该定义在某个公共的头文件中，而不是每个头文件都包含
// 如果它们是常量，最好定义为 const int/float/double
#define INVALID_VALUE 99999999
#define VEHICLE_LENGTH 5259.9
#define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7

// 请检查 BIRD_VIEW_HEIGHT 和 LR_BIRD_PIXECL_2_WORLD 的定义在哪里。
// 如果它们是宏，可能也需要移动到公共的头文件，或者作为命名空间内的常量。
// 假设它们在 psd_fusion_process_header.h 或 apa_define.h 中。

namespace math{
    // --- 只保留函数声明，函数体已移到 math.cpp ---

    int CalcDistance(POINT_I a, POINT_I b);
    float CalcDistanceF(POINT_F a, POINT_F b);
    int CalPointAndLineDistance(const POINT_I& point, const POINT_I& pta, const POINT_I& ptb);
    bool CompareDistance(const std::pair<Sfus::FusionSlotInfo, float>& p1, const std::pair<Sfus::FusionSlotInfo, float>& p2);
    std::vector<Sfus::FusionSlotInfo> findClosesParkingSpots(const POINT_F& car_position, const std::vector<Sfus::FusionSlotInfo>& parking_spots, int num_closest);
    POINT_F coordConvert_car_center(const POINT_F& slot);
    POINT_I coordConvert_car_center_to_pixel(const POINT_I& car_center);
    apaSlotListInfo ConvertSingeleframe2Local(const std::vector<padVisionSlotCoord> &singleframeslot);
    bool isNeedSingleframe2Update(apaSlotInfo slot_list_a, apaSlotInfo slot_list_b);
    void adjustOutputSlotRectOrder(apaSlotListInfo& slot_list_info);

} // namespace math