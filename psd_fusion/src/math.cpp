#include "math.hpp" // 包含对应的头文件，以便获取函数声明
#include <cmath> // 确保包含数学函数所需的头文件
#include <algorithm> // For std::sort, std::min
#include <vector>
#include <utility> // For std::pair
#include <array> // For std::array
// 如果 Eigen 库是独立于 math.hpp 包含的，确保这里也包含
#include <Eigen/Dense> // 确保包含了 Eigen 的头文件

// 如果这些宏是全局的，并且在 math.cpp 中也需要，它们可能需要被包含，
// 或者在 math.cpp 中再次定义（如果它们是文件私有的常量）
// 但更好的做法是将其定义在一个通用的头文件中并被包含。
// 假设它们在 math.hpp 中定义一次，并通过它传递过来。
// #define VEHICLE_LENGTH 5259.9
// #define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7
// #define BIRD_VIEW_HEIGHT 896 // 示例，请替换为实际值
// #define LR_BIRD_PIXECL_2_WORLD 10.0f // 示例，请替换为实际值

namespace math{

    int CalcDistance(POINT_I a, POINT_I b)
    {
        float dis = std::sqrt(static_cast<float>((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y)));
        return static_cast<int>(dis); // 确保返回类型匹配
    }

    float CalcDistanceF(POINT_F a, POINT_F b)
    {
        float dis = std::sqrt(static_cast<float>((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y)));
        return dis;
    }

    int CalPointAndLineDistance(const POINT_I& point, const POINT_I& pta, const POINT_I& ptb)
    {
        float threshold = 6000; // in mm
        if((point.x != INVALID_VALUE) && (point.y != INVALID_VALUE) &&
        (CalcDistance(point, pta) <= threshold || CalcDistance(point, ptb) <= threshold)) {
            // 确保除数不为零，并进行浮点运算
            float denominator = std::sqrt(std::pow(static_cast<float>(ptb.y - pta.y), 2) + std::pow(static_cast<float>(pta.x - ptb.x), 2));
            if (denominator < 1e-6) { // 避免除以零
                return -1; // 或者其他错误处理
            }
            return static_cast<int>(std::fabs(static_cast<float>((ptb.y - pta.y) * point.x + (pta.x - ptb.x) * point.y + ((ptb.x * pta.y) - (pta.x * ptb.y)))) / denominator);
        }
        return -1;
    }

    bool CompareDistance(const std::pair<Sfus::FusionSlotInfo, float>& p1, const std::pair<Sfus::FusionSlotInfo, float>& p2){
        return p1.second < p2.second;
    }

    std::vector<Sfus::FusionSlotInfo> findClosesParkingSpots(const POINT_F& car_position, const std::vector<Sfus::FusionSlotInfo>& parking_spots, int num_closest){

        std::vector<std::pair<Sfus::FusionSlotInfo, float>> distances;

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

            slot_center.x = (point_a.x + point_b.x + point_c.x + point_d.x) / 4;
            slot_center.y = (point_a.y + point_b.y + point_c.y + point_d.y) / 4;
            LOGD("Parking Spot Center: (%f, %f), Distance to car: %f m",slot_center.x, slot_center.y, dist);
        }

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
            closest_spots[i].pt[3].y);
        }

        return closest_spots;
    }

    POINT_F coordConvert_car_center(const POINT_F& slot)
    {
        POINT_F grand;

        float REAR_AXEL_TO_CENTER = (VEHICLE_LENGTH / 2.0f) - REAR_AXLE_CENTER_VEHICLE_REAR; // 使用浮点数

        // 假设 BIRD_VIEW_HEIGHT 和 LR_BIRD_PIXECL_2_WORLD 在某个地方被定义为宏或常量
        // 这里需要你提供这些宏的具体定义或它们所在的头文件
        // 为了编译通过，我暂时假设它们是全局宏，但请检查你的项目
        #ifndef BIRD_VIEW_HEIGHT
        #define BIRD_VIEW_HEIGHT 896.0f // 示例值，请根据实际定义修改
        #endif
        #ifndef LR_BIRD_PIXECL_2_WORLD
        #define LR_BIRD_PIXECL_2_WORLD 10.0f // 示例值，请根据实际定义修改
        #endif

        float x = slot.x - BIRD_VIEW_HEIGHT/2.0f;
        float y = BIRD_VIEW_HEIGHT/2.0f - slot.y + (REAR_AXEL_TO_CENTER / LR_BIRD_PIXECL_2_WORLD);

        grand.x = x * LR_BIRD_PIXECL_2_WORLD;
        grand.y = y * LR_BIRD_PIXECL_2_WORLD;

        return grand;
    }

    POINT_I coordConvert_car_center_to_pixel(const POINT_I& car_center)
    {
        POINT_I pixel;

        float REAR_AXEL_TO_CENTER = (VEHICLE_LENGTH / 2.0f) - REAR_AXLE_CENTER_VEHICLE_REAR;

        #ifndef BIRD_VIEW_HEIGHT
        #define BIRD_VIEW_HEIGHT 896.0f // 示例值，请根据实际定义修改
        #endif
        #ifndef LR_BIRD_PIXECL_2_WORLD
        #define LR_BIRD_PIXECL_2_WORLD 10.0f // 示例值，请根据实际定义修改
        #endif

        float x_pixel = static_cast<float>(car_center.x) / LR_BIRD_PIXECL_2_WORLD;
        float y_pixel = static_cast<float>(car_center.y) / LR_BIRD_PIXECL_2_WORLD;

        pixel.x = static_cast<int>(x_pixel + BIRD_VIEW_HEIGHT / 2.0f);
        pixel.y = static_cast<int>(BIRD_VIEW_HEIGHT / 2.0f - (y_pixel - (REAR_AXEL_TO_CENTER / LR_BIRD_PIXECL_2_WORLD)));

        return pixel;
    }

    apaSlotListInfo ConvertSingeleframe2Local(const std::vector<padVisionSlotCoord> &singleframeslot){
        apaSlotListInfo local_slots;
        apaSlotInfo rect_local;
        int singleframe_size = singleframeslot.size();
        local_slots.slots_in_cur_frame.reserve(singleframe_size);

        for(int icnt = 0; icnt < singleframeslot.size(); icnt++){

            POINT_F slot_a_pt, slot_b_pt, slot_c_pt, slot_d_pt;

            slot_a_pt.x = static_cast<float>(singleframeslot[icnt].a.x); // 确保类型匹配或进行转换
            slot_a_pt.y = static_cast<float>(singleframeslot[icnt].a.y);
            slot_b_pt.x = static_cast<float>(singleframeslot[icnt].b.x);
            slot_b_pt.y = static_cast<float>(singleframeslot[icnt].b.y);
            slot_c_pt.x = static_cast<float>(singleframeslot[icnt].c.x);
            slot_c_pt.y = static_cast<float>(singleframeslot[icnt].c.y);
            slot_d_pt.x = static_cast<float>(singleframeslot[icnt].d.x);
            slot_d_pt.y = static_cast<float>(singleframeslot[icnt].d.y);

            // 请确保 coordConvert_car_center 的输入是 POINT_F，输出也是 POINT_F
            // 然后再将结果的 .x/.y 赋给 POINT_I 的 .x/.y (如果 apaSlotInfo.rectInfo.pt 是 POINT_I 类型)
            POINT_F converted_a = coordConvert_car_center(slot_a_pt);
            POINT_F converted_b = coordConvert_car_center(slot_b_pt);
            POINT_F converted_c = coordConvert_car_center(slot_c_pt);
            POINT_F converted_d = coordConvert_car_center(slot_d_pt);

            rect_local.rectInfo.pt[0].x = static_cast<int>(converted_a.x);
            rect_local.rectInfo.pt[0].y = static_cast<int>(converted_a.y);
            rect_local.rectInfo.pt[1].x = static_cast<int>(converted_b.x);
            rect_local.rectInfo.pt[1].y = static_cast<int>(converted_b.y);
            rect_local.rectInfo.pt[2].x = static_cast<int>(converted_c.x);
            rect_local.rectInfo.pt[2].y = static_cast<int>(converted_c.y);
            rect_local.rectInfo.pt[3].x = static_cast<int>(converted_d.x);
            rect_local.rectInfo.pt[3].y = static_cast<int>(converted_d.y);

            rect_local.rectInfo.PStype = singleframeslot[icnt].bayType;

            local_slots.slots_in_cur_frame.push_back(rect_local);
        }
        return local_slots;
    }

    bool isNeedSingleframe2Update(apaSlotInfo slot_list_a, apaSlotInfo slot_list_b){
        int center_x = 0;
        for (int i = 0; i < 4; ++i) {
            center_x += slot_list_a.rectInfo.pt[i].x;
        }
        center_x /= 4;
        LOGD("single_frame slot center_x: %d", center_x);

        POINT_I pt_a1 = slot_list_a.rectInfo.pt[0];
        POINT_I pt_b1 = slot_list_a.rectInfo.pt[1];
        POINT_I pt_c1 = slot_list_a.rectInfo.pt[2];

        POINT_I pt_a2 = slot_list_b.rectInfo.pt[0];
        POINT_I pt_b2 = slot_list_b.rectInfo.pt[1];
        POINT_I pt_c2 = slot_list_b.rectInfo.pt[2];

        int distance1 = 0;
        int distance2 = 0;

        if (center_x >= -1200 && center_x <= 1200) {
            distance1 = CalcDistance(pt_b1, pt_b2);
            distance2 = CalcDistance(pt_c1, pt_c2);
            LOGD("use BC compare: (%d,%d)-(%d,%d), (%d,%d)-(%d,%d)",
                pt_b1.x, pt_b1.y, pt_b2.x, pt_b2.y,
                pt_c1.x, pt_c1.y, pt_c2.x, pt_c2.y);
        } else {
            distance1 = CalcDistance(pt_a1, pt_a2);
            distance2 = CalcDistance(pt_b1, pt_b2);
            LOGD("use AB compare: (%d,%d)-(%d,%d), (%d,%d)-(%d,%d)",
                pt_a1.x, pt_a1.y, pt_a2.x, pt_a2.y,
                pt_b1.x, pt_b1.y, pt_b2.x, pt_b2.y);
        }

        LOGD("single_frame slot compare: distance1: %d, distance2: %d", distance1, distance2);

        return ((distance1 + distance2) / 2 < 800);
    }

    void adjustOutputSlotRectOrder(apaSlotListInfo& slot_list_info)
    {
        for (auto& slot : slot_list_info.slots_in_cur_frame) {
            std::array<Eigen::Vector3f, 4> corners_world;
            for (int i = 0; i < 4; ++i) {
                corners_world[i] = Eigen::Vector3f(
                    static_cast<float>(slot.rectInfo.pt[i].x),
                    static_cast<float>(slot.rectInfo.pt[i].y),
                    0.0f
                );
            }

            int center_x = 0;
            for (int i = 0; i < 4; ++i) {
                center_x += slot.rectInfo.pt[i].x;
            }
            center_x /= 4;
            bool is_left = (center_x < 0);

            std::sort(corners_world.begin(), corners_world.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                return a.x() < b.x();
            });

            std::array<Eigen::Vector3f, 2> left_pts = {corners_world[0], corners_world[1]}; // Rename to avoid conflict
            std::array<Eigen::Vector3f, 2> right_pts = {corners_world[2], corners_world[3]}; // Rename to avoid conflict

            std::sort(left_pts.begin(), left_pts.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                return a.y() > b.y();
            });
            std::sort(right_pts.begin(), right_pts.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                return a.y() > b.y();
            });

            std::array<Eigen::Vector3f, 4> ordered;
            if (is_left) {
                ordered = {right_pts[1], right_pts[0], left_pts[0], left_pts[1]}; // A, B, C, D
            } else {
                ordered = {left_pts[1], left_pts[0], right_pts[0], right_pts[1]}; // A, B, C, D
            }

            for (int i = 0; i < 4; ++i) {
                slot.rectInfo.pt[i].x = static_cast<int>(ordered[i].x());
                slot.rectInfo.pt[i].y = static_cast<int>(ordered[i].y());
            }
        }
    }

} // namespace math