/**
* Copyright @ 2020 - 2020 iAUTO(Shanghai) Co., Ltd.
* All Rights Reserved.
*
* Copyright @ 2020 - 2020 Pan Asia Technical Automotive Center Co., Ltd.
* All Rights Reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are NOT permitted except as agreed by
* iAUTO(Shanghai) Co., Ltd.
* or
* Pan Asia Technical Automotive Center Co., Ltd.
* *
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file PSD_FusionModuleIF.h
 * @brief PSD_FusionModuleIF
 *
 * 
 *
 * @attention It is only for C++. It cannot be used in C.
 */

#include "PSD_FusionModuleIF.h"
#include "stdio.h"
#include "string.h"
#include <cfloat>
#include "Eigen/Dense"
//#include "apa_log.h"
#include "math.h"
#include <algorithm>
#include "iou.h"
#include "apa_define.h"
//#include "apa_performance.h"
#include <iostream>
#include <fstream>

#define PI 3.1415926
#define VEHICLE_WIDTH 2000
#define VEHICLE_LENGTH 5259.9 //458:5245.0
#define REAR_AXLE_CENTER_VEHICLE_REAR 1130 //e2sb:1100,458:1130
#define FRONT_SONAR_PITCH_HEAD 400
#define FRONT_REAR_SONAR_DISTANCE 3900
#define NUM 12
#define MAX_DISTANCE 30000
#define INVALID_DISTANCE 32000
#define OBSTACLE_POINTS_SIZE 1000 
#define OBSTACLE_POINTS_THRESHOLD 3000
#define INVALID_VALUE 99999999
#define EFFECTIVE_OBSTACLE_POINT_DISTANCE 8000

#define EFFECTIVE_THRESHOLD 5

#define MAX_SLOT_NUM 200

#define EFFECTIVE_SLOT_Y_1 113
#define EFFECTIVE_SLOT_Y_2 335

using namespace IOU;
#define wheel_base 3.16
#define VERTICAL 0
#define PARALLEL 1
#define SLANT 2
#define KF true

bool PSD_FusionModuleIF::Initialize()
{
    m_select_slot_label_id = -1;
    m_apa_psinfo.WorldoutRect.clear();
    m_slot_direction = 0;
    m_next_available_label_idx = 1;

    int ego_wheelbase = wheel_base;
    float ipm2baselink_ = 0.5 * ego_wheelbase;
    float ipm2baselink_pixel_ratio =
        ipm2baselink_ * ipmp_.focal_length / ipmp_.ipm_height;
    intrinsic_car2ipm_ << 0, -ipmp_.focal_length,
                          ipmp_.ipm_width * ipmp_.cx_ratio, -ipmp_.focal_length, 0,
                          ipmp_.ipm_height * (ipmp_.cy_ratio + ipm2baselink_pixel_ratio), 0, 0, 1;
    intrinsic_ipm2car_ = intrinsic_car2ipm_.inverse();
    return true;
}

bool PSD_FusionModuleIF::Start()
{
    return true;
}

bool PSD_FusionModuleIF::Stop()
{
    return true;
}

bool PSD_FusionModuleIF::Destroy()
{
    return true;
}

void PSD_FusionModuleIF::UpdateVechiclePose(const padVehiclePose& pose_global)
{
    m_vehicle_pose = pose_global;
}

POINT_I PSD_FusionModuleIF::coordConvert_car_center(const padPoint& slot)
{
    POINT_I grand;

    float x = slot.x - BIRD_VIEW_HEIGHT/2;
    float y = BIRD_VIEW_HEIGHT/2 - slot.y;

    grand.x = x * LR_BIRD_PIXECL_2_WORLD;
    grand.y = y * LR_BIRD_PIXECL_2_WORLD;

    return grand;
}

Kalman_filterPtr PSD_FusionModuleIF::check_slot_existance(
    const QuadInfoPtr& quad_info) {
    if (slots_map_.empty()) {
        return nullptr;
    }

    Eigen::Vector3f center_sum = quad_info->corners_world.rowwise().sum(); // 对每一行（即 x、y、z 坐标）进行求和，得到总和向量
    Eigen::Vector3f center = center_sum / quad_info->corners_world.cols(); // 将总和向量除以列数（角点数量），得到中心点的坐标 center
    if (slots_map_.size() < 2) {
        if (slots_map_.begin()->second->point_in_slot(center))
            return slots_map_.begin()->second;
        return nullptr;
    };
    point_t quad_center{center.x(), center.y()};
    auto nearest_index = slots_tree_->nearest_index(quad_center);
   
    const auto& check_slot = slots_map_.at(slots_remap_.at(nearest_index));

    if (check_slot->point_in_slot(center)) return check_slot;

    auto neighbor_indexes = slots_tree_->neighborhood_indices(
        quad_center, psmp_.check_same_slot_range);
    for (const auto& index : neighbor_indexes) {
        if (index == nearest_index) continue;
       
        const auto& check_slot = slots_map_.at(slots_remap_.at(index));
        if (check_slot->point_in_slot(center)) return check_slot;
    }
   
    return nullptr;
};

POINT_I PSD_FusionModuleIF::coordConvert_global_dr(const padPoint& slot, const padVehiclePose& pose)
{
    POINT_I grand;

    //使图像坐标转为世界坐标
    //1 原点从RD左上角 转到 图像中心，x坐标平移图像中心，乘像素范围系数
    //2 y坐标平移到后周中心
    float x = (slot.x - BIRD_VIEW_HEIGHT / 2) * LR_BIRD_PIXECL_2_WORLD;
    float y = (BIRD_VIEW_HEIGHT/2.0 + (VEHICLE_LENGTH/2.0-REAR_AXLE_CENTER_VEHICLE_REAR)/LR_BIRD_PIXECL_2_WORLD - slot.y) * LR_BIRD_PIXECL_2_WORLD;

    float yaw = pose.yaw * PI / 180.0;

    //旋转矩阵计算，正负号
    grand.x = x * cos(yaw) + y * sin(yaw);
    grand.y = y * cos(yaw) - x * sin(yaw);

    //平移dr偏移量
    grand.x += pose.coord.x;
    grand.y -= pose.coord.y;

    return grand;
}

int PSD_FusionModuleIF::CalMixSideDistance(const apaSlotInfo& target_slot)
{
    std::vector<POINT_I> area_points;
    std::vector<int> distance;

    for(auto obstacle_point : m_obstacle_info) {
        std::vector<int> item = MixDistanceDataset(target_slot, obstacle_point);
        if(item.size() > 0) {
            distance.insert(distance.end(), item.begin(), item.end());
        }    
    }

    m_min_distance.clear();
    
    if(distance.size() > 0) {
        std::sort(distance.begin(), distance.end(), compareDistance);
        m_min_distance.insert(m_min_distance.begin(), distance.begin(), distance.begin() + EFFECTIVE_THRESHOLD);
        //APA_Debug_Log(APA_MODULE_ID_PSD_FUSION, " distance.size=%d, size=%d", distance.size(), m_min_distance.size());
        int min = m_min_distance[m_min_distance.size() - 1] * LR_BIRD_PIXECL_2_WORLD;
        //APA_Debug_Log(APA_MODULE_ID_PSD_FUSION, "min=%d",  min);
        return min;
    }

    return -1;
}

std::vector<int> PSD_FusionModuleIF::MixDistanceDataset(const apaSlotInfo& target_slot, const ObstacleInfo& obstacle_point)
{
    std::vector<int> distance;

    if(m_slot_direction == 0 ){
        int temp_fl = CalPointAndLineDistance(obstacle_point.front_left, target_slot.rectInfo.pt[0], target_slot.rectInfo.pt[1]); 
        if(temp_fl >= 0) {
            distance.push_back(temp_fl);
        }

        int temp_rl = CalPointAndLineDistance(obstacle_point.rear_left, target_slot.rectInfo.pt[0], target_slot.rectInfo.pt[1]);  
        if(temp_rl >= 0) {
            distance.push_back(temp_rl);
        }
    }
    else{
        int temp_lr = CalPointAndLineDistance(obstacle_point.front_right, target_slot.rectInfo.pt[0], target_slot.rectInfo.pt[1]);  
        if(temp_lr >= 0) {
            distance.push_back(temp_lr);
        }
    
        int temp_rr = CalPointAndLineDistance(obstacle_point.rear_right, target_slot.rectInfo.pt[0], target_slot.rectInfo.pt[1]);  
        if(temp_rr >= 0) {
            distance.push_back(temp_rr);
        }
    } 
    return distance;
}

bool PSD_FusionModuleIF::compareDistance(int pre, int current)
{
    return pre < current;
}

int PSD_FusionModuleIF::CalPointAndLineDistance(const POINT_I& point, const POINT_I& pta, const POINT_I& ptb)
{
    // float threshold = EFFECTIVE_OBSTACLE_POINT_DISTANCE / LR_BIRD_PIXECL_2_WORLD;
    float threshold = 6000; // in mm
    if((point.x != INVALID_VALUE) && (point.y != INVALID_VALUE) && 
       (CalcDistance(point, pta) <= threshold || CalcDistance(point, ptb) <= threshold)) {
        return (fabs((ptb.y - pta.y) * point.x + (pta.x - ptb.x) * point.y + ((ptb.x * pta.y) - (pta.x * ptb.y)))) / (sqrt(pow(ptb.y - pta.y, 2) + pow(pta.x - ptb.x, 2)));
  
    }

    return -1;
}


// void PSD_FusionModuleIF::CalStopDisAndLoc(apaSlotInfo psdmoutput, Fus::PkEmapObs &empobs, float &stopdis, int &stoplocation, int &lockinslot, int &obsinslot){
//     POINT_I wheelstop_dis, point_a, point_b, point_c, point_d;;
//     for (auto &obs : empobs.pkEmapObs){
//         if (obs.obsTyp == Fus::OBS_WHEELSTOP){
//             Eigen::Vector3f obs_point3f;
//             obs_point3f << obs.obsCenter.x * 1000.0, obs.obsCenter.y * 1000.0, obs.obsCenter.z * 1000.0;

//             //TODO
//             if (slots_map_.empty()) {
//                 return;
//             }

//             if (slots_map_.size() < 2) {
//                 if (slots_map_.begin()->second->point_in_rect(obs_point3f)){
//                     const auto& check_slot = slots_map_.begin()->second;
                    
//                     wheelstop_dis.x = obs_point3f.x();
//                     wheelstop_dis.y = obs_point3f.y();
                
//                     if (check_slot->GetSlotType() == SLOT_TYPE::VERTICALSLOT){
//                         point_a.x = check_slot->GetApoint().x();
//                         point_a.y = check_slot->GetApoint().y();
//                         point_b.x = check_slot->GetBpoint().x();
//                         point_b.y = check_slot->GetBpoint().y();

//                         point_c.x = check_slot->GetCpoint().x();
//                         point_c.y = check_slot->GetCpoint().y();
//                         point_d.x = check_slot->GetDpoint().x();
//                         point_d.y = check_slot->GetDpoint().y();

//                         //不确定限位块在AB还是在CD
//                         float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_a, point_b);
//                         float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_c, point_d);
//                         stopdis = std::max(temp_dis1, temp_dis2); 

//                         //判断离AB近还是离CD近
//                         if (temp_dis1 > temp_dis2){
//                             stoplocation = SOD_LOCATION_CD;
//                         }
//                         else if(temp_dis1 < temp_dis2){
//                             stoplocation = SOD_LOCATION_AB;
//                         }
//                         else{
//                             stoplocation = SOD_LOCATION_NO;
//                         }
                        
//                     }else if(check_slot->GetSlotType() == SLOT_TYPE::PARALLELSLOT){
//                         point_a.x = check_slot->GetApoint().x();
//                         point_a.y = check_slot->GetApoint().y();
//                         point_b.x = check_slot->GetBpoint().x();
//                         point_b.y = check_slot->GetBpoint().y();

//                         point_c.x = check_slot->GetCpoint().x();
//                         point_c.y = check_slot->GetCpoint().y();
//                         point_d.x = check_slot->GetDpoint().x();
//                         point_d.y = check_slot->GetDpoint().y();

//                         //不确定限位块在BC还是在AD
//                         float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_b, point_c);
//                         float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_a, point_d);
//                         stopdis = std::max(temp_dis1, temp_dis2); 

//                         //判断离BC近还是离AD近
//                         if (temp_dis1 > temp_dis2){
//                             stoplocation = SOD_LOCATION_DA;
//                         }
//                         else if(temp_dis1 < temp_dis2){
//                             stoplocation = SOD_LOCATION_BC;
//                         }
//                         else{
//                             stoplocation = SOD_LOCATION_NO;
//                         }
//                     }
//                     else{
//                         stopdis = 0.0;
//                     }
//                 }   
//                 return;
//             };
//             point_t obs_point2f{obs_point3f.x(), obs_point3f.y()};
//             auto nearest_index = slots_tree_->nearest_index(obs_point2f);

//             const auto& check_slot = slots_map_.at(slots_remap_.at(nearest_index));
            
//             // 找到最近的停车位
//             if(check_slot->point_in_rect(obs_point3f)){
                
//                 wheelstop_dis.x = obs_point3f.x();
//                 wheelstop_dis.y = obs_point3f.y();

//                 if (check_slot->GetSlotType() == SLOT_TYPE::VERTICALSLOT){
//                     point_a.x = check_slot->GetApoint().x();
//                     point_a.y = check_slot->GetApoint().y();
//                     point_b.x = check_slot->GetBpoint().x();
//                     point_b.y = check_slot->GetBpoint().y();

//                     //不确定限位块在AB还是在CD
//                     float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_a, point_b);
//                     float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_c, point_d);
//                     stopdis = std::max(temp_dis1, temp_dis2); 

//                     //判断离AB近还是离CD近
//                     if (temp_dis1 > temp_dis2){
//                         stoplocation = SOD_LOCATION_CD;
//                     }
//                     else if(temp_dis1 < temp_dis2){
//                         stoplocation = SOD_LOCATION_AB;
//                     }
//                     else{
//                         stoplocation = SOD_LOCATION_NO;
//                     }

//                 }else if(check_slot->GetSlotType() == SLOT_TYPE::PARALLELSLOT){
//                     point_a.x = check_slot->GetApoint().x();
//                     point_a.y = check_slot->GetApoint().y();
//                     point_b.x = check_slot->GetBpoint().x();
//                     point_b.y = check_slot->GetBpoint().y();

//                     point_c.x = check_slot->GetCpoint().x();
//                     point_c.y = check_slot->GetCpoint().y();
//                     point_d.x = check_slot->GetDpoint().x();
//                     point_d.y = check_slot->GetDpoint().y();

//                     //不确定限位块在BC还是在AD
//                     float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_b, point_c);
//                     float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_a, point_d);
//                     stopdis = std::max(temp_dis1, temp_dis2); 

//                     //判断离BC近还是离AD近
//                     if (temp_dis1 > temp_dis2){
//                         stoplocation = SOD_LOCATION_DA;
//                     }
//                     else if(temp_dis1 < temp_dis2){
//                         stoplocation = SOD_LOCATION_BC;
//                     }
//                     else{
//                         stoplocation = SOD_LOCATION_NO;
//                     }
//                 }else{
//                     stopdis = 0.0;
//                 }
                
//             }

//             auto neighbor_indexes = slots_tree_->neighborhood_indices(obs_point2f, 2000);
//             for (const auto& index : neighbor_indexes) {
//                 if (index == nearest_index) continue;
       
//                 const auto& check_slot = slots_map_.at(slots_remap_.at(index));
//                 if (check_slot->point_in_rect(obs_point3f)){
//                     POINT_I wheelstop_dis, point_a, point_b;
                
//                     wheelstop_dis.x = obs_point3f.x();
//                     wheelstop_dis.y = obs_point3f.y();

//                     if (check_slot->GetSlotType() == SLOT_TYPE::VERTICALSLOT){
//                         point_a.x = check_slot->GetApoint().x();
//                         point_a.y = check_slot->GetApoint().y();
//                         point_b.x = check_slot->GetBpoint().x();
//                         point_b.y = check_slot->GetBpoint().y();

//                         //不确定限位块在AB还是在CD
//                         float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_a, point_b);
//                         float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_c, point_d);
//                         stopdis = std::max(temp_dis1, temp_dis2); 

//                         //判断离AB近还是离CD近
//                         if (temp_dis1 > temp_dis2){
//                             stoplocation = SOD_LOCATION_CD;
//                         }
//                         else if(temp_dis1 < temp_dis2){
//                             stoplocation = SOD_LOCATION_AB;
//                         }
//                         else{
//                             stoplocation = SOD_LOCATION_NO;
//                         }

//                     }else if(check_slot->GetSlotType() == SLOT_TYPE::PARALLELSLOT){
//                         POINT_I point_c, point_d;

//                         point_a.x = check_slot->GetApoint().x();
//                         point_a.y = check_slot->GetApoint().y();
//                         point_b.x = check_slot->GetBpoint().x();
//                         point_b.y = check_slot->GetBpoint().y();

//                         point_c.x = check_slot->GetCpoint().x();
//                         point_c.y = check_slot->GetCpoint().y();
//                         point_d.x = check_slot->GetDpoint().x();
//                         point_d.y = check_slot->GetDpoint().y();

//                         //不确定限位块在BC还是在AD
//                         float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_b, point_c);
//                         float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_a, point_d);
//                         stopdis = std::max(temp_dis1, temp_dis2); 

//                         //判断离BC近还是离AD近
//                         if (temp_dis1 > temp_dis2){
//                             stoplocation = SOD_LOCATION_DA;
//                         }
//                         else if(temp_dis1 < temp_dis2){
//                             stoplocation = SOD_LOCATION_BC;
//                         }
//                         else{
//                             stoplocation = SOD_LOCATION_NO;
//                         }

//                     }else{
//                         stopdis = 0.0;
//                     }
//                 }
//             }
            
//         }else if(obs.obsTyp == Fus::OBS_SLOT_LOCK){
//             Eigen::Vector3f obs_point3f;
//             obs_point3f << obs.obsCenter.x * 1000.0, obs.obsCenter.y * 1000.0, obs.obsCenter.z * 1000.0;
//             //没找到车位
//             if (slots_map_.empty()) {
//                 return;
//             }
//             //只有一个车位
//             if (slots_map_.size() < 2) {
//                 if (slots_map_.begin()->second->point_in_rect(obs_point3f)){
//                     // LOGD("LOCK IN SLOT");
//                     lockinslot = 1;
//                 }
//                 else{
//                     // LOGD("LOCK NOT IN SLOT");
//                     lockinslot = 0;
//                 }
//             };
//             //多个车位，找到最近的停车位
//             point_t obs_point2f{obs_point3f.x(), obs_point3f.y()};
//             auto nearest_index = slots_tree_->nearest_index(obs_point2f);
//             const auto& check_slot = slots_map_.at(slots_remap_.at(nearest_index));
//             if(check_slot->point_in_rect(obs_point3f)){
//                 // LOGD("LOCK IN SLOT");
//                 lockinslot = 1;
//             }
//             else{
//                 // LOGD("LOCK NOT IN SLOT");
//                 lockinslot = 0;
//             }
//             //在obs_point2f周围的半径2000mm内搜索其他邻近停车位
//             auto neighbor_indexes = slots_tree_->neighborhood_indices(obs_point2f, 2000);
//             for (const auto& index : neighbor_indexes) {
//                 if (index == nearest_index) continue;
       
//                 const auto& check_slot = slots_map_.at(slots_remap_.at(index));
//                 if (check_slot->point_in_rect(obs_point3f)){
//                     // LOGD("LOCK IN SLOT");
//                     lockinslot = 1;
//                 }
//                 else{
//                     // LOGD("LOCK NOT IN SLOT");
//                     lockinslot = 0;
//                 }
//             }
//         }

//         else if (obs.obsTyp != Fus::OBS_NULL){
//             Eigen::Vector3f obs_point3f;
//             obs_point3f << obs.obsCenter.x * 1000.0, obs.obsCenter.y * 1000.0, obs.obsCenter.z * 1000.0;

//             psdmoutput.rectInfo.pt[0].x


            
//             //没找到车位
//             if (slots_map_.empty()) {
//                 return;
//             }
//             //只有一个车位
//             if (slots_map_.size() < 2) {
//                 if (slots_map_.begin()->second->point_in_rect(obs_point3f)){
//                     // LOGD("OBS IN SLOT");
//                     obsinslot = 1;
//                 }
//                 else{
//                     // LOGD("OBS NOT IN SLOT");
//                     obsinslot = 0;
//                 }
//             };
//             //多个车位，找到最近的停车位
//             point_t obs_point2f{obs_point3f.x(), obs_point3f.y()};
//             auto nearest_index = slots_tree_->nearest_index(obs_point2f);
//             LOGD("OBS find in slot, nearest_index: %zu", nearest_index);
//             const auto& check_slot = slots_map_.at(slots_remap_.at(nearest_index));
//             if(check_slot->point_in_rect(obs_point3f)){
//                 // LOGD("OBS IN SLOT");
//                 obsinslot = 1;
//             }
//             else{
//                 // LOGD("OBS NOT IN SLOT");
//                 obsinslot = 0;
//             }
//             //在obs_point2f周围的半径2000mm内搜索其他邻近停车位
//             auto neighbor_indexes = slots_tree_->neighborhood_indices(obs_point2f, 2000);
//             for (const auto& index : neighbor_indexes) {
//                 if (index == nearest_index) continue;
       
//                 const auto& check_slot = slots_map_.at(slots_remap_.at(index));
//                 if (check_slot->point_in_rect(obs_point3f)){
//                     // LOGD("LOCK IN SLOT");
//                     obsinslot = 1;
//                 }
//                 else{
//                     // LOGD("LOCK NOT IN SLOT");
//                     obsinslot = 0;
//                 }
//             }
//         }
//     }
// }


void PSD_FusionModuleIF::CalStopDisAndLoc(const Fus::PkEmapObs &empobs){
    POINT_I wheelstop_dis, point_a, point_b, point_c, point_d;;
    for (auto &obs : empobs.pkEmapObs){
        if (obs.obsTyp == Fus::OBS_WHEELSTOP){
            Eigen::Vector3f obs_point3f;
            obs_point3f << obs.obsCenter.x * 1000.0, obs.obsCenter.y * 1000.0, obs.obsCenter.z * 1000.0;

            //没找到车位
            if (slots_map_.empty()) {
                return;
            }
            //只有一个车位
            if (slots_map_.size() < 2) {
                if (slots_map_.begin()->second->point_in_rect(obs_point3f)){
                    auto& check_slot = slots_map_.begin()->second;
                    
                    wheelstop_dis.x = obs_point3f.x();
                    wheelstop_dis.y = obs_point3f.y();
                
                    if (check_slot->GetSlotType() == SLOT_TYPE::VERTICALSLOT){
                        point_a.x = check_slot->GetApoint().x();
                        point_a.y = check_slot->GetApoint().y();
                        point_b.x = check_slot->GetBpoint().x();
                        point_b.y = check_slot->GetBpoint().y();
                        point_c.x = check_slot->GetCpoint().x();
                        point_c.y = check_slot->GetCpoint().y();
                        point_d.x = check_slot->GetDpoint().x();
                        point_d.y = check_slot->GetDpoint().y();

                        //不确定限位块在AB还是在CD
                        float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_a, point_b);
                        float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_c, point_d);
                        check_slot->stopper_distance_ = std::max(temp_dis1, temp_dis2); 
                        //判断离AB近还是离CD近
                        if (temp_dis1 > temp_dis2){
                            check_slot->stopper_location_ = SOD_LOCATION_CD;
                        }
                        else if(temp_dis1 < temp_dis2){
                            check_slot->stopper_location_ = SOD_LOCATION_AB;
                        }
                        else{
                            check_slot->stopper_location_ = SOD_LOCATION_NO;
                        }
                        LOGD("stopdis: %f, stoploc: %d",check_slot->stopper_distance_,check_slot->stopper_location_);
                        
                    }else if(check_slot->GetSlotType() == SLOT_TYPE::PARALLELSLOT){
                        point_a.x = check_slot->GetApoint().x();
                        point_a.y = check_slot->GetApoint().y();
                        point_b.x = check_slot->GetBpoint().x();
                        point_b.y = check_slot->GetBpoint().y();
                        point_c.x = check_slot->GetCpoint().x();
                        point_c.y = check_slot->GetCpoint().y();
                        point_d.x = check_slot->GetDpoint().x();
                        point_d.y = check_slot->GetDpoint().y();

                        //不确定限位块在BC还是在AD
                        float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_b, point_c);
                        float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_a, point_d);
                        check_slot->stopper_distance_ = std::max(temp_dis1, temp_dis2); 
                        //判断离BC近还是离AD近
                        if (temp_dis1 > temp_dis2){
                            check_slot->stopper_location_ = SOD_LOCATION_DA;
                        }
                        else if(temp_dis1 < temp_dis2){
                            check_slot->stopper_location_ = SOD_LOCATION_BC;
                        }
                        else{
                            check_slot->stopper_location_ = SOD_LOCATION_NO;
                        }
                        LOGD("stopdis: %f, stoploc: %d",check_slot->stopper_distance_,check_slot->stopper_location_);
                    }
                    else{
                        check_slot->stopper_distance_  = 0.0;
                        LOGD("stopdis: %f, stoploc: %d",check_slot->stopper_distance_,check_slot->stopper_location_);
                    }
                }   
                return;
            };
            //多个车位，找到最近的停车位
            point_t obs_point2f{obs_point3f.x(), obs_point3f.y()};
            auto nearest_index = slots_tree_->nearest_index(obs_point2f);
            auto& check_slot = slots_map_.at(slots_remap_.at(nearest_index));
            
            // 找到最近的停车位
            if(check_slot->point_in_rect(obs_point3f)){
                
                wheelstop_dis.x = obs_point3f.x();
                wheelstop_dis.y = obs_point3f.y();

                if (check_slot->GetSlotType() == SLOT_TYPE::VERTICALSLOT){
                    point_a.x = check_slot->GetApoint().x();
                    point_a.y = check_slot->GetApoint().y();
                    point_b.x = check_slot->GetBpoint().x();
                    point_b.y = check_slot->GetBpoint().y();
                    point_c.x = check_slot->GetCpoint().x();
                    point_c.y = check_slot->GetCpoint().y();
                    point_d.x = check_slot->GetDpoint().x();
                    point_d.y = check_slot->GetDpoint().y();

                    //不确定限位块在AB还是在CD
                    float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_a, point_b);
                    float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_c, point_d);
                    check_slot->stopper_distance_ = std::max(temp_dis1, temp_dis2); 
                    //判断离AB近还是离CD近
                    if (temp_dis1 > temp_dis2){
                        check_slot->stopper_location_ = SOD_LOCATION_CD;
                    }
                    else if(temp_dis1 < temp_dis2){
                        check_slot->stopper_location_ = SOD_LOCATION_AB;
                    }
                    else{
                        check_slot->stopper_location_ = SOD_LOCATION_NO;
                    }
                    LOGD("stopdis: %f, stoploc: %d",check_slot->stopper_distance_,check_slot->stopper_location_);

                }else if(check_slot->GetSlotType() == SLOT_TYPE::PARALLELSLOT){
                    point_a.x = check_slot->GetApoint().x();
                    point_a.y = check_slot->GetApoint().y();
                    point_b.x = check_slot->GetBpoint().x();
                    point_b.y = check_slot->GetBpoint().y();
                    point_c.x = check_slot->GetCpoint().x();
                    point_c.y = check_slot->GetCpoint().y();
                    point_d.x = check_slot->GetDpoint().x();
                    point_d.y = check_slot->GetDpoint().y();

                    //不确定限位块在BC还是在AD
                    float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_b, point_c);
                    float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_a, point_d);
                    check_slot->stopper_distance_ = std::max(temp_dis1, temp_dis2); 
                    //判断离BC近还是离AD近
                    if (temp_dis1 > temp_dis2){
                        check_slot->stopper_location_ = SOD_LOCATION_DA;
                    }
                    else if(temp_dis1 < temp_dis2){
                        check_slot->stopper_location_ = SOD_LOCATION_BC;
                    }
                    else{
                        check_slot->stopper_location_ = SOD_LOCATION_NO;
                    }
                    LOGD("stopdis: %f, stoploc: %d",check_slot->stopper_distance_,check_slot->stopper_location_);
                }else{
                    check_slot->stopper_distance_ = 0.0;
                    LOGD("stopdis: %f, stoploc: %d",check_slot->stopper_distance_,check_slot->stopper_location_);
                }
                
            }

            auto neighbor_indexes = slots_tree_->neighborhood_indices(obs_point2f, 2000);
            for (const auto& index : neighbor_indexes) {
                if (index == nearest_index) continue;
       
                auto& check_slot = slots_map_.at(slots_remap_.at(index));
                if (check_slot->point_in_rect(obs_point3f)){
                    POINT_I wheelstop_dis, point_a, point_b;
                
                    wheelstop_dis.x = obs_point3f.x();
                    wheelstop_dis.y = obs_point3f.y();

                    if (check_slot->GetSlotType() == SLOT_TYPE::VERTICALSLOT){
                        point_a.x = check_slot->GetApoint().x();
                        point_a.y = check_slot->GetApoint().y();
                        point_b.x = check_slot->GetBpoint().x();
                        point_b.y = check_slot->GetBpoint().y();
                        point_c.x = check_slot->GetCpoint().x();
                        point_c.y = check_slot->GetCpoint().y();
                        point_d.x = check_slot->GetDpoint().x();
                        point_d.y = check_slot->GetDpoint().y();

                        //不确定限位块在AB还是在CD
                        float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_a, point_b);
                        float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_c, point_d);
                        check_slot->stopper_distance_ = std::max(temp_dis1, temp_dis2); 

                        //判断离AB近还是离CD近
                        if (temp_dis1 > temp_dis2){
                            check_slot->stopper_location_ = SOD_LOCATION_CD;
                        }
                        else if(temp_dis1 < temp_dis2){
                            check_slot->stopper_location_ = SOD_LOCATION_AB;
                        }
                        else{
                            check_slot->stopper_location_ = SOD_LOCATION_NO;
                        }
                        LOGD("Near stopdis: %f, stoploc: %d",check_slot->stopper_distance_,check_slot->stopper_location_);

                    }else if(check_slot->GetSlotType() == SLOT_TYPE::PARALLELSLOT){
                        POINT_I point_c, point_d;

                        point_a.x = check_slot->GetApoint().x();
                        point_a.y = check_slot->GetApoint().y();
                        point_b.x = check_slot->GetBpoint().x();
                        point_b.y = check_slot->GetBpoint().y();
                        point_c.x = check_slot->GetCpoint().x();
                        point_c.y = check_slot->GetCpoint().y();
                        point_d.x = check_slot->GetDpoint().x();
                        point_d.y = check_slot->GetDpoint().y();

                        //不确定限位块在BC还是在AD
                        float temp_dis1 = CalPointAndLineDistance(wheelstop_dis, point_b, point_c);
                        float temp_dis2 = CalPointAndLineDistance(wheelstop_dis, point_a, point_d);
                        check_slot->stopper_distance_ = std::max(temp_dis1, temp_dis2); 

                        //判断离BC近还是离AD近
                        if (temp_dis1 > temp_dis2){
                            check_slot->stopper_location_ = SOD_LOCATION_DA;
                        }
                        else if(temp_dis1 < temp_dis2){
                            check_slot->stopper_location_ = SOD_LOCATION_BC;
                        }
                        else{
                            check_slot->stopper_location_ = SOD_LOCATION_NO;
                        }
                        LOGD("Near stopdis: %f, stoploc: %d",check_slot->stopper_distance_,check_slot->stopper_location_);

                    }else{
                        check_slot->stopper_distance_ = 0.0;
                        LOGD("Near stopdis: %f, stoploc: %d",check_slot->stopper_distance_,check_slot->stopper_location_);
                    }
                }
            }
            
        }else if(obs.obsTyp == Fus::OBS_SLOT_LOCK){
            Eigen::Vector3f obs_point3f;
            obs_point3f << obs.obsCenter.x * 1000.0, obs.obsCenter.y * 1000.0, obs.obsCenter.z * 1000.0;
            //没找到车位
            if (slots_map_.empty()) {
                return;
            }
            //只有一个车位
            if (slots_map_.size() < 2) {
                if (slots_map_.begin()->second->point_in_rect(obs_point3f)){
                    slots_map_.begin()->second->lock_in_slot_ = 1;
                }
                else{
                    slots_map_.begin()->second->lock_in_slot_ = 0;
                }
                LOGD("lock_in_slot: %d",slots_map_.begin()->second->lock_in_slot_);
            };
            //多个车位，找到最近的停车位
            point_t obs_point2f{obs_point3f.x(), obs_point3f.y()};
            auto nearest_index = slots_tree_->nearest_index(obs_point2f);
            auto& check_slot = slots_map_.at(slots_remap_.at(nearest_index));
            if(check_slot->point_in_rect(obs_point3f)){
                check_slot->lock_in_slot_ = 1;
            }
            else{
                check_slot->lock_in_slot_ = 0;
            }
            LOGD("lock_in_slot: %d",check_slot->lock_in_slot_);
            //在obs_point2f周围的半径2000mm内搜索其他邻近停车位
            auto neighbor_indexes = slots_tree_->neighborhood_indices(obs_point2f, 2000);
            for (const auto& index : neighbor_indexes) {
                if (index == nearest_index) continue;
       
                auto& check_slot = slots_map_.at(slots_remap_.at(index));
                if (check_slot->point_in_rect(obs_point3f)){
                    check_slot->lock_in_slot_ = 1;
                }
                else{
                    check_slot->lock_in_slot_ = 0;
                }
                LOGD("Near lock_in_slot: %d",check_slot->lock_in_slot_);
            }
        }

        else if (obs.obsTyp != Fus::OBS_NULL){
            Eigen::Vector3f obs_point3f;
            obs_point3f << obs.obsCenter.x * 1000.0, obs.obsCenter.y * 1000.0, obs.obsCenter.z * 1000.0;

            //没找到车位
            if (slots_map_.empty()) {
                return;
            }
            //只有一个车位
            if (slots_map_.size() < 2) {
                if (slots_map_.begin()->second->point_in_rect(obs_point3f)){
                    slots_map_.begin()->second->obs_in_slot_ = 1;
                }
                else{
                    slots_map_.begin()->second->obs_in_slot_ = 0;
                }
                LOGD("obs_in_slot_: %d",slots_map_.begin()->second->obs_in_slot_);
            };
            //多个车位，找到最近的停车位
            point_t obs_point2f{obs_point3f.x(), obs_point3f.y()};
            auto nearest_index = slots_tree_->nearest_index(obs_point2f);
            LOGD("OBS find in slot, nearest_index: %zu", nearest_index);
            auto& check_slot = slots_map_.at(slots_remap_.at(nearest_index));
            if(check_slot->point_in_rect(obs_point3f)){
                check_slot->obs_in_slot_ = 1;
            }
            else{
                check_slot->obs_in_slot_ = 0;
            }
            LOGD("obs_in_slot_: %d",check_slot->obs_in_slot_);
            //在obs_point2f周围的半径2000mm内搜索其他邻近停车位
            auto neighbor_indexes = slots_tree_->neighborhood_indices(obs_point2f, 2000);
            for (const auto& index : neighbor_indexes) {
                if (index == nearest_index) continue;
       
                auto& check_slot = slots_map_.at(slots_remap_.at(index));
                if (check_slot->point_in_rect(obs_point3f)){
                    check_slot->obs_in_slot_ = 1;
                }
                else{
                    check_slot->obs_in_slot_ = 0;
                }
                LOGD("Near obs_in_slot_: %d",check_slot->obs_in_slot_);
            }
        }
    }
}


void PSD_FusionModuleIF::UpdateVisionSlots(uint64_t frameid, std::vector<padVisionSlotCoord> slots, int status, int search_interrupt)
{
    auto start_update = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(m_psinfo_mutex);
    
    // if(slots.size() <= 0) {
    //     return;
    // }

    // 清空上一帧车位
    m_frame_id = frameid;
    m_output_slot.padRealTimeLocation.x = m_vehicle_pose.coord.x;
    m_output_slot.padRealTimeLocation.y = m_vehicle_pose.coord.y;
    m_output_slot.padRealTimeLocation.yaw = -m_vehicle_pose.yaw * PI / 180;
    m_output_slot.ullFrameId = frameid;

    // 清空车位2：update内部变量 //@TODO VC7 RELEASE
    if (status == 0 || status == 1 || status == 6 || status == 7 || search_interrupt == 2){
        slots_map_.clear();
        m_output_slot.slots_in_cur_frame.clear();
    }

    m_output_slot.slots_in_cur_frame.clear();
    m_output_slot.WorldoutRect.clear();
    int next_available_label_idx = m_apa_psinfo.WorldoutRect.size();
    for (auto slot : slots) 
    {
        //过滤以下车位：
        //超过鸟瞰图长宽
        if(slot.a.x > BIRD_VIEW_HEIGHT && slot.a.y > BIRD_VIEW_HEIGHT && slot.b.x > BIRD_VIEW_HEIGHT && slot.b.y > BIRD_VIEW_HEIGHT) {
            continue;
        }
        //一条线车位
        if((slot.a.x == slot.b.x && slot.a.y == slot.b.y) || (slot.a.x == slot.d.x && slot.a.y == slot.d.y)) {
            continue;
        }
        // // 中心点不在有效范围
        // if((slot.a.y + slot.b.y) / 2 < EFFECTIVE_SLOT_Y_1 || (slot.a.y + slot.b.y) / 2 > EFFECTIVE_SLOT_Y_2) {
        //     continue;
        // }

        if (KF){
            auto quad = std::make_shared<QuadInfo>();
            // filterSlotOccupy(slot.occupy); // 20250220 RD占用判断跳动，加滤波算法
            quad->occupy = slot.occupy;
            quad->material = slot.material;
            quad->quads.bottomRows<1>().setOnes();
            // 原始检测角点信息，乱序的
            Eigen::Vector2f tl, tr, bl, br;
            tl<<slot.a.x, slot.a.y;
            tr<<slot.b.x, slot.b.y;
            br<<slot.c.x, slot.c.y;
            bl<<slot.d.x, slot.d.y;
            quad->quads.col(0).head<2>() = tl;
            quad->quads.col(1).head<2>() = tr;
            quad->quads.col(2).head<2>() = br;
            quad->quads.col(3).head<2>() = bl;
            transform2world(m_vehicle_pose,quad);
            
            auto slot_existance = check_slot_existance(quad);
            if (slot_existance != nullptr) { //找到存在的车位
                Eigen::Vector3d pose{m_vehicle_pose.coord.x, m_vehicle_pose.coord.y, m_vehicle_pose.yaw};
                Eigen::Vector3f diff = pose.cast<float>() - slot_existance->GetSlotCenter();
                float dist = diff.head<2>().norm();
                if (slot_existance->IsConfiremd() &&
                    dist > slot_existance->GetMinDist2EgoCar()) {
                    continue;
                }
                // 已经存在与之对应的车位，用角点信息进行更新
                slot_existance->SetLatestFrameId(static_cast<uint32_t>(frameid));
                slot_existance->Update(quad);
                
            } else {
                auto result = std::make_shared<ParkingSlotResult>();
                mParkingLineMask_ptr=std::make_shared<PSMaskU8>(896, 896, 0);
                for (uint32_t row = 0; row < 896; row++) {
                    for (uint32_t col = 0; col < 896; col++) {
                        mParkingLineMask_ptr->At(col, row) = 1;
                    }
                }
                if (ProcessParkingSlotResult(*mParkingLineMask_ptr, slot, result)) {
                    transform2world(m_vehicle_pose, result, quad);
                    if (check_slot_existance(quad) != nullptr) {
                        continue;
                    }
                    auto new_slot = std::make_shared<Kalman_filter>(result, quad);
                    slots_map_[new_slot->GetSlotApaId()] = new_slot;
                    new_slot->SetLatestFrameId(static_cast<uint32_t>(frameid));
                    rebuild_slots_tree();
                }
            }
            
        }else{
            apaSlotInfo rect;
            apaSlotInfo rect_car_center;
            rect.rectInfo.iRectType = 0; // 0为视觉检测结果 
            rect.rectInfo.iSodType = 0;  // 0为没有障碍物，1为有障碍物，-1为未知情况（reset,或车停下来）
            // rect.rectInfo.iMinOtherSideDist = -1;
            rect.rectInfo.level=2; //跟踪值还是检测值 2为检测值，1为跟踪值
            rect.rectInfo.PStype = slot.bayType;
            rect.detect_frame_count = 1; 

            if (m_select_slot_label_id > 0) { 
                if(m_select_slot_label_id == rect.rectInfo.label) { 
                    // *it = rect;
                }
            }
            else {
                // *it = rect;
            } 

            if(rect.is_reliable == 1) {
                // rect_car_center = *it;
                rect_car_center.rectInfo.pt[0] = coordConvert_car_center(slot.a);
                rect_car_center.rectInfo.pt[1] = coordConvert_car_center(slot.b);
                rect_car_center.rectInfo.pt[2] = coordConvert_car_center(slot.c);
                rect_car_center.rectInfo.pt[3] = coordConvert_car_center(slot.d); 
                
                m_output_slot.slots_in_cur_frame.push_back(rect_car_center);
            }  
        }
    }
    delete_invalid_slots();
    collect_confirmed_slots(m_output_slot);

    // 检查状态2：update内部变量
    // LOGD("CHECK SIZE m_output_slot:%d, slots_map:%d",m_output_slot.slots_in_cur_frame.size(),slots_map_.size());
    
    auto end_update = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_update - start_update);
    std::cout<<"[TIMECOST]UPDATE time is:"<< elapsed.count() <<std::endl;
}

void PSD_FusionModuleIF::collect_confirmed_slots(apaSlotListInfo &slot_res){
    slot_res.slots_in_cur_frame.clear();
    slot_res.WorldoutRect.clear();
    apaSlotInfo rect_local, rect_world, rect_world_shrink;
    int slot_id = 1000;
    for (const auto &slot : slots_map_){
        auto corner_world = slot.second.get()->GetCornersWorld();
        
            rect_local.rectInfo.label = slot_id;
            rect_local.rectInfo.PStype = (int)slot.second.get()->GetSlotType();
            rect_local.rectInfo.iSodType = slot.second.get()->GetOccupy();
            rect_local.rectInfo.iMaterial = slot.second.get()->GetMaterial();
            rect_local.rectInfo.StopperDistance = slot.second.get()->stopper_distance_;
            rect_local.rectInfo.StopperLocation = slot.second.get()->stopper_location_;
            rect_local.rectInfo.LockInSlot = slot.second.get()->lock_in_slot_;
            rect_local.rectInfo.OBSInSlot = slot.second.get()->obs_in_slot_;

            rect_world.rectInfo.label = slot_id;
            rect_world.rectInfo.PStype = (int)slot.second.get()->GetSlotType();
            rect_world.rectInfo.iSodType = slot.second.get()->GetOccupy();
            rect_world.rectInfo.iMaterial = slot.second.get()->GetMaterial();
            rect_world.rectInfo.StopperDistance = slot.second.get()->stopper_distance_;
            rect_world.rectInfo.StopperLocation = slot.second.get()->stopper_location_;
            rect_world.rectInfo.LockInSlot = slot.second.get()->lock_in_slot_;
            rect_world.rectInfo.OBSInSlot = slot.second.get()->obs_in_slot_;


            for(int i = 0; i < 4; i++){
                if(corner_world[i].hasNaN()){
                    continue;    
                }else{
                    Eigen::Vector3f pt;
                    pt << corner_world[i].head<2>().x(), corner_world[i].head<2>().y(), 0.0;
                    world2car(pt);
                    // local
                    rect_local.rectInfo.pt[i].x = pt.x();
                    rect_local.rectInfo.pt[i].y = pt.y();
                    // world
                    rect_world.rectInfo.pt[i].x = corner_world[i].head<2>().x();
                    rect_world.rectInfo.pt[i].y = corner_world[i].head<2>().y();
                }
            }
            // LOGD("[SHRINK] before shrink: (%d,%d), (%d,%d), (%d, %d), (%d, %d)",rect_local.rectInfo.pt[0].x,
            //                                                            rect_local.rectInfo.pt[0].y,
            //                                                            rect_local.rectInfo.pt[1].x,
            //                                                            rect_local.rectInfo.pt[1].y,
            //                                                            rect_local.rectInfo.pt[2].x,
            //                                                            rect_local.rectInfo.pt[2].y,
            //                                                            rect_local.rectInfo.pt[3].x,
            //                                                            rect_local.rectInfo.pt[3].y)
            shrink_quad(rect_local); 
            // LOGD("[SHRINK] after shrink: (%d,%d), (%d,%d), (%d, %d), (%d, %d)",rect_local.rectInfo.pt[0].x,
            //                                                           rect_local.rectInfo.pt[0].y,
            //                                                           rect_local.rectInfo.pt[1].x,
            //                                                           rect_local.rectInfo.pt[1].y,
            //                                                           rect_local.rectInfo.pt[2].x,
            //                                                           rect_local.rectInfo.pt[2].y,
            //                                                           rect_local.rectInfo.pt[3].x,
            //                                                           rect_local.rectInfo.pt[3].y)
            // rect_world = shrink_quad(rect_world);
            
            slot_res.slots_in_cur_frame.push_back(rect_local);
            slot_res.WorldoutRect.push_back(rect_world);
            slot_id++;
    } 
}

void PSD_FusionModuleIF::world2car(Eigen::Vector3f &pt){
    const auto& yaw = m_output_slot.padRealTimeLocation.yaw;
    // const auto& yaw = m_vehicle_pose.yaw * M_PI / 180;

    std::cout<<"Algoyaw is:"<< yaw << std::endl;
    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);
    // 全局坐标中的点 pt (pt.x(), pt.y())
    // 车辆位置
    float Vx = m_vehicle_pose.coord.x;
    float Vy = m_vehicle_pose.coord.y;

    // 平移
    float dx = pt.x() - Vx;
    float dy = pt.y() - Vy;

    // 旋转（逆旋转yaw，将点从全局对齐到车体坐标）
    float local_x = dx * cos_yaw + dy * sin_yaw;
    float local_y = -dx * sin_yaw + dy * cos_yaw;
    pt << local_x, local_y, 0.0;
}

void PSD_FusionModuleIF::delete_invalid_slots() {
    // delete missing slots
    bool deleted = false;
    for (auto iter = slots_map_.begin(); iter != slots_map_.end();) {
        if (iter->second->IsToBeDeleted()) {
            
            iter = slots_map_.erase(iter);
            deleted = true;
        } else {
            ++iter;
        }
    }
    if (deleted) rebuild_slots_tree();

    // delete faraway slots
    if (slots_map_.size() < 2) return;
    auto tracking_indexes = slots_tree_->neighborhood_indices(
        point_t{m_vehicle_pose.coord.x, m_vehicle_pose.coord.y}, psmp_.neighborhood_range);

    deleted = false;
    for (const auto& remap : slots_remap_) {
        if (std::find(tracking_indexes.begin(), tracking_indexes.end(),
                      remap.first) == tracking_indexes.end()) {
            
            slots_map_.erase(remap.second);
            deleted = true;
        }
    }
    if (deleted) rebuild_slots_tree();
};


int PSD_FusionModuleIF::CalcDistance(POINT_I a, POINT_I b)
{
    float dis = sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
    return dis;
}


vector<apaSlotInfo>::iterator PSD_FusionModuleIF::existed_in_psinfo(const apaSlotInfo& rect_new, bool& mis_detect_flag)
{
    if(m_next_available_label_idx > 120) {
        printf("id=%d\n",m_frame_id); 
    }

    double iou = 0;

    if(m_apa_psinfo.WorldoutRect.size() == 0) {
        auto it = m_apa_psinfo.WorldoutRect.end();
        return it;
    }

    //size>0时，反向遍历psinfo
    //从倒数第一个向前遍历，假设新增的车位和上一个最匹配。和之前的所有车位对比
    auto it = m_apa_psinfo.WorldoutRect.end() - 1; 
    for( ; it >= m_apa_psinfo.WorldoutRect.begin(); it--) 
    {
        //把rect_new转换成Vertexes格式的vert_new
        Vertexes vert_new, vert;
        changePoint(rect_new, vert_new);
        changePoint(*it, vert);
        iou = iouEx(vert_new, vert); //当前帧和上一帧的车位IOU


        // // for test
        // iou = 0.0;
        //IOU>0.4重复，此时返回it是重叠的老车位
        if (iou >= 0.7) {
            break;
        }

        //0.2-0.4misdetect
        else if (iou >= 0.2 && iou < 0.7) {
            mis_detect_flag = false;
        }

        //<0.2认为没有相同车位继续循环
        
    }
    if(iou < 0.7) 
    {
        it = m_apa_psinfo.WorldoutRect.end(); //遍历完，没有找到重复车位
    }
    return it;
}

void PSD_FusionModuleIF::transform2world(
    const padVehiclePose& loc_pose,
    const ParkingSlotResultPtr& post_result,
    QuadInfoPtr& quad_info) {
    // quad_info->quads.resize(4, 3);
    quad_info->quads.bottomRows<1>().setOnes();

    // // 后处理处理好的车位角点，有顺序性
    quad_info->quads.col(0).head<2>() = post_result->tl;
    quad_info->quads.col(1).head<2>() = post_result->tr;
    quad_info->quads.col(2).head<2>() = post_result->br;
    quad_info->quads.col(3).head<2>() = post_result->bl;

    quad_info->center_pixel.setOnes();
    quad_info->center_pixel.head<2>() = post_result->center;
    float x = quad_info->center_pixel.x() - BIRD_VIEW_HEIGHT / 2;
    float y = BIRD_VIEW_HEIGHT/2.0 - quad_info->center_pixel.y();
 
    float temp_x = x * LR_BIRD_PIXECL_2_WORLD;
    float temp_y = y * LR_BIRD_PIXECL_2_WORLD;
    quad_info->center_ego << temp_x,temp_y, 0.0;
    // quad_info->center_ego = intrinsic_ipm2car_ * quad_info->center_pixel;
    // quad_info->long_dir_pixel = post_result->long_direction;
    // quad_info->wide_dir_pixel = post_result->wide_direction;
    if (post_result->type == 1){
        quad_info->long_dir_pixel = post_result->wide_direction;
        quad_info->wide_dir_pixel = post_result->long_direction;
    }else{
        quad_info->long_dir_pixel = post_result->long_direction;
        quad_info->wide_dir_pixel = post_result->wide_direction;
    }
    quad_info->length_world = post_result->length * std::fabs(intrinsic_ipm2car_(0, 1));
    quad_info->width_world = post_result->width * std::fabs(intrinsic_ipm2car_(1, 0));

    transform2world(loc_pose, quad_info);
};

void PSD_FusionModuleIF::transform2world(const padVehiclePose& loc_pose,
                                         QuadInfoPtr& quad_info) {
    const float& min_u = ipmp_.min_u;
    const float& max_u = ipmp_.max_u;
    const float& min_v = ipmp_.min_v;
    const float& max_v = ipmp_.max_v;
    const float& cam_v = ipmp_.cam_v;
    const float& edge_thr = ipmp_.edge_thr;

    for (int i = 0; i < quad_info->quads.cols(); ++i) {
        const auto& quad = quad_info->quads.col(i);
        // 判断是否IPM边缘点，先重置为初始值，避免查重的结果影响
        quad_info->near_edge.at(i) = false;
        if (quad.x() < edge_thr || quad.y() < edge_thr ||
            quad.x() + edge_thr > ipmp_.ipm_width ||
            quad.y() + edge_thr > ipmp_.ipm_height) {
            quad_info->near_edge.at(i) = true;
            
        }

        // 设置协方差
        Eigen::Vector2f cam_pixel;
        if (quad.x() < min_u) {  // left
            cam_pixel = Eigen::Vector2f(min_u + 1.f, cam_v);
        } else if (quad.x() > max_u) {  // right
            cam_pixel = Eigen::Vector2f(max_u - 1.f, cam_v);
        } else if (quad.y() < min_v) {  // top
            cam_pixel = Eigen::Vector2f(ipmp_.ipm_width * 0.5f, min_v + 1.f);
        } else if (quad.y() > max_v) {  // bottom
            cam_pixel = Eigen::Vector2f(ipmp_.ipm_width * 0.5f, max_v - 1.f);
        } else {
            cam_pixel = Eigen::Vector2f(ipmp_.ipm_width * 0.5f, cam_v);
        };

        Eigen::Vector2f pixel_vec = quad.head<2>() - cam_pixel;
        float pixel_dist = pixel_vec.norm();
        Eigen::Vector2f v_1_pixel = pixel_vec / pixel_dist;  // 像素坐标系
        Eigen::Vector2f v_1 = {-v_1_pixel.y(), -v_1_pixel.x()};  // 自车系
        Eigen::Vector2f v_2 = {v_1_pixel.x(),
                               -v_1_pixel.y()};  // {-v_1(1), v_1(0)}
        float sigma_1 = std::max(psmp_.sigma_1_ratio * pixel_dist,
                                 psmp_.sigma_1_ratio * 10.0f) *
                        intrinsic_ipm2car_(0, 1);
        float sigma_2 = std::max(psmp_.sigma_2_ratio * pixel_dist,
                                 psmp_.sigma_2_ratio * 10.0f) *
                        intrinsic_ipm2car_(0, 1);

       
        if (quad_info->near_edge.at(i) || pixel_dist > psmp_.pixel_dist_thr) {
            sigma_1 *= pixel_dist / psmp_.sigma_enlarge_coeff;
            sigma_2 *= pixel_dist / psmp_.sigma_enlarge_coeff;
        }
        Eigen::Matrix2f V;
        V.col(0) = v_1;
        V.col(1) = v_2;
        Eigen::Vector2f lambda;
        lambda(0) = sigma_1 * sigma_1;
        lambda(1) = sigma_2 * sigma_2;
        quad_info->cov_ego.at(i) = V * lambda.asDiagonal() * V.transpose();
        
    }

    // 转换坐标系
    quad_info->corners_ego = intrinsic_ipm2car_ * quad_info->quads;
    const auto& yaw = - loc_pose.yaw * PI / 180.0;
    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);
    Eigen::Matrix3f trans_matrix;
    trans_matrix << cos_yaw, -sin_yaw, loc_pose.coord.x, sin_yaw, cos_yaw,
        loc_pose.coord.y, 0, 0, 1;
    // quad_info->corners_world = trans_matrix * quad_info->corners_ego;
    // 协方差转换到世界坐标系
    Eigen::Matrix2f Rotation = trans_matrix.topLeftCorner<2, 2>();
    for (size_t i = 0; i < quad_info->quads.cols(); ++i) {
        quad_info->cov_world.at(i) =
            Rotation * quad_info->cov_ego.at(i) * Rotation.transpose();
       
    };

    float global_x, global_y, local_x, local_y;
    for (int i = 0; i < quad_info->quads.cols(); ++i) {
        const auto& quad = quad_info->quads.col(i);
        float x = (quad.x() - BIRD_VIEW_HEIGHT / 2) * LR_BIRD_PIXECL_2_WORLD;
        float y = (BIRD_VIEW_HEIGHT/2.0 + (VEHICLE_LENGTH/2.0-REAR_AXLE_CENTER_VEHICLE_REAR)/LR_BIRD_PIXECL_2_WORLD - quad.y()) * LR_BIRD_PIXECL_2_WORLD;
        float yaw = loc_pose.yaw * PI / 180.0;   
        
        global_x = x * cos(yaw) + y * sin(yaw) + loc_pose.coord.x;
        global_y = y * cos(yaw) - x * sin(yaw) + loc_pose.coord.y;

        quad_info->corners_world.col(i)<<global_x, global_y, 0.0;
    }
    
    if (quad_info->center_ego != Eigen::Vector3f::Zero()) {
        Eigen::Vector3f center_sum = quad_info->corners_world.rowwise().sum(); // 对每一行（即 x、y、z 坐标）进行求和，得到总和向量
        Eigen::Vector3f center = center_sum / quad_info->corners_world.cols(); // 将总和向量除以列数（角点数量），得到中心点的坐标 center
        quad_info->center_world = center;
        // quad_info->center_world = trans_matrix * quad_info->center_ego;
        quad_info->long_dir_world = trans_matrix.topLeftCorner<2, 2>() *
                                    (intrinsic_ipm2car_.topLeftCorner<2, 2>() *
                                     quad_info->long_dir_pixel)
                                        .normalized();
        quad_info->wide_dir_world = trans_matrix.topLeftCorner<2, 2>() *
                                    (intrinsic_ipm2car_.topLeftCorner<2, 2>() *
                                     quad_info->wide_dir_pixel)
                                        .normalized();
    }
    
};

void PSD_FusionModuleIF::rebuild_slots_tree() {
    if (slots_map_.empty()) return;
    std::vector<point_t> slots_vec;
    slots_vec.reserve(slots_map_.size());
    slots_remap_.clear();

    for (const auto& slot : slots_map_) {
        slots_remap_[slots_vec.size()] = slot.first;
        slots_vec.emplace_back(point_t{slot.second->GetSlotCenter().x(),
                                       slot.second->GetSlotCenter().y()});
    }

    slots_tree_ = std::make_shared<KDTree>(KDTree(slots_vec));
    
};

static inline float Cosine(const Eigen::Vector2f &l, const Eigen::Vector2f &r) {
    float cos_theta = l.normalized().dot(r.normalized());
    return cos_theta;
}

bool PSD_FusionModuleIF::ProcessParkingSlotResult(const PSMaskU8 &parking_line_mask, const padVisionSlotCoord &bbox, ParkingSlotResultPtr &result){
    // if (bbox->confidence < 0.25) {
    //     return false;
    // }
    auto complete_quad = std::make_shared<ParkingSlotQuad>();
    if (!CalibrateSingleSlot(bbox, parking_line_mask ,*complete_quad)){
        return false;
    }

    if (FillSingleSlot(*complete_quad)) {
        result->tl = complete_quad->tl;
        result->tr = complete_quad->tr;
        result->bl = complete_quad->bl;
        result->br = complete_quad->br;
        result->ori_tl = {complete_quad->ori_tl(0), complete_quad->ori_tl(1)};
        result->ori_tr = {complete_quad->ori_tr(0), complete_quad->ori_tr(1)};
        result->ori_bl = {complete_quad->ori_bl(0), complete_quad->ori_bl(1)};
        result->ori_br = {complete_quad->ori_br(0), complete_quad->ori_br(1)};
        result->confidence = complete_quad->confidence;
        result->is_occupied = (complete_quad->label == 1);
        result->width = complete_quad->width;
        result->length = complete_quad->length;
        result->center = {complete_quad->center(0), complete_quad->center(1)};
        result->wide_direction = complete_quad->dir_width;
        result->long_direction = complete_quad->dir_length;
        result->type = complete_quad->slot_type;

        return true;
    }
    return false;
}

static inline void GetGravityCenter(ParkingSlotQuad &quad) {
    float area = 0.F;
    float x = 0.F, y = 0.F;

    std::vector<Eigen::Vector2f> points = {quad.tl, quad.tr, quad.br, quad.bl};
    for (int i = 0, i1 = 3; i < 4; i++, i1 = (i - 1) % 4) {
        float fg =
            (points[i](0) * points[i1](1) - points[i](1) * points[i1](0)) *
            0.5F;
        area += fg;
        x += fg * (points[i](0) + points[i1](0)) / 3.0F;
        y += fg * (points[i](1) + points[i1](1)) / 3.0F;
    }
    x /= area;
    y /= area;
    quad.center = {x, y};
}

bool PSD_FusionModuleIF::FillSingleSlot(ParkingSlotQuad &approx_quad) {
    ObtainDirection(approx_quad);
    if (approx_quad.filtered) {
        return false;
    }

    float cos_wl =
        std::abs(Cosine(approx_quad.dir_width, approx_quad.dir_length));
    if (param_.ps_score_range.InRange(cos_wl) && approx_quad.valid_slant) {
        approx_quad.slot_type = SLANT;
    } else {
        approx_quad.slot_type = -1;
    }
    if (!param_.ps_width_range.InRange(approx_quad.p_tl->line_len) &&
        param_.ps_width_slant_range.InRange(approx_quad.p_tl->line_len) &&
        approx_quad.slot_type != SLANT) {
        
        return false;
    }
    if (approx_quad.slot_type == SLANT && !approx_quad.valid_slant) {
        
        return false;
    }
    ObtainSlotType(approx_quad);

    CompleteBoxWithArrowFix(approx_quad);
    if (approx_quad.filtered) {
        return false;
    }

    GetGravityCenter(approx_quad);

    if (approx_quad.slot_type != SLANT) {
        if (approx_quad.width > param_.vp_MaxW) {
            return false;
        }
    }

    ModifyDirIn(approx_quad);

    return true;
}

void PSD_FusionModuleIF::ObtainDirection(ParkingSlotQuad &quad) {
    Eigen::Vector2f dir1 = {floor((quad.tr(0) - quad.br(0)) / 2),
                  floor((quad.tr(1) - quad.br(1)) / 2)};
    dir1 = dir1.normalized();
    Eigen::Vector2f dir2 = {floor((quad.tr(0) - quad.tl(0)) / 2),
                  floor((quad.tr(1) - quad.tl(1)) / 2)};
    dir2 = dir2.normalized();
    Eigen::Vector2f dir3 = {floor((quad.tl(0) - quad.bl(0)) / 2),
                  floor((quad.tl(1) - quad.bl(1)) / 2)};
    dir3 = dir3.normalized();
    float cosine = std::abs(Cosine(dir1, dir3));
    float cosine1 = std::abs(Cosine(dir1, dir2));
    float cosine3 = std::abs(Cosine(dir3, dir2));
    float opplen = quad.p_tl->line_len;

    float min_cosine;
    Eigen::Vector2f min_dir;
    // if (cosine1 <= cosine3) {
    //      min_cosine = cosine1;
    //      min_dir = dir1;
    //  } else {
    //      min_cosine = cosine3;
    //      min_dir = dir3;
    // }
    float p1_len = quad.p_tr->line_len;
    float p4_len = quad.p_bl->line_len;
    float border_point_dis_thr = param_.border_point_dis_thr;
    if (quad.p_tr->border_dist > border_point_dis_thr &&
        quad.p_bl->border_dist > border_point_dis_thr) {
        if (p1_len > p4_len) {
            min_cosine = cosine1;
            min_dir = dir1;
        } else {
            min_cosine = cosine3;
            min_dir = dir3;
        }
    } else if (quad.p_tr->border_dist > border_point_dis_thr) {
        min_cosine = cosine1;
        min_dir = dir1;
    } else if (quad.p_bl->border_dist > border_point_dis_thr) {
        min_cosine = cosine3;
        min_dir = dir3;
    } else {
        if (p1_len > p4_len) {
            min_cosine = cosine1;
            min_dir = dir1;
        } else {
            min_cosine = cosine3;
            min_dir = dir3;
        }
    }

    float opp_score = quad.p_tl->line_score;
    // FloatNumEqual(quad.p_tl->line_score, 1.0F)
    //     ? 1.0F
    //     : std::min(std::min(quad.s_tl, quad.s_tr),
    //     quad.p_tl->line_score);
    float line1_score = quad.p_tr->line_score;
    // FloatNumEqual(quad.p_tr->line_score, 1.0F)
    //     ? 1.0F
    //     : std::min(std::min(quad.s_tr, quad.s_br),
    //     quad.p_tr->line_score);
    float line2_score = quad.p_bl->line_score;
    // FloatNumEqual(quad.p_bl->line_score, 1.0F)
    //     ? 1.0F
    //     : std::min(std::min(quad.s_bl, quad.s_tl),
    //     quad.p_bl->line_score);

    if (param_.ps_length_range.InRange(opplen)) {
        if (opp_score < param_.direction_score_thr1) {
            if (line1_score > line2_score &&
                line1_score > param_.direction_score_thr2 &&
                cosine1 < param_.ps_score_range.low) {
                quad.dir_width = dir1;
                quad.dir_length = {dir1(1), -dir1(0)};
                quad.opp_modify = true;
                return;
            }
            if (line2_score > line1_score &&
                line2_score > param_.direction_score_thr2 &&
                cosine3 < param_.ps_score_range.low) {
                quad.dir_width = dir3;
                quad.dir_length = {dir3(1), -dir3(0)};
                quad.opp_modify = true;
                return;
            }
        }
        quad.dir_length = dir2;
        if (cosine > 0.994F || cosine < -0.994F) {
            // parallel
            if (line1_score > param_.direction_score_thr2 &&
                line2_score < param_.direction_score_thr1 &&
                (cosine1 < 0.05F || param_.ps_score_range.InRange(cosine1))) {
                quad.dir_width = dir1;
                quad.opp_modify = false;
                return;
            }
            if (line2_score > param_.direction_score_thr2 &&
                line1_score < param_.direction_score_thr1 &&
                (cosine3 < 0.05F || param_.ps_score_range.InRange(cosine3))) {
                quad.dir_width = dir3;
                quad.opp_modify = false;
                return;
            }
            // if (std::abs(cosine1) < 0.05F && std::abs(cosine3) < 0.05F) {
            //     quad.dir_width = (dir1 + dir3) * 0.5F;
            //     quad.opp_modify = false;
            //     return;
            // }
            if (min_cosine < 0.05F) {
                quad.dir_width = min_dir;
                quad.opp_modify = false;
                return;
            }
            if (min_cosine > param_.ps_score_range.high) {
                quad.filtered = true;
                return;
            }
            quad.dir_width = (dir1 + dir3) * 0.5F;
            return;
        } else {
            if (min_cosine < 0.05F or
                param_.ps_score_range.InRange(min_cosine)) {
                quad.dir_width = min_dir;
                quad.opp_modify = false;
                return;
            }
            quad.dir_width = {dir2(1), -dir2(0)};
            quad.opp_modify = false;
            return;
        }
    } else if (param_.ps_width_range.InRange(opplen) ||
               param_.ps_width_slant_range.InRange(opplen)) {
        if (opp_score < param_.direction_score_thr1) {
            if (line1_score > line2_score &&
                line1_score > param_.direction_score_thr2 &&
                cosine1 < param_.ps_score_range.low) {
                quad.dir_length = dir1;
                quad.dir_width = {dir1(1), -dir1(0)};
                quad.opp_modify = true;
                return;
            }
            if (line2_score > line1_score &&
                line2_score > param_.direction_score_thr2 &&
                cosine3 < param_.ps_score_range.low) {
                quad.dir_length = dir3;
                quad.dir_width = {dir3(1), -dir3(0)};
                quad.opp_modify = true;
                return;
            }
        }
        quad.dir_width = -dir2;
        if (cosine > 0.994F || cosine < -0.994F) {
            // parallel
            if (line1_score > param_.direction_score_thr2 &&
                line2_score < param_.direction_score_thr1 &&
                (cosine1 < 0.05F || param_.ps_score_range.InRange(cosine1))) {
                quad.dir_length = dir1;
                quad.opp_modify = false;
                return;
            }
            if (line2_score > param_.direction_score_thr2 &&
                line1_score < param_.direction_score_thr1 &&
                (cosine3 < 0.05F || param_.ps_score_range.InRange(cosine3))) {
                quad.dir_length = dir3;
                quad.opp_modify = false;
                return;
            }
            // if (std::abs(cosine1) < 0.05F && std::abs(cosine3) < 0.05F) {
            //     quad.dir_length = (dir1 + dir3) * 0.5F;
            //     quad.opp_modify = false;
            //     return;
            // }
            if (min_cosine < 0.05F) {
                quad.dir_length = min_dir;
                quad.opp_modify = false;
                return;
            }
            if (min_cosine > param_.ps_score_range.high) {
                quad.filtered = true;
                return;
            }
            quad.dir_length = (dir1 + dir3) * 0.5F;
            return;
        } else {
            if (min_cosine < 0.05F or
                param_.ps_score_range.InRange(min_cosine)) {
                quad.dir_length = min_dir;
                quad.opp_modify = false;
                return;
            }
            quad.dir_length = {dir2(1), -dir2(0)};
            quad.opp_modify = false;
            return;
        }
    } else {
        // Checked before
        quad.filtered = true;
        return;
    }
}

void PSD_FusionModuleIF::ObtainSlotType(ParkingSlotQuad &quad) {
    if (quad.dir_length.norm() > FLT_EPSILON &&
        quad.dir_width.norm() > FLT_EPSILON) {
        double cosine_len = Cosine(quad.dir_length, Eigen::Vector2f(0.0F, 1.0F));  //车位长度方向与垂直方向(0,1)之间的余铉值
        double cos_wid_len{Cosine(quad.dir_length, quad.dir_width)};
        
        // static const double COS_75{0.259};
        // static const double COS_15{0.966};
        static const double COS_60{0.5};
        static const double COS_30{0.866};
        if (fabs(cos_wid_len) > COS_60 && fabs(cos_wid_len) < COS_30) {
            quad.slot_type = SLANT;
           
        } else {
            static const double parallel_cos_threshold{0.8};
            if (fabs(cosine_len) > parallel_cos_threshold) {
                quad.slot_type = PARALLEL;
               
            } else {
                quad.slot_type = VERTICAL;
                
            }
        }
    } else {
        quad.slot_type = -1;
    }
}

static inline void PointRectify(const Eigen::Vector2f &dir,
                                const float len,
                                const Eigen::Vector2f &point_base,
                                Eigen::Vector2f &point_dst) {
    Eigen::Vector2f dir_ori = point_base - point_dst;
    Eigen::Vector2f dir_new = dir;
    if ((dir_ori(0) * dir(0) + dir_ori(1) * dir(1)) < 0) {
        dir_new = -dir_new;
    }

    point_dst(0) = point_base(0) - dir_new(0) * len;
    point_dst(1) = point_base(1) - dir_new(1) * len;
}

static inline void OpplineRectify(const Eigen::Vector2f &dir,
                                  const float len,
                                  ParkingSlotQuad &quad) {
    if (quad.s_tl > quad.s_tr) {
        // rectify tr
        PointRectify(dir, len, quad.tl, quad.tr);
    } else if (quad.s_tl < quad.s_tr) {
        // rectify tl
        PointRectify(dir, len, quad.tr, quad.tl);
    }
}

static inline float CrossProduct(const Eigen::Vector2f &l, const Eigen::Vector2f &r) {
    return l(0) * r(1) - l(1) * r(0);
}

static inline bool IsNearEdge(
    ParkingSlotQuad &quad, const ParkingSlotParam param_) {
    if (quad.slot_type == PARALLEL) {
        ParkingSlotRect rect = quad.BoundingRect();
        if (rect.top() < 5 || rect.bottom() > param_.input_h - 5) {
            return true;
        }
    }

    return false;
}

void PSD_FusionModuleIF::CompleteBoxWithArrowFix(ParkingSlotQuad &quad) {
    float line1_score = quad.p_tr->line_score;
    // FloatNumEqual(quad.p_tr->line_score, 1.0F)
    //     ? 1.0F
    //     : std::min(std::min(quad.s_tr, quad.s_br),
    //     quad.p_tr->line_score);
    float line2_score = quad.p_bl->line_score;
    // FloatNumEqual(quad.p_bl->line_score, 1.0F)
    //     ? 1.0F
    //     : std::min(std::min(quad.s_bl, quad.s_tl),
    //     quad.p_bl->line_score);
    float MaxH, MinH;
    std::vector<std::pair<float, float>> ps_sizes;
    int slot_type_map_slam;
    if (quad.map_slot_type != -1) {
        slot_type_map_slam = quad.map_slot_type;
        
    } else {
        slot_type_map_slam = quad.slot_type;
    }
    
    if (slot_type_map_slam == 2) {
        ps_sizes = param_.ps_size_controller.slant_slot_size;
        MaxH = param_.slant_MaxH;
        MinH = param_.slant_MinH;
        quad.length = quad.slant_length;
    } else if (slot_type_map_slam == 1) {
        ps_sizes = param_.ps_size_controller.p_slot_sizes;
        MaxH = param_.p_MaxH;
        MinH = param_.p_MinH;
    } else {
        ps_sizes = param_.ps_size_controller.v_slot_sizes;
        MaxH = param_.v_MaxH;
        MinH = param_.v_MinH;
    }
    if (param_.ps_width_range.InRange(quad.p_tl->line_len) ||
        param_.ps_width_slant_range.InRange(quad.p_tl->line_len)) {
        if (quad.is_complete) {
            quad.width = quad.p_tl->line_len;
            if (line1_score >= param_.direction_score_thr1 &&
                line2_score < param_.direction_score_thr1) {
                quad.length = quad.p_tr->line_len;
            } else if (line2_score >= param_.direction_score_thr1 &&
                       line1_score < param_.direction_score_thr1) {
                quad.length = quad.p_bl->line_len;
            } else if (line1_score >= param_.direction_score_thr1 &&
                       line2_score >= param_.direction_score_thr1) {
                quad.length =
                    (quad.p_tr->line_len + quad.p_bl->line_len) * 0.5F;
            } else if (quad.length <= 1e-7) {
                param_.ps_size_controller.Adjust(quad.length, quad.width, false,
                                                 ps_sizes, false);
            }
        } else {
            if (quad.length > 1e-7) {
                if (quad.opp_modify) {
                    param_.ps_size_controller.Adjust(quad.length, quad.width,
                                                     true, ps_sizes, false);
                } else {
                    quad.width = quad.p_tl->line_len;
                }
            } else {
                quad.width = quad.p_tl->line_len;
                param_.ps_size_controller.Adjust(quad.length, quad.width, false,
                                                 ps_sizes, true);
            }
            if (!quad.opp_modify) {
                quad.width = quad.p_tl->line_len;
            }
        }
        quad.length = std::min(quad.length, MaxH);
        if (quad.opp_modify) {
            OpplineRectify(quad.dir_width, quad.width, quad);
        }

        PointRectify(quad.dir_length, quad.length, quad.tl, quad.bl);
        PointRectify(quad.dir_length, quad.length, quad.tr, quad.br);
        if (IsNearEdge(quad, param_)) {
            quad.length = std::max(quad.length, MinH);
            PointRectify(quad.dir_length, quad.length, quad.tl, quad.bl);
            PointRectify(quad.dir_length, quad.length, quad.tr, quad.br);
        }
        
    } else if (param_.ps_length_range.InRange(quad.p_tl->line_len)) {
        if (quad.is_complete) {
            quad.length = quad.p_tl->line_len;
            if (line1_score >= param_.direction_score_thr1 &&
                line2_score < param_.direction_score_thr1) {
                quad.width = quad.p_tr->line_len;
            } else if (line2_score >= param_.direction_score_thr1 &&
                       line1_score < param_.direction_score_thr1) {
                quad.width = quad.p_bl->line_len;
            } else if (line1_score >= param_.direction_score_thr1 &&
                       line2_score >= param_.direction_score_thr1) {
                quad.width = (quad.p_tr->line_len + quad.p_bl->line_len) * 0.5F;
            } else {
                param_.ps_size_controller.Adjust(quad.length, quad.width, true,
                                                 ps_sizes, false);
            }
        } else {
            quad.length = quad.p_tl->line_len;
            param_.ps_size_controller.Adjust(quad.length, quad.width, true,
                                             ps_sizes, true);
            if (!quad.opp_modify) {
                quad.length = quad.p_tl->line_len;
            }
        }
        if (quad.length > MaxH) {
            quad.length = MaxH;
            quad.opp_modify = true;
        }
        if (quad.opp_modify) {
            OpplineRectify(quad.dir_length, quad.length, quad);
        }
        PointRectify(quad.dir_width, quad.width, quad.tl, quad.bl);
        PointRectify(quad.dir_width, quad.width, quad.tr, quad.br);
        if (IsNearEdge(quad, param_)) {
            if (quad.length < MinH) {
                quad.length = MinH;
                quad.opp_modify = true;
            }
            if (quad.opp_modify) {
                OpplineRectify(quad.dir_length, quad.length, quad);
            }
            PointRectify(quad.dir_width, quad.width, quad.tl, quad.bl);
            PointRectify(quad.dir_width, quad.width, quad.tr, quad.br);
        }
        
    } else {
        
        quad.filtered = true;
        return;
    }

}

void PSD_FusionModuleIF::ModifyDirIn(ParkingSlotQuad &quad) {
    if (quad.map_slot_type == PARALLEL ||
        (quad.map_slot_type == -1 && quad.slot_type == PARALLEL)) {
        Eigen::Vector2f tmp = quad.dir_length;
        quad.dir_length = quad.dir_width;
        quad.dir_width = tmp;
        float tmp_len = quad.length;
        quad.length = quad.width;
        quad.width = tmp_len;
    }

    // check direction of entrance
    quad.dir_in = {param_.input_w / 2 - quad.center(0),
                   param_.input_h / 3 - quad.center(1)};
    double cosine_width = Cosine(quad.dir_width, quad.dir_in);
    double cosine_length = Cosine(quad.dir_length, quad.dir_in);
    if (cosine_length < 0) {
        quad.dir_length = -quad.dir_length;
        quad.dir_width = -quad.dir_width;
    }

    // check clockwise
    float cross_between =
        CrossProduct(quad.dir_length.normalized(), quad.dir_width.normalized());
    
    // quad.dir_length.normalized().cross(quad.dir_width.normalized());
    if (cross_between > 0) {
        quad.dir_width = -quad.dir_width;
    }

    return;
}

bool ParkingSlotSizeController::Adjust(
    float &length,
    float &width,
    bool base_on_length,
    std::vector<std::pair<float, float>> ps_sizes,
    bool adjust_both) {
    float l = length;
    float w = width;
    int len = ps_sizes.size();
    w = ps_sizes[len - 1].first;
    l = ps_sizes[len - 1].second;
    if (base_on_length) {
        for (int i = 0; i < len - 1; i++) {
            if (length <= ps_sizes[i].second) {
                w = ps_sizes[i].first;
                l = ps_sizes[i].second;
                break;
            }
        }
        width = w;
        if (adjust_both) {
            length = l;
        }
    } else {
        for (int i = 0; i < len - 1; i++) {
            if (width <= ps_sizes[i].first) {
                l = ps_sizes[i].second;
                w = ps_sizes[i].first;
                break;
            }
        }
        length = l;
        if (adjust_both) {
            width = w;
        }
    }
    return true;
}

template <typename T>
static inline T min4(T v1, T v2, T v3, T v4) {
    return std::min(v1, std::min(v2, std::min(v3, v4)));
}


static inline float CheckBorder(const float x,
                                const float y,
                                const float x_max,
                                const float y_max) {
    return std::max(0.F, min4(x, y, x_max - x, y_max - y));
}

bool PSD_FusionModuleIF::CheckComplete(
    const std::vector<ApproxBoxPoints> &points) {
    int w_cnt = 0, l_cnt = 0;
    for (size_t i = 0; i < points.size(); i++) {
        size_t i1 = (i + 1) % 4;
        if (param_.ps_length_range.InRange(points[i].line_len)) {
            l_cnt++;
        } else if (param_.ps_width_range.InRange(points[i].line_len) ||
                   param_.ps_width_slant_range.InRange(points[i].line_len)) {
            w_cnt++;
        }
    }

    return w_cnt == 2 && l_cnt == 2;
}

bool PSD_FusionModuleIF::ApproxBox(const PSMaskU8 &mask,
                                  ParkingSlotQuad &approx) {
    std::vector<ApproxBoxPoints> points(4);
    points[0].p = approx.tl;
    points[0].point_score = approx.s_tl;
    points[1].p = approx.tr;
    points[1].point_score = approx.s_tr;
    points[2].p = approx.br;
    points[2].point_score = approx.s_br;
    points[3].p = approx.bl;
    points[3].point_score = approx.s_bl;

    bool is_complete = false;
    int faraway_line = 0;
    float line_score_sum = 0.0;
    // find line closest to border

    float min_border_dis = std::numeric_limits<float>::max();
    int min_border_idx = -1;

    // distance between each point and image border
    for (int i = 0; i < 4; i++) {
        points[i].border_dist =
            CheckBorder(points[i].p(0), points[i].p(1), param_.input_w - 1,
                        param_.input_h - 1);

    }

    for (int i = 0, i1 = 1; i < 4; i++, i1 = (i + 1) % 4) {
        if (points[i].border_dist <= 3 || points[i1].border_dist <= 3) {
            points[i].has_border_point = true;
        }
    }

    bool has_border = false;
    // 判断边的中点是否靠近图像边缘
    for (int i = 0, i1 = 1; i < 4; i++, i1 = (i + 1) % 4) {
        Eigen::Vector2f point_mid = (points[i].p + points[i1].p) * 0.5F;
        const float border_dis = CheckBorder(
            static_cast<int>(point_mid(0)), static_cast<int>(point_mid(1)),
            param_.input_w - 1, param_.input_h - 1);
        faraway_line += border_dis > param_.line_border_dist_thres;

        // find minimum
        if (i == 0) {
            min_border_idx = i;
            min_border_dis = border_dis;
            has_border = points[(i + 2) % 4].has_border_point;
            continue;
        }
        if (has_border && !points[(i + 2) % 4].has_border_point) {
            min_border_idx = i;
            min_border_dis = border_dis;
            has_border = false;
        }
        if (border_dis < min_border_dis &&
            (has_border == points[(i + 2) % 4].has_border_point)) {
            min_border_idx = i;
            min_border_dis = border_dis;
        }
    }
    // 判断起点、中点、终点是否都在分割的车位线上，有一个在，+1分；
    for (int i = 0, i1 = 1; i < 4; i++, i1 = (i + 1) % 4) {
        Eigen::Vector2f point_mid = (points[i].p + points[i1].p) * 0.5F;
        float &score = points[i].line_score;
        score = 0.F;
        if (mask.At(points[i].p(0), points[i].p(1), 0) != 0) {
            score += 1.F;
        }
        if (mask.At(points[i1].p(0), points[i1].p(1), 0) != 0) {
            score += 1.F;
        }
        if (point_mid[0] >= 0 && point_mid[0] < param_.input_w &&
            point_mid[1] >= 0 && point_mid[1] < param_.input_h &&
            mask.At(point_mid(0), point_mid(1), 0) != 0) {
            score += 1.F;
        }
        score /= 3.F;
        score = FloatNumEqual(score, 1.0F)
                    ? 1.0F
                    : std::min(std::min(points[i].point_score,
                                        points[i1].point_score),
                               score);
        line_score_sum += score;
        Eigen::Vector2f line = {points[i].p(0) - points[i1].p(0),
                      points[i].p(1) - points[i1].p(1)};
        points[i].line_len = line.norm();

       
    }

    // 过滤车位结果，至少要有两条远离图像边缘的车位线
    if ((faraway_line < 3 && line_score_sum < 2) || faraway_line < 2) {
       
        approx.filtered = true;
        return false;
    }

    // obtain dir_in
    const int p_idx = min_border_idx;
    const int p1_idx = (min_border_idx + 1) % 4;
    const int po_idx = (min_border_idx + 2) % 4;
    const int p2_idx = (min_border_idx + 3) % 4;
    Eigen::Vector2f p_tl_in = points[po_idx].p;
    Eigen::Vector2f p_tr_in = points[p2_idx].p;
    Eigen::Vector2f p_br_in = points[p_idx].p;
    Eigen::Vector2f p_bl_in = points[p1_idx].p;
    Eigen::Vector2f dir1_in = (p_tr_in - p_br_in).normalized();  // -line1_dir
    Eigen::Vector2f dir3_in = (p_tl_in - p_bl_in).normalized();  // line2_dir
    Eigen::Vector2f dir_in = (dir1_in + dir3_in) * 0.5F;

    // check if complete
    if (faraway_line >= 4 && CheckComplete(points)) {
        is_complete = true;
    }

    float max_score = std::numeric_limits<float>::lowest();
    int max_score_idx = -1;
    // judge baseline based on score
    for (int i = 0, i1 = 1; i < 4; i++, i1 = (i + 1) % 4) {
        const float score = (points[i].point_score + points[i1].point_score +
                             points[i].line_score) *
                            0.5F;

        if (FloatNumEqual(score, 1.F)) {
            max_score = 1.F;
            max_score_idx = i;
            break;
        }

        if (score > max_score) {
            max_score = score;
            max_score_idx = i;
        }
    }
    if (is_complete ||
        !(max_score < 1 || points[max_score_idx].has_border_point)) {
        min_border_idx = (max_score_idx + 2) % 4;
    }
    const int lineborder_idx = min_border_idx;
    const int line1_idx = (min_border_idx + 1) % 4;
    const int lineopp_idx = (min_border_idx + 2) % 4;
    const int line2_idx = (min_border_idx + 3) % 4;
    const auto &p = points[lineborder_idx];
    const auto &p1 = points[line1_idx];
    const auto &po = points[lineopp_idx];
    const auto &p2 = points[line2_idx];

    float ret_len = 0.F;
    float ret_slant_len = 0.F;
    if (is_complete || param_.ps_width_range.InRange(po.line_len) ||
        param_.ps_width_slant_range.InRange(po.line_len)) {
        int ret_len_size = 0;
        if (param_.ps_length_complete_range.InRange(p1.line_len) ||
            (param_.ps_length_2_range.InRange(p1.line_len) &&
             p2.border_dist > param_.point_border_dist_complete_h2 &&
             po.border_dist > param_.point_border_dist_complete_h2)) {
            ret_len += p1.line_len;
            ret_len_size++;
        }
        if (param_.ps_length_complete_range.InRange(p2.line_len) ||
            (param_.ps_length_2_range.InRange(p2.line_len) &&
             p2.border_dist > param_.point_border_dist_complete_h2 &&
             p.border_dist > param_.point_border_dist_complete_h2)) {
            ret_len += p2.line_len;
            ret_len_size++;
        }
        if (ret_len_size) {
            ret_len /= ret_len_size;
        }
    }
    if (is_complete || param_.ps_width_range.InRange(po.line_len) ||
        param_.ps_width_slant_range.InRange(po.line_len)) {
        int ret_len_size = 0;
        if (param_.ps_length_slant_complete_range.InRange(p1.line_len) ||
            (param_.ps_length_slant_2_range.InRange(p1.line_len) &&
             p2.border_dist > param_.point_border_dist_complete_h2 &&
             po.border_dist > param_.point_border_dist_complete_h2)) {
            ret_slant_len += p1.line_len;
            ret_len_size++;
        }
        if (param_.ps_length_slant_complete_range.InRange(p2.line_len) ||
            (param_.ps_length_slant_2_range.InRange(p2.line_len) &&
             p2.border_dist > param_.point_border_dist_complete_h2 &&
             p.border_dist > param_.point_border_dist_complete_h2)) {
            ret_slant_len += p2.line_len;
            ret_len_size++;
        }
        if (ret_len_size) {
            ret_slant_len /= ret_len_size;
        }
    }

    if (!param_.ps_width_range.InRange(po.line_len) &&
        !param_.ps_length_range.InRange(po.line_len) &&
        !param_.ps_width_slant_range.InRange(po.line_len)) {
        
        return false;
    }
    if (param_.ps_width_slant_range.InRange(po.line_len)) {
       
    }

    // check_bbox
    if (p1.line_len <= 1e-7 || p2.line_len <= 1e-7 || po.line_len <= 1e-7) {
       
        return false;
    }
    // the opposite line should be entire line, put it into tl-tr.
    // others points put in clock-wise
    approx.tl = po.p;
    approx.tr = p2.p;
    approx.br = p.p;
    approx.bl = p1.p;
    approx.s_tl = po.point_score;
    approx.s_tr = p2.point_score;
    approx.s_br = p.point_score;
    approx.s_bl = p1.point_score;
    approx.p_tl = std::make_shared<ApproxBoxPoints>(po);
    approx.p_tr = std::make_shared<ApproxBoxPoints>(p2);
    approx.p_br = std::make_shared<ApproxBoxPoints>(p);
    approx.p_bl = std::make_shared<ApproxBoxPoints>(p1);
    approx.is_complete = is_complete;
    approx.length = ret_len;
    approx.slant_length = ret_slant_len;
    approx.dir_in = dir_in;

    return true;
}

void PSD_FusionModuleIF::ModifyCornerScore(const PSMaskU8 &mask,
                                          Eigen::Vector2f &p,
                                          float &score,
                                          float boarder_dis) {
    const float dis_x = std::min(p(0), param_.input_w - 1 - p(0));
    const float dis_y = std::min(p(1), param_.input_h - 1 - p(1));
    if (p(0) < 0.F || p(0) >= param_.input_w || p(1) < 0.F ||
        p(1) >= param_.input_h) {
        score = 0.1F;
    } else {
        if (mask.At(p(0), p(1), 0) != 0) {
            score = std::max(score, 0.5F);
        } else if (dis_x < boarder_dis || dis_y < boarder_dis) {
            score = 0.1F;
        } else {
            score *= 0.5F;
        }

        // Check If corner under car
        if (param_.car_length_range.InRange(p(1)) &&
            param_.car_width_range.InRange(p(0))) {
            score -= 0.00;
        }
    }
}

bool PSD_FusionModuleIF::CheckSlant(ParkingSlotQuad &quad) {
    auto p1 = quad.tl;
    auto p2 = quad.tr;
    auto p3 = quad.bl;
    auto p4 = quad.br;
    auto pp_1 = quad.p_tl;
    auto pp_2 = quad.p_tr;
    auto pp_3 = quad.p_bl;
    auto pp_4 = quad.p_br;
    if (pp_1 == nullptr || pp_2 == nullptr || pp_3 == nullptr ||
        pp_4 == nullptr)
        return false;
    Eigen::Vector2f dir1 = {floor((p1(0) - p2(0)) / 2), floor((p1(1) - p2(1)) / 2)};
    dir1 = dir1.normalized();
    Eigen::Vector2f dir2 = {floor((p1(0) - p3(0)) / 2), floor((p1(1) - p3(1)) / 2)};
    dir2 = dir2.normalized();
    Eigen::Vector2f dir3 = {floor((p3(0) - p4(0)) / 2), floor((p3(1) - p4(1)) / 2)};
    dir3 = dir3.normalized();
    Eigen::Vector2f dir4 = {floor((p4(0) - p2(0)) / 2), floor((p4(1) - p2(1)) / 2)};
    dir4 = dir4.normalized();
    float parallel_cos_v = std::abs(Cosine(dir1, dir3));
    float parallel_cos_p = std::abs(Cosine(dir2, dir4));
    const float COS75{0.2588f};
    const float COS15{0.9659f};
    if (parallel_cos_v > 0.99 && parallel_cos_p > 0.99) {
        float dir2_cos =
            (std::abs(Cosine(dir1, dir2)) + std::abs(Cosine(dir3, dir2))) / 2;
        float dir4_cos =
            (std::abs(Cosine(dir1, dir4)) + std::abs(Cosine(dir3, dir4))) / 2;
        if (dir2_cos > COS75 && dir2_cos < COS15 && dir4_cos > COS75 &&
            dir4_cos < COS15) {
           
            return true;
        } else {
            return false;
        }
    }

    auto valid_slant_cos = [](float dir_cos1, float dir_cos2,
                              float slant_cos_para, float slant_cos_low,
                              float slant_cos_up) {
        return (dir_cos1 > dir_cos2 && dir_cos2 < slant_cos_para &&
                dir_cos1 > slant_cos_low && dir_cos1 < slant_cos_up);
    };

    bool valid_flag = false;
    float cur_border_dist = 0.0;
    float border_point_dis_thr = param_.border_point_dis_thr;
    if (parallel_cos_v > 0.99 && parallel_cos_p < 0.982) {
        float dir2_cos =
            (std::abs(Cosine(dir1, dir2)) + std::abs(Cosine(dir3, dir2))) / 2;
        float dir4_cos =
            (std::abs(Cosine(dir1, dir4)) + std::abs(Cosine(dir3, dir4))) / 2;
        
        if (valid_slant_cos(dir2_cos, dir4_cos, param_.slant_cos_para,
                            param_.slant_cos_low, param_.slant_cos_up) &&
            pp_3->border_dist > border_point_dis_thr &&
            pp_1->border_dist > border_point_dis_thr &&
            pp_3->border_dist > cur_border_dist) {
            cur_border_dist = pp_3->border_dist;
            quad.tl = p3;
            quad.br = p2;
            quad.bl = p4;
            quad.tr = p1;
            quad.p_tl = pp_3;
            quad.p_br = pp_2;
            quad.p_bl = pp_4;
            quad.p_tr = pp_1;
            
            valid_flag = true;
        }
        if (valid_slant_cos(dir4_cos, dir2_cos, param_.slant_cos_para,
                            param_.slant_cos_low, param_.slant_cos_up) &&
            pp_2->border_dist > border_point_dis_thr &&
            pp_4->border_dist > border_point_dis_thr &&
            pp_2->border_dist > cur_border_dist) {
            cur_border_dist = pp_2->border_dist;
            quad.tl = p2;
            quad.br = p3;
            quad.bl = p1;
            quad.tr = p4;
            quad.p_tl = pp_2;
            quad.p_br = pp_3;
            quad.p_bl = pp_1;
            quad.p_tr = pp_4;
           
            valid_flag = true;
        }
    }
    if (parallel_cos_p > 0.99 && parallel_cos_v < 0.982) {
        float dir1_cos =
            (std::abs(Cosine(dir1, dir2)) + std::abs(Cosine(dir1, dir4))) / 2;
        float dir3_cos =
            (std::abs(Cosine(dir3, dir2)) + std::abs(Cosine(dir3, dir4))) / 2;
        
        if (valid_slant_cos(dir1_cos, dir3_cos, param_.slant_cos_para,
                            param_.slant_cos_low, param_.slant_cos_up) &&
            pp_1->border_dist > border_point_dis_thr &&
            pp_2->border_dist > border_point_dis_thr &&
            pp_1->border_dist > cur_border_dist) {
            cur_border_dist = pp_1->border_dist;
            quad.tl = p1;
            quad.br = p4;
            quad.bl = p3;
            quad.tr = p2;
            quad.p_tl = pp_1;
            quad.p_br = pp_4;
            quad.p_bl = pp_3;
            quad.p_tr = pp_2;
            
            valid_flag = true;
        }
        if (valid_slant_cos(dir3_cos, dir1_cos, param_.slant_cos_para,
                            param_.slant_cos_low, param_.slant_cos_up) &&
            pp_4->border_dist > border_point_dis_thr &&
            pp_3->border_dist > border_point_dis_thr &&
            pp_4->border_dist > cur_border_dist) {
            cur_border_dist = pp_4->border_dist;
            quad.tl = p4;
            quad.br = p1;
            quad.bl = p2;
            quad.tr = p3;
            quad.p_tl = pp_4;
            quad.p_br = pp_1;
            quad.p_bl = pp_2;
            quad.p_tr = pp_3;
            
            valid_flag = true;
        }
    }
    return valid_flag;
}

void PSD_FusionModuleIF::RecalculateRetlen(ParkingSlotQuad &quad) {
    const auto &p = quad.p_br;
    const auto &p1 = quad.p_tr;
    const auto &po = quad.p_tl;
    const auto &p2 = quad.p_bl;
    bool is_complete = quad.is_complete;
    float ret_len = 0.F;
    float ret_slant_len = 0.F;

    if (is_complete || param_.ps_width_range.InRange(po->line_len) ||
        param_.ps_width_slant_range.InRange(po->line_len)) {
        int ret_len_size = 0;
        if (param_.ps_length_slant_complete_range.InRange(p1->line_len) ||
            (param_.ps_length_slant_2_range.InRange(p1->line_len) &&
             p2->border_dist > param_.point_border_dist_complete_h2 &&
             po->border_dist > param_.point_border_dist_complete_h2)) {
            ret_slant_len += p1->line_len;
            ret_len_size++;
        }
        if (param_.ps_length_slant_complete_range.InRange(p2->line_len) ||
            (param_.ps_length_slant_2_range.InRange(p2->line_len) &&
             p2->border_dist > param_.point_border_dist_complete_h2 &&
             p->border_dist > param_.point_border_dist_complete_h2)) {
            ret_slant_len += p2->line_len;
            ret_len_size++;
        }
        if (ret_len_size) {
            ret_slant_len /= ret_len_size;
        }
    }

    quad.length = ret_len;
    quad.slant_length = ret_slant_len;
    
}

bool PSD_FusionModuleIF::CalibrateSingleSlot(const padVisionSlotCoord &quad,
                                             const PSMaskU8 &mask,
                                            ParkingSlotQuad &approx_quad) {
    // approx_quad = quad;
    // parking_bboxes topic上的tl已经被补全的车位覆盖了，需要重新拿回来。
    approx_quad.tl << quad.a.x, quad.a.y;
    approx_quad.tr << quad.b.x, quad.b.y;
    approx_quad.br << quad.c.x, quad.c.y; //TODO
    approx_quad.bl << quad.d.x, quad.d.y;

    // 对角线顶点x差值最大值和对角线顶点y差值最大值来判断是否为小矩形以及宽高比来过滤车位
    float Filtering_small_boxes_threshold = 0.025;
    float Filtering_small_boxes_threshold_wh = 4;
    float height = std::max(abs(approx_quad.tl(1) - approx_quad.tr(1)),
                            abs(approx_quad.br(1) - approx_quad.bl(1)));
    float width = std::max(abs(approx_quad.tl(0) - approx_quad.bl(0)),
                           abs(approx_quad.tr(0) - approx_quad.br(0)));
    float min_w = 0.1;
    // width = std::max(width, min_w);
    if (width < min_w) {
        return false;
    }
    if (height * width < Filtering_small_boxes_threshold * param_.input_w *
                             param_.input_h and
        (height / width > Filtering_small_boxes_threshold_wh or
         height / width < 1 / Filtering_small_boxes_threshold_wh)) {
        return false;
    }
    ModifyCornerScore(mask, approx_quad.tl, approx_quad.s_tl,
                      param_.point_border_dis_thres_for_score_modify);
    ModifyCornerScore(mask, approx_quad.tr, approx_quad.s_tr,
                      param_.point_border_dis_thres_for_score_modify);
    ModifyCornerScore(mask, approx_quad.bl, approx_quad.s_bl,
                      param_.point_border_dis_thres_for_score_modify);
    ModifyCornerScore(mask, approx_quad.br, approx_quad.s_br,
                      param_.point_border_dis_thres_for_score_modify);

    bool approx_flag = ApproxBox(mask, approx_quad);
    if (CheckSlant(approx_quad)) {
        if (!param_.ps_width_range.InRange(approx_quad.p_tl->line_len) &&
            !param_.ps_width_slant_range.InRange(approx_quad.p_tl->line_len)) {
            approx_flag = false;
        } else {
            approx_quad.valid_slant = true;
            RecalculateRetlen(approx_quad);
        }
    }
   
    return approx_flag;
}


void PSD_FusionModuleIF::shrink_quad(apaSlotInfo &original_rect){
    if (original_rect.rectInfo.PStype == 0){//垂直车位
        int shrink_amount = 120;
        POINT_F AB_unit = unit_vector(original_rect.rectInfo.pt[0],original_rect.rectInfo.pt[1]);
        POINT_F CD_unit = unit_vector(original_rect.rectInfo.pt[2],original_rect.rectInfo.pt[3]);
        

        original_rect.rectInfo.pt[0] = shrink(original_rect.rectInfo.pt[0],AB_unit,shrink_amount);
        original_rect.rectInfo.pt[1] = shrink(original_rect.rectInfo.pt[1],AB_unit,-shrink_amount);
        original_rect.rectInfo.pt[2] = shrink(original_rect.rectInfo.pt[2],CD_unit,shrink_amount);
        original_rect.rectInfo.pt[3] = shrink(original_rect.rectInfo.pt[3],CD_unit,-shrink_amount);
    }else if(original_rect.rectInfo.PStype == 1){ //平行车位
        int shrink_amount = 80;
        
        POINT_F AD_unit = unit_vector(original_rect.rectInfo.pt[0],original_rect.rectInfo.pt[3]);
        POINT_F BC_unit = unit_vector(original_rect.rectInfo.pt[1],original_rect.rectInfo.pt[2]);

        original_rect.rectInfo.pt[0] = shrink(original_rect.rectInfo.pt[0],AD_unit,shrink_amount);
        original_rect.rectInfo.pt[1] = shrink(original_rect.rectInfo.pt[1],BC_unit,shrink_amount);
        
    }
}

POINT_F PSD_FusionModuleIF::unit_vector(POINT_I p1, POINT_I p2){
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float length = sqrt(dx * dx + dy * dy);

    return POINT_F(dx / length, dy / length);
}

POINT_I PSD_FusionModuleIF::shrink(POINT_I p, POINT_F direction, int shrink){
    return POINT_I(p.x + direction.x * shrink, p.y + direction.y * shrink);
}
/* EOF */
