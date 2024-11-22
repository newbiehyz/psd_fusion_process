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
#define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7 //e2sb:1100,458:1130
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

bool PSD_FusionModuleIF::Initialize()
{
    m_select_slot_label_id = -1;
    m_apa_psinfo.WorldoutRect.clear();
    m_slot_direction = 0;
    m_next_available_label_idx = 1;
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

void PSD_FusionModuleIF::RegisterCallback(IFusionMapCallback* callback, Fusion_Error_Code& error_code)
{
    error_code = FUSION_SUCCESS;
    if(callback) {
        m_callback = callback; 
    }
    else {
        error_code = REGISTER_POINTER_FAIL;
        //APA_Error_Log(APA_MODULE_ID_PSD_FUSION, "error_code=%d, error_message=RegisterCallback is fail", error_code);
    }

}

// void PSD_FusionModuleIF::ProcessObstaclePoint(const UssInfo& info)
// {
//     apa_performance cm_init("[PSD_FusionModuleIF] ProcessObstaclePoint"); 
//     //obstacle coordiate
//     padPoint left_front;
//     padPoint left_rear;
//     padPoint right_front;
//     padPoint right_rear;

//     ObstacleInfo item;

//     if(info.APAFLS_Distance < INVALID_DISTANCE) {
//         left_front.x = -( VEHICLE_WIDTH / 2 + info.APAFLS_Distance);
//         left_front.y = VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR - FRONT_SONAR_PITCH_HEAD;
//         item.front_left = coordConvert_sonar_dr(left_front, m_vehicle_pose);
//     }
//     else {
//         item.front_left.x = INVALID_VALUE;
//         item.front_left.y = INVALID_VALUE;
//     }

//     if(info.APARLS_Distance < INVALID_DISTANCE ) {
//         left_rear.x = -( VEHICLE_WIDTH / 2 + info.APAFLS_Distance);
//         left_rear.y = VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR - (FRONT_SONAR_PITCH_HEAD + FRONT_REAR_SONAR_DISTANCE);
//         item.rear_left = coordConvert_sonar_dr(left_rear, m_vehicle_pose);
//     }
//     else {
//         item.rear_left.x = INVALID_VALUE;
//         item.rear_left.y = INVALID_VALUE;
//     }

//     if(info.APAFRS_Distance < INVALID_DISTANCE) {  
//         right_front.x = VEHICLE_WIDTH / 2 + info.APAFRS_Distance;
//         right_front.y = VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR - FRONT_SONAR_PITCH_HEAD;
//         item.front_right = coordConvert_sonar_dr(right_front, m_vehicle_pose);
//     }
//     else {
//         item.front_right.x = INVALID_VALUE;
//         item.front_right.y = INVALID_VALUE;
//     }

//     if(info.APARRS_Distance < INVALID_DISTANCE) {
//         right_rear.x = VEHICLE_WIDTH / 2 + info.APARRS_Distance;
//         right_rear.y = VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR - (FRONT_SONAR_PITCH_HEAD + FRONT_REAR_SONAR_DISTANCE);
//         item.rear_right = coordConvert_sonar_dr(right_rear, m_vehicle_pose);
//     }
//     else {
//         item.rear_right.x = INVALID_VALUE;
//         item.rear_right.y = INVALID_VALUE;
//     }

//     APA_Debug_Log(APA_MODULE_ID_PSD_FUSION, "origin obstacle pa[%d,%d],pb[%d,%d],pc[%d,%d],pd[%d,%d]\n",
//     left_front.x, left_front.y,
//     left_rear.x, left_rear.y, right_front.x, right_front.y, right_rear.x , right_rear.y);


//     APA_Debug_Log(APA_MODULE_ID_PSD_FUSION, "Convert obstacle pa[%d,%d],pb[%d,%d],pc[%d,%d],pd[%d,%d]\n",
//     item.front_left.x, item.front_left.y, item.rear_left.x, item.rear_left.y, 
//     item.front_right.x, item.front_right.y, item.rear_right.x, item.rear_right.y);

//     item.vehicle_pose = m_vehicle_pose;
    
//     m_obstacle_info.push_back(item);

//     if(m_obstacle_info.size() > OBSTACLE_POINTS_SIZE) {
//         int num = m_obstacle_info.size() - OBSTACLE_POINTS_SIZE;
//         m_obstacle_info.erase(m_obstacle_info.begin(), m_obstacle_info.begin() + num);
//     }

//     auto it = m_obstacle_info.begin();
//     while (it!=m_obstacle_info.end()){
//         int length = sqrt(pow((it->vehicle_pose.coord.x - m_vehicle_pose.coord.x),2) + 
//                             pow((it->vehicle_pose.coord.y - m_vehicle_pose.coord.y),2));
//         if (length > OBSTACLE_POINTS_THRESHOLD)
//             it = m_obstacle_info.erase(it);//返回下一个元素iterator
//         else
//             it++;
//     }
// }

bool PSD_FusionModuleIF::isOddNumber(const POINT_I& obstacle, const std::vector<int>& rect_pt_x, const std::vector<int>& rect_pt_y)
{
    int x_max = *max_element(rect_pt_x.begin(), rect_pt_x.end());
    int x_min = *min_element(rect_pt_x.begin(), rect_pt_x.end());
    int y_max = *max_element(rect_pt_y.begin(), rect_pt_y.end());
    int y_min = *min_element(rect_pt_y.begin(), rect_pt_y.end());

    if(obstacle.x < x_min || obstacle.x > x_max || obstacle.y < y_min || obstacle.y > y_max) {
        return false;
    }

    int i,j,c=0;
    for(i=0, j=rect_pt_x.size() - 1; i < rect_pt_x.size() - 1; j = i++){
        if(((rect_pt_y[i]>obstacle.y) != (rect_pt_y[j]>obstacle.y)) &&
        (obstacle.x < (rect_pt_x[j]-rect_pt_x[i]) * (obstacle.y-rect_pt_y[i]) / (rect_pt_y[j]-rect_pt_y[i]) + rect_pt_x[i]) )
            c = !c;
    }

    if(c){
        return true;
    }

    return false;

}

bool PSD_FusionModuleIF::CheckSlotAreaObstacleNum(apaSlotInfo rect, padVisionSlotCoord slot)
{
    //apa_performance cm_init("[PSD_FusionModuleIF] CheckSlotAreaObstacleNum"); 
    int num = 0;
    std::vector<int> rect_pt_x, rect_pt_y;
    for(auto point : rect.rectInfo.pt){
        rect_pt_x.push_back(point.x);
        rect_pt_y.push_back(point.y);
    }

    for(auto info : m_obstacle_info) {
        bool lf_flag = isOddNumber(info.front_left, rect_pt_x, rect_pt_y);
        bool lr_flag = isOddNumber(info.rear_left, rect_pt_x, rect_pt_y);
        bool rf_flag = isOddNumber(info.front_right, rect_pt_x, rect_pt_y);
        bool rr_flag = isOddNumber(info.rear_right, rect_pt_x, rect_pt_y);

        if(lf_flag || lr_flag || rf_flag || rr_flag) {
            num++;
        }
    }

    if(num > NUM) {
        return true;
    }

    return false;
}

// void PSD_FusionModuleIF::UpdateSonarSlots(const UssInfo& info)
// {
//     // Invalid parking space
//     if((info.USSlot_PtA_X == info.USSlot_PtB_X) && (info.USSlot_PtA_Y == info.USSlot_PtB_Y)) {
//         return;
//     }

//     if(info.USSlot_Depth > 32000 && info.USSlot_Length > 32000){
//         return;
//     }

//     if ((info.USSlot_PtA_X == 0) 
//     && (info.USSlot_PtA_Y == 0)
//     && (info.USSlot_PtB_X == 0)
//     && (info.USSlot_PtB_Y == 0) ) {
//         return;
//     }
//     padPoint pa;
//     padPoint pb;
//     padPoint pc;
//     padPoint pd;
//     pa.x = info.USSlot_PtA_X;
//     pa.y = info.USSlot_PtA_Y;

//     pb.x = info.USSlot_PtB_X;
//     pb.y = info.USSlot_PtB_Y;
//     int depth = info.USSlot_Depth;
//     // depth = info.USSlot_Depth > 6000 ? 6000 : info.USSlot_Depth;   
//     // depth = 6000;
//     if (pa.x < 0) { // left slot
//         pc.x = pb.x - depth;
//         pc.y = pb.y;

//         pd.x = pa.x - depth;
//         pd.y = pa.y;
//     }
//     else { // right slot
//         pc.x = pb.x + depth;
//         pc.y = pb.y;

//         pd.x = pa.x + depth;
//         pd.y = pa.y;
//     }
    

//     apaSlotInfo rect;
//     rect.rectInfo.iRectType = 1; // 1为超声波检测结果 
//     rect.rectInfo.label = 0;
//     rect.rectInfo.iSodType = 0;
//     // rect.rectInfo.iMinOtherSideDist = -1;

//     rect.rectInfo.pt[0] = coordConvert_sonar_dr(pa, m_vehicle_pose);
//     rect.rectInfo.pt[1] = coordConvert_sonar_dr(pb, m_vehicle_pose);
//     rect.rectInfo.pt[2] = coordConvert_sonar_dr(pc, m_vehicle_pose);
//     rect.rectInfo.pt[3] = coordConvert_sonar_dr(pd, m_vehicle_pose);
//     rect.sonar_pt[0] = rect.rectInfo.pt[0];
//     rect.sonar_pt[1] = rect.rectInfo.pt[1];
//     rect.sonar_pt[2] = rect.rectInfo.pt[2];
//     rect.sonar_pt[3] = rect.rectInfo.pt[3];

//     bool flag = true;
//     auto it = existed_in_psinfo(rect,flag); 
    
//     if (it == m_apa_psinfo.WorldoutRect.end()) {
//         rect.rectInfo.label = m_apa_psinfo.WorldoutRect.size();
//         m_apa_psinfo.WorldoutRect.push_back(rect);
//         return;
//     }

//     //0为视觉检测的，1为超声波检测出来的车位信息，2为视觉和超声波融合得出来的信息
//     if (it->rectInfo.iRectType == 0) {
//         // change the slot type to mix
//         it->rectInfo.iRectType = 2;

//     }
//     else if(it->rectInfo.iRectType == 1) {
//         // ommit it
//         memcpy(&it->rectInfo.pt, &rect.rectInfo.pt, 4 * sizeof(POINT_I));
//     }
//     else if(it->rectInfo.iRectType == 2) {
//         // ommit it
//     }
//     else {

//     }
//     memcpy(&it->sonar_pt, &rect.sonar_pt, 4 * sizeof(POINT_I));
//     APA_Debug_Log(APA_MODULE_ID_PSD_FUSION, "it->sonar_pt pa[%d,%d],pb[%d,%d],pc[%d,%d],pd[%d,%d]", 
//                 it->sonar_pt[0].x,
//                 it->sonar_pt[0].y,
//                 it->sonar_pt[1].x,
//                 it->sonar_pt[1].y,
//                 it->sonar_pt[2].x,
//                 it->sonar_pt[2].y,
//                 it->sonar_pt[3].x,
//                 it->sonar_pt[3].y)
//                 ;    
// }

void PSD_FusionModuleIF::UpdateVechiclePose(const padVehiclePose& pose_global)
{
    m_vehicle_pose = pose_global;
}

// POINT_I PSD_FusionModuleIF::coordConvert_sonar_dr(const padPoint& slot, const padVehiclePose& pose)
// {
//     POINT_I grand;
//     int x = slot.x;
//     int y = slot.y;
    
//     float yaw = pose.yaw * PI / 180.0;

//     grand.x = x * cos(yaw) + y * sin(yaw);
//     grand.y = y * cos(yaw) - x * sin(yaw);

//     grand.x += pose.coord.x;
//     grand.y += pose.coord.y;

//     grand.x /= LR_BIRD_PIXECL_2_WORLD;
//     grand.y /= LR_BIRD_PIXECL_2_WORLD;

//     return grand;
// }

POINT_I PSD_FusionModuleIF::coordConvert_car_center(const padPoint& slot)
{
    POINT_I grand;

    float x = slot.x - BIRD_VIEW_HEIGHT/2;
    float y = BIRD_VIEW_HEIGHT/2 - slot.y;

    grand.x = x * LR_BIRD_PIXECL_2_WORLD;
    grand.y = y * LR_BIRD_PIXECL_2_WORLD;

    return grand;
}

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
    grand.y += pose.coord.y;

    return grand;
}

// void PSD_FusionModuleIF::UpdateSonarObstacle(const UssInfo& info)
// {
//     // temp delete
//     ProcessObstaclePoint(info);
//     // if(m_select_slot_label_id == -1) {
//     //     return;
//     // }

//     // for(auto& rect : m_apa_psinfo.WorldoutRect) {
//     //     if (rect.label == m_select_slot_label_id) { 
//     //         std::vector<int> distance = MixDistanceDataset(rect, m_obstacle_info[m_obstacle_info.size() - 1]);

//     //         if(distance.size() > 0) {
//     //             std::vector<int> data;
//     //             data.insert(data.end(), distance.begin(), distance.end());
//     //             std::sort(data.begin(), data.end(), compareDistance);
//     //             m_min_distance.clear();
//     //             m_min_distance.insert(m_min_distance.begin(), data.begin(), data.begin() + EFFECTIVE_THRESHOLD);
//     //         }
//     //         rect.iMinOtherSideDist = m_min_distance[m_min_distance.size() - 1] * LR_BIRD_PIXECL_2_WORLD;
//     //     }
//     // }
//     // // update fusion map
//     // if (m_callback) {
//     //     m_callback->UpdateFusionMap(m_frame_id, m_apa_psinfo);
//     // }
// }

ERECT_EDGE_SOD_TYPE  PSD_FusionModuleIF::CalcDownSlotSOD(const apaSlotInfo& targetslot)
{
    // 0 : B
    // 1 : A
    // 2 : D
    // 3 : C
    // vision slot info
    int ax = targetslot.rectInfo.pt[1].x;
    int ay = targetslot.rectInfo.pt[1].y;
    int bx = targetslot.rectInfo.pt[0].x;
    int by = targetslot.rectInfo.pt[0].y;

    int sodType_A = -1;
    int sodType_B = -1;
    for(auto& rect : m_apa_psinfo.WorldoutRect) {
        int rect_ax = rect.rectInfo.pt[1].x;
        int rect_ay = rect.rectInfo.pt[1].y;
        int rect_bx = rect.rectInfo.pt[0].x;
        int rect_by = rect.rectInfo.pt[0].y;
        if ((abs(ax - rect_bx) < 100) && (abs(ay - rect_by) < 100)) {
            sodType_A = rect.rectInfo.iSodType;
        }
        else if ((abs(bx - rect_ax) < 100) && (abs(by - rect_ay) < 100)) {
            sodType_B = rect.rectInfo.iSodType;
        }
    }
    // E_EDGESOD_DEFAULT = 0,   //默认，上下空间都未知情况
    // E_EDGESOD_U_UNKNOW_D_EMPTY,  //上部空间未知，下部分为空
    // E_EDGESOD_U_UNKNOW_D_SOD,  //上部空间未知，下部分为SOD
    // E_EDGESOD_U_EMPTY_D_UNKNOW,  //上部空间为空，下部分为未知
    // E_EDGESOD_U_EMPTY_D_EMPTY,  //上部空间为空，下部分为空
    // E_EDGESOD_U_EMPTY_D_SOD,  //上部空间为空，下部分为SOD
    // E_EDGESOD_U_SOD_D_UNKNOW,  //上部空间为SOD，下部分为未知
    // E_EDGESOD_U_SOD_D_EMPTY,  //上部空间为SOD，下部分为空
    // E_EDGESOD_U_SOD_D_SOD   //上部空间为SOD，下部分为SOD

    ERECT_EDGE_SOD_TYPE  egdesod_type = E_EDGESOD_DEFAULT;
    if ((sodType_A == -1) && (sodType_B == 0)) {
        egdesod_type = E_EDGESOD_U_UNKNOW_D_EMPTY;
    }
    else if ((sodType_A == -1) && (sodType_B == 1)) {
        egdesod_type = E_EDGESOD_U_UNKNOW_D_SOD;
    }
    else if ((sodType_A == 0) && (sodType_B == -1)) {
        egdesod_type = E_EDGESOD_U_EMPTY_D_UNKNOW;
    }
    else if ((sodType_A == 0) && (sodType_B == 0)) {
        egdesod_type = E_EDGESOD_U_EMPTY_D_EMPTY;
    }
    else if ((sodType_A == 0) && (sodType_B == 1)) {
        egdesod_type = E_EDGESOD_U_EMPTY_D_SOD;
    }
    else if ((sodType_A == 1) && (sodType_B == -1)) {
        egdesod_type = E_EDGESOD_U_SOD_D_UNKNOW;
    }
    else if ((sodType_A == 1) && (sodType_B == 0)) {
        egdesod_type = E_EDGESOD_U_SOD_D_EMPTY;
    }
    else if ((sodType_A == 1) && (sodType_B == 1)) {
        egdesod_type = E_EDGESOD_U_SOD_D_SOD;
    }
    else {
        egdesod_type = E_EDGESOD_DEFAULT;
    }
    return egdesod_type;
}

void PSD_FusionModuleIF::UpdateUserSelectSlotId(int user_select_slot_id)
{
    m_select_slot_label_id = user_select_slot_id;
    //APA_Debug_Log(APA_MODULE_ID_PSD_FUSION, "m_select_slot_label_id=%d", m_select_slot_label_id);
    for(auto& rect : m_apa_psinfo.WorldoutRect) {
        if (rect.rectInfo.label == m_select_slot_label_id) { 
            // check right or left slot
            int direction = (rect.rectInfo.pt[0].x - rect.rectInfo.pt[1].x) / (rect.rectInfo.pt[0].y - rect.rectInfo.pt[1].y) * (m_vehicle_pose.coord.y - rect.rectInfo.pt[1].y) + rect.rectInfo.pt[1].x;
            direction *= LR_BIRD_PIXECL_2_WORLD;
            if(direction < m_vehicle_pose.coord.x) {
                m_slot_direction = 1; // select left slot
            }

            rect.rectInfo.iMinOtherSideDist = CalMixSideDistance(rect);
            rect.rectInfo.iDownSlotSOD = CalcDownSlotSOD(rect);
        }
    }
    // update fusion map
    if (m_callback) {
        m_callback->UpdateFusionMap(m_frame_id, m_apa_psinfo);
    }
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
    float threshold = EFFECTIVE_OBSTACLE_POINT_DISTANCE / LR_BIRD_PIXECL_2_WORLD;
    if((point.x != INVALID_VALUE) && (point.y != INVALID_VALUE) && 
       (CalcDistance(point, pta) <= threshold || CalcDistance(point, ptb) <= threshold)) {
        return (fabs((ptb.y - pta.y) * point.x + (pta.x - ptb.x) * point.y + ((ptb.x * pta.y) - (pta.x * ptb.y)))) / (sqrt(pow(ptb.y - pta.y, 2) + pow(pta.x - ptb.x, 2)));
  
    }

    return -1;
}

void PSD_FusionModuleIF::UpdateVisionSlots(int frameid, std::vector<padVisionSlotCoord> slots)
{
    //check updatevisionslots' input
    printf("[_test updatevisionslots] start!!\n");
    printf("[_test updatevisionslots check] timestamp:%d, RD's singelframe slots have: %d\n",frameid,slots.size());

    std::lock_guard<std::mutex> lock(m_psinfo_mutex);
    
    if(slots.size() <= 0) {
        return;
    }

    //清空上一帧车位
    m_frame_id = frameid;
    m_output_slot.padRealTimeLocation.x = m_vehicle_pose.coord.x;
    m_output_slot.padRealTimeLocation.y = m_vehicle_pose.coord.y;
    m_output_slot.padRealTimeLocation.yaw = m_vehicle_pose.yaw;
    m_output_slot.ullFrameId = frameid;
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
        //中心点不在有效范围
        if((slot.a.y + slot.b.y) / 2 < EFFECTIVE_SLOT_Y_1 || (slot.a.y + slot.b.y) / 2 > EFFECTIVE_SLOT_Y_2) {
            continue;
        }

        apaSlotInfo rect;
        apaSlotInfo rect_car_center;
        rect.rectInfo.iRectType = 0; // 0为视觉检测结果 
        rect.rectInfo.iSodType = 0;  // 0为没有障碍物，1为有障碍物，-1为未知情况（reset,或车停下来）
        // rect.rectInfo.iMinOtherSideDist = -1;
        rect.rectInfo.level=2; //跟踪值还是检测值 2为检测值，1为跟踪值
        rect.rectInfo.PStype = slot.bayType;
        rect.detect_frame_count = 1; 

        //从RD左上角原点 转为 后轴中心为原点
        rect.rectInfo.pt[0] = coordConvert_global_dr(slot.a, m_vehicle_pose);
        rect.rectInfo.pt[1] = coordConvert_global_dr(slot.b, m_vehicle_pose);
        rect.rectInfo.pt[2] = coordConvert_global_dr(slot.c, m_vehicle_pose);
        rect.rectInfo.pt[3] = coordConvert_global_dr(slot.d, m_vehicle_pose);

        bool mis_detect_flag = true;
        auto it = existed_in_psinfo(rect, mis_detect_flag); 
        if(!mis_detect_flag && it == m_apa_psinfo.WorldoutRect.end()) {
            continue;
        }

        if (it == m_apa_psinfo.WorldoutRect.end()) {
            rect.rectInfo.label = m_next_available_label_idx++;
            if(m_apa_psinfo.WorldoutRect.size() >= MAX_SLOT_NUM) {
                m_apa_psinfo.WorldoutRect.erase(m_apa_psinfo.WorldoutRect.begin());
            }
            m_apa_psinfo.WorldoutRect.push_back(rect);
        }
        else {
            rect.rectInfo.label = it->rectInfo.label;
            rect.detect_frame_count = it->detect_frame_count + 1;
            rect.is_reliable = rect.detect_frame_count >= 5;
            rect.detect_as_occupy_count = it->detect_as_occupy_count + slot.occupy;
            rect.rectInfo.iSodType = float(rect.detect_as_occupy_count) / float(rect.detect_frame_count) > 0.5;

            if (m_select_slot_label_id > 0) { 
                if(m_select_slot_label_id == rect.rectInfo.label) { 
                    *it = rect;
                }
            }
            else {
                *it = rect;
            } 

            if(rect.is_reliable == 1) {
                rect_car_center = *it;
                rect_car_center.rectInfo.pt[0] = coordConvert_car_center(slot.a);
                rect_car_center.rectInfo.pt[1] = coordConvert_car_center(slot.b);
                rect_car_center.rectInfo.pt[2] = coordConvert_car_center(slot.c);
                rect_car_center.rectInfo.pt[3] = coordConvert_car_center(slot.d); 
                
                m_output_slot.slots_in_cur_frame.push_back(rect_car_center);
                // //check m_output_slot.slots_in_cur_frame
                // printf("[_test updatevisionslots] slots_in_cur_frame size: %d\n",m_output_slot.slots_in_cur_frame.size());
                // for (const auto& psd_slot1 : m_output_slot.slots_in_cur_frame)
                // {
                //     printf("[_test updatevisionslots] slots_in_cur_frame, isreliable:%d, ",psd_slot1.is_reliable);
                //     for (int i = 0; i < RECTPointNum; ++i) 
                //     {
                //         printf("(%d,%d)",psd_slot1.rectInfo.pt[i].x,psd_slot1.rectInfo.pt[i].y);
                //     }
                //     printf("\n");
                // }
            }  
        }
    }
   

    for(auto info = m_apa_psinfo.WorldoutRect.rbegin(); info != m_apa_psinfo.WorldoutRect.rend(); ++info) {
        if(info->is_reliable == 1) {
            m_output_slot.WorldoutRect.push_back(*info);
        }
    }

    // check m_output_slot 
    printf("[_test updatevisionslots] m_output_slot list size: %d\n",m_output_slot.WorldoutRect.size());
    for (const auto& psd_slot2 : m_output_slot.WorldoutRect)
    {
        printf("[_test updatevisionslots] m_output_slot, isreliable:%d, label:%d, PStype:%d, ",
        psd_slot2.is_reliable,psd_slot2.rectInfo.label,psd_slot2.rectInfo.PStype);
        for (int i = 0; i < RECTPointNum; ++i) 
        {
            printf("(%d,%d)",psd_slot2.rectInfo.pt[i].x,psd_slot2.rectInfo.pt[i].y);
        }
        printf("\n");
    }

    if (m_callback) {
        m_callback->UpdateFusionMap(m_frame_id, m_output_slot);
    }
    printf("[_test updatevisionslots] end!!\n");
}


int PSD_FusionModuleIF::CalcDistance(POINT_I a, POINT_I b)
{
    return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}


vector<apaSlotInfo>::iterator PSD_FusionModuleIF::existed_in_psinfo(const apaSlotInfo& rect_new, bool& mis_detect_flag)
{
    if(m_next_available_label_idx > 120) {
        printf("id=%d\n",m_frame_id); 
    }

    double iou = 0;;

    if(m_apa_psinfo.WorldoutRect.size() == 0) {
        auto it = m_apa_psinfo.WorldoutRect.end();
        return it;
    }

    //size>0时，反向遍历psinfo
    //从倒数第一个向前遍历
    auto it = m_apa_psinfo.WorldoutRect.end() - 1; 
    for( ; it >= m_apa_psinfo.WorldoutRect.begin(); it--) 
    {
        //把rect_new转换成Vertexes格式的vert_new
        Vertexes vert_new, vert;
        changePoint(rect_new, vert_new);
        changePoint(*it, vert);
        iou = iouEx(vert_new, vert); //当前帧和上一帧的矩形框IOU
        
        //IOU>0.4重复
        if (iou >= 0.4) {
            break;
        }

        //0.2-0.4misdetect
        else if (iou >= 0.2 && iou < 0.4) {
            mis_detect_flag = false;
        }

        //<0.2认为没有相同车位继续循环
        
    }
    if(iou < 0.4) 
    {
        it = m_apa_psinfo.WorldoutRect.end(); //遍历完，没有找到重复车位
    }
    return it;
}


/* EOF */
