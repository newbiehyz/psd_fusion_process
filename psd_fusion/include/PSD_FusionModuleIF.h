/**
* Copyright @ 2024 iAUTO(Shanghai) Co., Ltd.
* All Rights Reserved.
*
* Copyright @ 2024 Pan Asia Technical Automotive Center Co., Ltd.
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
#ifndef __cplusplus
#   error ERROR: This file requires C++ compilation (use a .cpp suffix)
#endif

#ifndef PSD_FUSION_MODULE_IF
#define PSD_FUSION_MODULE_IF

#include "psd_fusion_process_header.h"
#include "all.h"
#include "apa_define.h"
#include "math.h"
#include <map>
#include <mutex>
#include "Eigen/Core"
#include "apa_module_id.h"
#include "kd_tree.hpp"
#include "Kalman_filter.h"

/**
 * PSD_FusionModuleIF
 *
 *
 */
typedef enum { 
    FUSION_SUCCESS = APA_ERRORCODE_BASE_PSD_FUSION + 0,
    REGISTER_POINTER_FAIL = APA_ERRORCODE_BASE_PSD_FUSION + 1
}Fusion_Error_Code;
class IFusionMapCallback
{
    public:
        virtual void UpdateFusionMap(int frameid, const apaSlotListInfo& apainfo) = 0;
};

typedef struct 
{
    int vec_index;
    int label_index;
} LabelInfo;

typedef struct 
{
    padVehiclePose vehicle_pose;
    POINT_I front_left;
    POINT_I rear_left;
    POINT_I front_right;
    POINT_I rear_right;
} ObstacleInfo;

class PSD_FusionModuleIF 
{
private:
    IFusionMapCallback* m_callback;
    padVehiclePose      m_vehicle_pose;
    apaSlotListInfo     m_apa_psinfo;
    apaSlotListInfo     m_output_slot;
    std::mutex          m_output_slot_mutex;
    int                 m_frame_id;
    std::mutex m_psinfo_mutex;
    std::vector<ObstacleInfo> m_obstacle_info;
    int                 m_select_slot_label_id;
    int                 m_slot_direction;
    std::vector<int>    m_min_distance;
    int m_next_available_label_idx;
    FILE* m_all_slot_out;

    Eigen::Matrix3f intrinsic_car2ipm_, intrinsic_ipm2car_;
    std::shared_ptr<KDTree> slots_tree_;
    std::map<uint32_t, Kalman_filterPtr> slots_map_;
    // kdtree index to slot id
    std::unordered_map<uint64_t, uint32_t> slots_remap_;
    ParkingSlotParam param_;
    std::shared_ptr<PSMaskU8> mParkingLineMask_ptr= nullptr;

public:
    PSD_FusionModuleIF() = default;
    virtual ~PSD_FusionModuleIF() = default;

    virtual bool Initialize();

    virtual bool Start();

    virtual bool Stop();

    virtual bool Destroy();

    void UpdateVechiclePose(const padVehiclePose& pose_global);
    void UpdateVisionSlots(uint64_t frameid, std::vector<padVisionSlotCoord> slots, int status, int holdstatus);
    // void CalStopDisAndLoc(apaSlotInfo psdmoutput, Fus::PkEmapObs &empobs, float &stopdis, int &stoplocation, int &lockinslot, int &obsinslot);
    void CalStopDisAndLoc(const Fus::PkEmapObs &empobs);
    void adjustRectOrder(apaSlotInfo &rect);
    bool isLeftOfOrigin(const apaSlotInfo rect);

    apaSlotListInfo GetOutputSlot()
    {
        // std::lock_guard<std::mutex> ld(m_output_slot_mutex);
        return m_output_slot;
    }
    apaSlotListInfo GetApaPsinfo()
    {
        return m_apa_psinfo;
    }

private:
    POINT_I coordConvert_global_dr(const padPoint& slot, const padVehiclePose& pose);
    //POINT_I coordConvert_sonar_dr(const padPoint& slot, const padVehiclePose& pose);
    int CalcDistance(POINT_I a, POINT_I b);
    vector<apaSlotInfo>::iterator existed_in_psinfo(const apaSlotInfo& rect, bool& mis_detect_flag);

    int CalMixSideDistance(const apaSlotInfo& target_slot);
    std::vector<int> MixDistanceDataset(const apaSlotInfo& target_slot, const ObstacleInfo& obstacle_point);
    int CalPointAndLineDistance(const POINT_I& point, const POINT_I& pta, const POINT_I& ptb);
    POINT_I coordConvert_car_center(const padPoint& slot);
    static bool compareDistance(int pre, int current);

    /**
     * @brief 检查输入的QuadInfo是否与已经跟踪的车位匹配
     * @param quad_info 模型检出的车位信息
     * @return 能找到已经跟踪的车位，将其返回，否则返回nullptr
     */
    POINT_F unit_vector(POINT_I p1, POINT_I p2);
    POINT_I shrink(POINT_I p, POINT_F direction, int shrink_x);
    Kalman_filterPtr check_slot_existance(const QuadInfoPtr& quad_info);
    void transform2world(const padVehiclePose& loc_pose, QuadInfoPtr& quad_info);
    void transform2world(const padVehiclePose& loc_pose, const ParkingSlotResultPtr& post_result,QuadInfoPtr& quad_info);
    void rebuild_slots_tree();
    bool ProcessParkingSlotResult(const PSMaskU8 &parking_line_mask, const padVisionSlotCoord &bbox, ParkingSlotResultPtr &result);
    bool FillSingleSlot(ParkingSlotQuad &approx_quad);
    void ObtainDirection(ParkingSlotQuad &quad);
    void ObtainSlotType(ParkingSlotQuad &quad);
    void CompleteBoxWithArrowFix(ParkingSlotQuad &quad);
    void ModifyDirIn(ParkingSlotQuad &quad);
    bool CalibrateSingleSlot(const padVisionSlotCoord &quad, const PSMaskU8 &mask, ParkingSlotQuad &approx_quad);
    bool ApproxBox(const PSMaskU8 &mask, ParkingSlotQuad &approx);
    bool CheckSlant(ParkingSlotQuad &quad);
    void RecalculateRetlen(ParkingSlotQuad &quad);
    bool CheckComplete(const std::vector<ApproxBoxPoints> &points);
    void collect_confirmed_slots(apaSlotListInfo &slot_res);
    void delete_invalid_slots();
    void ModifyCornerScore(const PSMaskU8 &mask,
                           Eigen::Vector2f &p,
                           float &score,
                           float boarder_dis = 2.F);
    void world2car(Eigen::Vector3f &pt);
 public:
    void shrink_quad(apaSlotInfo &original_rect);


 PSD_FusionModuleIF(const PSD_FusionModuleIF &);
 PSD_FusionModuleIF & operator=(const PSD_FusionModuleIF &);    
};
#endif
/* EOF */