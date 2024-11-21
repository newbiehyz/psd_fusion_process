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
#ifndef __cplusplus
#   error ERROR: This file requires C++ compilation (use a .cpp suffix)
#endif

#ifndef PSD_FUSION_MODULE_IF
#define PSD_FUSION_MODULE_IF

#include "apa_define.h"
#include "math.h"
#include <mutex>
#include "apa_module_id.h"
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
    int                 m_frame_id;
    std::mutex m_psinfo_mutex;
    std::vector<ObstacleInfo> m_obstacle_info;
    int                 m_select_slot_label_id;
    int                 m_slot_direction;
    std::vector<int>    m_min_distance;
    int m_next_available_label_idx;
    FILE* m_all_slot_out;
public:
    PSD_FusionModuleIF() = default;
    virtual ~PSD_FusionModuleIF() = default;

    virtual bool Initialize();

    virtual bool Start();

    virtual bool Stop();

    virtual bool Destroy();

    void RegisterCallback(IFusionMapCallback* callback, Fusion_Error_Code& error_code);
    //void UpdateSonarSlots(const UssInfo& info);
    void UpdateVechiclePose(const padVehiclePose& pose_global);
    void UpdateVisionSlots(int frameid, std::vector<padVisionSlotCoord> slots);
    void UpdateUserSelectSlotId(int user_select_slot_id);
    //void UpdateSonarObstacle(const UssInfo& info);
    apaSlotListInfo GetOutputSlot()
    {
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

    //void ProcessObstaclePoint(const UssInfo& info);
    bool CheckSlotAreaObstacleNum(apaSlotInfo rect, padVisionSlotCoord slot);
    bool isOddNumber(const POINT_I& obstacle, const std::vector<int>& rect_pt_x, const std::vector<int>& rect_pt_y);

    int CalMixSideDistance(const apaSlotInfo& target_slot);
    std::vector<int> MixDistanceDataset(const apaSlotInfo& target_slot, const ObstacleInfo& obstacle_point);
    int CalPointAndLineDistance(const POINT_I& point, const POINT_I& pta, const POINT_I& ptb);
    APA_SPACE::ERECT_EDGE_SOD_TYPE CalcDownSlotSOD(const apaSlotInfo& targetslot);
    POINT_I coordConvert_car_center(const padPoint& slot);
    static bool compareDistance(int pre, int current);

 PSD_FusionModuleIF(const PSD_FusionModuleIF &);
 PSD_FusionModuleIF & operator=(const PSD_FusionModuleIF &);    
};
#endif
/* EOF */