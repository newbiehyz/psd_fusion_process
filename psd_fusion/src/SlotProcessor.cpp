#include "SlotProcessor.h"
#include "utils.h"
#include <Eigen/Core>
#include <cmath>
#include <mutex>

void ProcessMapInfoSlot(const Loc::MapInfo& map_info,
                                       const padVehiclePose& pose_globaldata,
                                       apaSlotListInfo& outputSlot_VIS,
                                       PSD_FusionModuleIF& fusionModule)
{
    double pose_x = static_cast<double>(pose_globaldata.coord.x) * 0.001;
    double pose_y = static_cast<double>(pose_globaldata.coord.y) * 0.001;
    double theta = pose_globaldata.yaw / 180.0 * M_PI;
    Eigen::Vector2d twb(pose_x, pose_y);
    Eigen::Matrix2d Rwb;
    Rwb << std::cos(theta), -std::sin(theta),
           std::sin(theta),  std::cos(theta);

    outputSlot_VIS.slots_in_cur_frame.clear();
    outputSlot_VIS.WorldoutRect.clear();

    for (const auto& ps : map_info.ParkingSlot) {
        Eigen::Vector2d c_b(ps.center.x, ps.center.y);
        Eigen::Vector2d lon_dir(ps.longDirection.x, ps.longDirection.y);
        Eigen::Vector2d wide_dir(ps.wideDirection.x, ps.wideDirection.y);

        Eigen::Vector2d pt0_b = c_b + lon_dir * ps.length * 0.5 - wide_dir * ps.width * 0.5;
        Eigen::Vector2d pt1_b = c_b + lon_dir * ps.length * 0.5 + wide_dir * ps.width * 0.5;
        Eigen::Vector2d pt2_b = c_b - lon_dir * ps.length * 0.5 + wide_dir * ps.width * 0.5;
        Eigen::Vector2d pt3_b = c_b - lon_dir * ps.length * 0.5 - wide_dir * ps.width * 0.5;

        Eigen::Vector2d pt0_w = Rwb * pt0_b + twb;
        Eigen::Vector2d pt1_w = Rwb * pt1_b + twb;
        Eigen::Vector2d pt2_w = Rwb * pt2_b + twb;
        Eigen::Vector2d pt3_w = Rwb * pt3_b + twb;

        apaSlotInfo info_local, info_world;
        info_local.rectInfo.pt[1] = { static_cast<int>(-pt0_b.y() * 1000), static_cast<int>(pt0_b.x() * 1000) };
        info_local.rectInfo.pt[0] = { static_cast<int>(-pt1_b.y() * 1000), static_cast<int>(pt1_b.x() * 1000) };
        info_local.rectInfo.pt[3] = { static_cast<int>(-pt2_b.y() * 1000), static_cast<int>(pt2_b.x() * 1000) };
        info_local.rectInfo.pt[2] = { static_cast<int>(-pt3_b.y() * 1000), static_cast<int>(pt3_b.x() * 1000) };
        info_local.rectInfo.label = ps.id;
        info_local.rectInfo.PStype = ps.psType;
        info_local.rectInfo.iSodType = ps.isOccupancy;
        fusionModule.adjustRectOrder(info_local);
        outputSlot_VIS.slots_in_cur_frame.push_back(info_local);

        info_world.rectInfo.pt[0] = { static_cast<int>(pt0_w.x() * 1000), static_cast<int>(pt0_w.y() * 1000) };
        info_world.rectInfo.pt[1] = { static_cast<int>(pt1_w.x() * 1000), static_cast<int>(pt1_w.y() * 1000) };
        info_world.rectInfo.pt[2] = { static_cast<int>(pt2_w.x() * 1000), static_cast<int>(pt2_w.y() * 1000) };
        info_world.rectInfo.pt[3] = { static_cast<int>(pt3_w.x() * 1000), static_cast<int>(pt3_w.y() * 1000) };
        info_world.rectInfo.label = ps.id;
        info_world.rectInfo.PStype = ps.psType;
        info_world.rectInfo.iSodType = ps.isOccupancy;
        outputSlot_VIS.WorldoutRect.push_back(info_world);
    }

    LOGD("[APA_SLAM] finish processing slot list");
    LogSlotInfo(outputSlot_VIS, "ORIGIN VISSLOTS");
}


void ProcessUssSlots(UssIf_stPLVOutputInfo_t& uss_info,
                     UssIf_stPLVOutputInfo_t& uss_info_restruct,
                     apaSlotListInfo& outputSlot_USS,
                     apaSlotListInfo& outputSlot_VIS,
                     apaSlotListInfo& outputSlot_FUSED,
                     slotfusion& fusionslot)
{
    // get USS
    S2S_MCore_Bridge_GetSigUssIf_stPLVOutputInfo(&uss_info);
    // USS信息重构
    fusionslot.fillVisonstruct(uss_info, outputSlot_USS);
    fusionslot.clearInvalidUSSslots(outputSlot_USS);
    fusionslot.postprocessUSSslots(uss_info_restruct);
    fusionslot.mergeSlotLists(outputSlot_USS, outputSlot_VIS, outputSlot_FUSED);


    // 统计车位数
    LogSlotInfo(outputSlot_USS, "ORIGIN USSSLOTS");
    slotlist_size = outputSlot_FUSED.slots_in_cur_frame.size();
    LOGD("After VIS/USS Merge FUSIONSLOTS:")
    LogSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");
    LogWorldSlotInfo(outputSlot_FUSED, "FUSIONSLOTS");
}
