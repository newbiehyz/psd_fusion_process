/*
 * @copyright @ 2024 iAUTO(Shanghai) Co., Ltd. All rights reserved.
 * @Copyright @ 2020 - 2020 Pan Asia Technical Automotive Center Co., Ltd. All Rights Reserved.
 *
 * @brief 算法内部用于管理车位所有信息、进行位置更新的class.
 * @author changgui qian <changgui_qian@patac.com>
 * @date since 2024.11.27
 */
#pragma once

#include <memory>
#include <iostream>
#include <array>
#include <vector>
#include "Eigen/Core"
#include "apa_define.h"

enum SLOT_STATE_ELEMENT {
    SLOT_CENTER_X = 0,
    SLOT_CENTER_Y = 1,
    SLOT_ALPHA = 2,
    SLOT_BETA = 3,
    SLOT_LENGTH = 4,
    SLOT_WIDTH = 5,
    SLOT_STATE_SIZE = 6
};

enum MEASURE_ELEMENT {
    TOP_LEFT_X = 0,
    TOP_LEFT_Y = 1,
    TOP_RIGHT_X = 2,
    TOP_RIGHT_Y = 3,
    BOTTOM_RIGHT_X = 4,
    BOTTOM_RIGHT_Y = 5,
    BOTTOM_LEFT_X = 6,
    BOTTOM_LEFT_Y = 7,
    SLOT_MEASURE_SIZE = 8
};

enum class CORNER_STATUS : uint8_t {
    INVISIBLE = 0,
    VISIBLE = 1,
    CONFIRMED = 2,
    UNDER_CAR = 3
};

enum class SLOT_TYPE : uint8_t {
    VERTICALSLOT = 0,
    PARALLELSLOT = 1,
    SLANTSLOT = 2
};

enum class SLOT_SOURCE : uint8_t { VISUAL = 0, SPACING = 1, FUSED = 2 };
enum class SLOT_STATUS : uint8_t { TENTATIVE = 0, CONFIRMED = 1, DELETE = 2 };

class Kalman_filter{
 public:
    struct InnerSlotParameters {
        // For Q_
        float tight_center_coeff = 9e-4f;
        float tight_angle_coeff = 1e-6f;
        float tight_length_coeff = 1e-4f;
        float loose_center_coeff = 9e-4f;
        float loose_angle_coeff = 4e-6f;
        float loose_length_coeff = 4e-4f;

        float same_point_thr = 0.5f;  // 旋转点时候判断是同一个点的阈值
        float valid_measure_thr = 0.2f;  // 判定是有效测量的阈值
        // 判定无效测量点的协方差放大倍率
        float invalid_measure_enlarge_ratio = 9.0f;
        float invalid_innovation_thr = 1.0f;  // 偏差过大阈值

        uint32_t confirm_age = 2;  // 确认车位输出所需的帧数
        uint32_t delete_age = 2;   // 确认车位删除所需的帧数
        uint32_t delta_frame_thr = 200;  // 判断车位长时间没有被观测的阈值
    };
 
    InnerSlotParameters isp_;

 public:
    Kalman_filter() = delete;
    explicit Kalman_filter(const ParkingSlotResultPtr& post_slot,
              const QuadInfoPtr& quad_info);
    ~Kalman_filter();

    uint32_t GetSlotApaId() const { return apa_id_; }
    uint32_t GetSlotAge() const { return age_; }
    SLOT_TYPE GetSlotType() const { return type_; }
    SLOT_SOURCE GetSlotSource() const { return source_; }
    Eigen::Vector3f GetSlotCenter() const {
        Eigen::Vector3f c = Eigen::Vector3f::Ones();
        c.head<2>() = slot_state_.head<2>();
        return c;
    };
    Eigen::Vector3f GetSlotLongDir() const { return long_dir_; };
    Eigen::Vector3f GetSlotWideDir() const { return wide_dir_; };
    float GetSlotLongAngle() const { return slot_state_[SLOT_ALPHA]; };
    float GetSlotWideAngle() const { return slot_state_[SLOT_BETA]; };
    float GetSlotLength() const { return slot_state_[SLOT_LENGTH]; };
    float GetSlotWidth() const { return slot_state_[SLOT_WIDTH]; };
    float GetMinDist2EgoCar() const { return min_dist2egocar_; }
    uint32_t GetLatestFrameId() const { return lastest_frame_id_; }

    std::array<Eigen::Vector3f, 4> GetCornersWorld() const {
        return corners_world_;
    };
    std::array<CORNER_STATUS, 4> GetConerStatus() const {
        return corner_status_;
    };
    bool IsTentative() const {
        return this->status_ == SLOT_STATUS::TENTATIVE;
    };

    bool IsConfiremd() const {
        return this->status_ == SLOT_STATUS::CONFIRMED;
    };

    bool IsToBeDeleted() const { return this->status_ == SLOT_STATUS::DELETE; };

    void IncreaseMissingTime() {
        ++missing_time_;
        if (SLOT_STATUS::TENTATIVE == this->status_ &&
            this->missing_time_ > isp_.delete_age)
            this->status_ = SLOT_STATUS::DELETE;
    };
    void SetMinDist2EgoCar(const float& dist) { min_dist2egocar_ = dist; }

    void SetLatestFrameId(const uint32_t& new_frame_id) {
        delta_frame_cnt_ = new_frame_id - lastest_frame_id_;
        lastest_frame_id_ = new_frame_id;
    }

    bool point_in_slot(const Eigen::Vector3f& point,
                       float ratio_thr = 1.0) const;
    void Update(const QuadInfoPtr& quad_info);

 private:
    void creat_initial_covariance(const QuadInfoPtr& quad_info);
    bool pre_update(const QuadInfoPtr& quad_info);
    void extend_line(const QuadInfoPtr& quad_info, size_t valid, size_t edge);

 private:
    static uint32_t id_generator_;
    
    // 内部管理的唯一车位ID
    uint32_t apa_id_;
    // 观测次数
    uint32_t age_;
    // 车位确认有效之前的漏检次数
    uint32_t missing_time_;
    // 车位类型
    SLOT_TYPE type_;
    // 车位来源
    SLOT_SOURCE source_;
    // 车位跟踪状态
    SLOT_STATUS status_;
    // 最新观测帧Id
    uint32_t lastest_frame_id_ = 0;
    // 距离上一次观测间隔帧数
    uint32_t delta_frame_cnt_ = 0;

    // 状态向量
    Eigen::Matrix<float, SLOT_STATE_SIZE, 1> slot_state_;
     // 预测矩阵
    Eigen::Matrix<float, SLOT_STATE_SIZE, SLOT_STATE_SIZE> F_;
    // 状态协方差矩阵
    Eigen::Matrix<float, SLOT_STATE_SIZE, SLOT_STATE_SIZE> P_, Q_;
    // 测量矩阵
    Eigen::Matrix<float, SLOT_MEASURE_SIZE, SLOT_STATE_SIZE> H_;
    // 噪声协方差矩阵
    Eigen::Matrix<float, SLOT_MEASURE_SIZE, SLOT_MEASURE_SIZE> R_;

    // 当前角点在世界坐标系位置
    std::array<Eigen::Vector3f, 4> corners_world_;
    // 角点状态
    std::array<CORNER_STATUS, 4> corner_status_;
    // 车位深度朝向
    Eigen::Vector3f long_dir_;
    // 车位宽度朝向
    Eigen::Vector3f wide_dir_;

    // 自车距离车位最近距离
    float min_dist2egocar_;

    size_t rot_idx_{0UL};
    int valid_quad_cnt_{0};
    static size_t max_rot_idx_;  // 4UL
    
};
typedef std::shared_ptr<Kalman_filter> Kalman_filterPtr;