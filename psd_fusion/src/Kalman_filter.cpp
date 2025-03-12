#include "Kalman_filter.h"
#include "Eigen/Dense"
#include "c_sdk.h"

uint32_t Kalman_filter::id_generator_ = std::atomic<uint32_t>(1000);
uint64_t Kalman_filter::max_rot_idx_ = 4UL;
// Kalman_filter::InnerSlotParameters Kalman_filter::isp_;
// IPMParameters ipmp_;
// ParkingSlotManagerParameters psmp_;

Kalman_filter::Kalman_filter(const ParkingSlotResultPtr& post_slot,
                             const QuadInfoPtr& quad_info){
    age_ = 0;
    missing_time_ = 0;
    apa_id_ = id_generator_;

    occupy_ = quad_info->occupy;


    // occupy_ = 1;
    // occupies_.push(quad_info->occupy);
    // occupy_sum_ += occupy_;


    material_ = quad_info->material;
    ++id_generator_;
    
    switch (post_slot->type)
    {
    case 0x00:
        type_ = SLOT_TYPE::VERTICALSLOT;
        break;
    case 0x01:
        type_ = SLOT_TYPE::PARALLELSLOT;
        break;
    case 0x02:
        type_ = SLOT_TYPE::SLANTSLOT;
    default:
        break;
    }

    source_ = SLOT_SOURCE::VISUAL;
    status_ = SLOT_STATUS::TENTATIVE;

    slot_state_<< quad_info->center_world.x(),
                  quad_info->center_world.y(),
                  std::atan2(quad_info->long_dir_world.y(),
                             quad_info->long_dir_world.x()),
                  std::atan2(quad_info->wide_dir_world.y(),
                             quad_info->wide_dir_world.x()),
                  quad_info->length_world,quad_info->width_world;

    corners_world_.at(0) = quad_info->corners_world.col(0);
    corners_world_.at(1) = quad_info->corners_world.col(1);
    corners_world_.at(2) = quad_info->corners_world.col(2);
    corners_world_.at(3) = quad_info->corners_world.col(3);

    corner_status_ = {CORNER_STATUS::INVISIBLE, CORNER_STATUS::INVISIBLE,
                      CORNER_STATUS::INVISIBLE, CORNER_STATUS::INVISIBLE};
    long_dir_.setZero();
    long_dir_.head<2>() = quad_info->long_dir_world;
    wide_dir_.setZero();
    wide_dir_.head<2>() = quad_info->wide_dir_world;

    creat_initial_covariance(quad_info);

    min_dist2egocar_ = quad_info->center_ego.head<2>().norm();
};

Kalman_filter::~Kalman_filter(){};

void Kalman_filter::creat_initial_covariance(const QuadInfoPtr& quad_info) {
    // 设置初始化的协方差矩阵
    this->F_.setIdentity();

    this->Q_.setIdentity();
    this->Q_(SLOT_CENTER_X, SLOT_CENTER_X) *= isp_.loose_center_coeff;
    this->Q_(SLOT_CENTER_Y, SLOT_CENTER_Y) *= isp_.loose_center_coeff;

    switch (this->type_) {
        case SLOT_TYPE::SLANTSLOT:  // 斜列
            this->Q_(SLOT_ALPHA, SLOT_ALPHA) *= isp_.loose_angle_coeff;
            this->Q_(SLOT_BETA, SLOT_BETA) *= isp_.loose_angle_coeff;
            this->Q_(SLOT_LENGTH, SLOT_LENGTH) *= isp_.loose_length_coeff;
            this->Q_(SLOT_WIDTH, SLOT_WIDTH) *= isp_.loose_length_coeff;
            break;

        case SLOT_TYPE::PARALLELSLOT:  // 平行：wide准
        case SLOT_TYPE::VERTICALSLOT:  // 垂直和默认：wide准
        default:
            this->Q_(SLOT_ALPHA, SLOT_ALPHA) *= isp_.loose_angle_coeff;
            this->Q_(SLOT_BETA, SLOT_BETA) *= isp_.tight_angle_coeff;
            this->Q_(SLOT_LENGTH, SLOT_LENGTH) *= isp_.loose_length_coeff;
            this->Q_(SLOT_WIDTH, SLOT_WIDTH) *= isp_.tight_length_coeff;
            break;
    }

    // P_的结果依赖角点的协方差信息
    if (!this->pre_update(quad_info)) {
        this->P_.setIdentity();
        this->P_ *= 1e-4;
        return;
    };

    Eigen::Matrix<float, 6, 8> J;
    J.setZero();

    // 求cx，cy对四个角点的偏导数
    J(SLOT_CENTER_X, TOP_LEFT_X) = 0.25f;  // 1/4
    J(SLOT_CENTER_Y, TOP_LEFT_Y) = 0.25f;
    J(SLOT_CENTER_X, TOP_RIGHT_X) = 0.25f;
    J(SLOT_CENTER_Y, TOP_RIGHT_Y) = 0.25f;
    J(SLOT_CENTER_X, BOTTOM_RIGHT_X) = 0.25f;
    J(SLOT_CENTER_Y, BOTTOM_RIGHT_Y) = 0.25f;
    J(SLOT_CENTER_X, BOTTOM_LEFT_X) = 0.25f;
    J(SLOT_CENTER_Y, BOTTOM_LEFT_Y) = 0.25f;

    const auto& x1 = (this->corners_world_.at(0).x())/1000;
    const auto& y1 = (this->corners_world_.at(0).y())/1000;
    const auto& x2 = (this->corners_world_.at(1).x())/1000;
    const auto& y2 = (this->corners_world_.at(1).y())/1000;
    const auto& x3 = (this->corners_world_.at(2).x())/1000;
    const auto& y3 = (this->corners_world_.at(2).y())/1000;
    const auto& x4 = (this->corners_world_.at(3).x())/1000;
    const auto& y4 = (this->corners_world_.at(3).y())/1000;

    // 求alpha对四个角点的偏导数
    float t = y1 + y2 - y3 - y4;  // top 分子
    float b = x1 + x2 - x3 - x4;  // bottom 分母
    if (std::fabs(b) < 1e-3) {
        b = b < 0 ? -1e-3 : 1e-3;
    }
    float u = t / b;
    float df_du = 1.0f / (1.0f + u * u);
    float du_dx = df_du * (t / (b * b));
    float du_dy = df_du / b;
    J(SLOT_ALPHA, TOP_LEFT_X) = -du_dx;
    J(SLOT_ALPHA, TOP_LEFT_Y) = du_dy;
    J(SLOT_ALPHA, TOP_RIGHT_X) = -du_dx;
    J(SLOT_ALPHA, TOP_RIGHT_Y) = du_dy;
    J(SLOT_ALPHA, BOTTOM_RIGHT_X) = du_dx;
    J(SLOT_ALPHA, BOTTOM_RIGHT_Y) = -du_dy;
    J(SLOT_ALPHA, BOTTOM_LEFT_X) = du_dx;
    J(SLOT_ALPHA, BOTTOM_LEFT_Y) = -du_dy;

    // 求beta对四个角点的偏导数
    t = y1 - y2 - y3 + y4;  // top 分子
    b = x1 - x2 - x3 + x4;  // bottom 分母
    if (std::fabs(b) < 1e-3) {
        b = b < 0 ? -1e-3 : 1e-3;
    }
    u = t / b;
    df_du = 1.0f / (1.0f + u * u);
    du_dx = df_du * (t / (b * b));
    du_dy = df_du / b;
    J(SLOT_BETA, TOP_LEFT_X) = -du_dx;
    J(SLOT_BETA, TOP_LEFT_Y) = du_dy;
    J(SLOT_BETA, TOP_RIGHT_X) = du_dx;
    J(SLOT_BETA, TOP_RIGHT_Y) = -du_dy;
    J(SLOT_BETA, BOTTOM_RIGHT_X) = du_dx;
    J(SLOT_BETA, BOTTOM_RIGHT_Y) = -du_dy;
    J(SLOT_BETA, BOTTOM_LEFT_X) = -du_dx;
    J(SLOT_BETA, BOTTOM_LEFT_Y) = du_dy;

    // 求length对四个角点的偏导数
    float A = 2.0f * std::sqrt((y1 - y4) * (y1 - y4) + (x1 - x4) * (x1 - x4));
    float B = 2.0f * std::sqrt((y2 - y3) * (y2 - y3) + (x2 - x3) * (x2 - x3));

    J(SLOT_LENGTH, TOP_LEFT_X) = (x1 - x4) / A;
    J(SLOT_LENGTH, TOP_LEFT_Y) = (y1 - y4) / A;
    J(SLOT_LENGTH, TOP_RIGHT_X) = (x2 - x3) / B;
    J(SLOT_LENGTH, TOP_RIGHT_Y) = (y2 - y3) / B;
    J(SLOT_LENGTH, BOTTOM_RIGHT_X) = -(x2 - x3) / B;
    J(SLOT_LENGTH, BOTTOM_RIGHT_Y) = -(y2 - y3) / B;
    J(SLOT_LENGTH, BOTTOM_LEFT_X) = -(x1 - x4) / A;
    J(SLOT_LENGTH, BOTTOM_LEFT_Y) = -(y1 - y4) / A;

    // 求length对四个角点的偏导数
    A = 2.0f * std::sqrt((y1 - y2) * (y1 - y2) + (x1 - x2) * (x1 - x2));
    B = 2.0f * std::sqrt((y3 - y4) * (y3 - y4) + (x3 - x4) * (x3 - x4));

    J(SLOT_WIDTH, TOP_LEFT_X) = (x1 - x2) / A;
    J(SLOT_WIDTH, TOP_LEFT_Y) = (y1 - y2) / A;
    J(SLOT_WIDTH, TOP_RIGHT_X) = -(x1 - x2) / A;
    J(SLOT_WIDTH, TOP_RIGHT_Y) = -(y1 - y2) / A;
    J(SLOT_WIDTH, BOTTOM_RIGHT_X) = (x3 - x4) / B;
    J(SLOT_WIDTH, BOTTOM_RIGHT_Y) = (y3 - y4) / B;
    J(SLOT_WIDTH, BOTTOM_LEFT_X) = -(x3 - x4) / B;
    J(SLOT_WIDTH, BOTTOM_LEFT_Y) = -(y3 - y4) / B;

    Eigen::Matrix<float, 8, 8> P0;
    P0.setZero();
    for (uint64_t i = 0; i < max_rot_idx_; ++i) {
        P0.block<2, 2>(2 * i, 2 * i) = quad_info->cov_extended.at(i);
    }
    this->P_.setIdentity();
    this->P_ = J * P0 * J.transpose();
};

void Kalman_filter::extend_line(const QuadInfoPtr& quad_info,
                            size_t valid,
                            size_t edge) {
    quad_info->corners_extended.col(valid) =
        quad_info->corners_world.col(valid);
    quad_info->cov_extended.at(valid) = quad_info->cov_world.at(valid);

    const auto& valid_point = quad_info->corners_world.col(valid);
    const auto& edge_point = quad_info->corners_world.col(edge);
    float len = ((valid_point - edge_point).head<2>().norm())/1000;
    float extend_ratio;
    if (std::fabs((valid_point - edge_point)
                      .head<2>()
                      .dot(this->GetSlotLongDir().head<2>())) <
        std::fabs((valid_point - edge_point)
                      .head<2>()
                      .dot(this->GetSlotWideDir().head<2>()))) {
        extend_ratio = this->GetSlotLength() / len;
    } else {
        extend_ratio = this->GetSlotWidth() / len;
    }
    quad_info->corners_extended.col(edge) =
        valid_point + (edge_point - valid_point) * extend_ratio;
    quad_info->cov_extended.at(edge) =
        quad_info->cov_world.at(edge) * (extend_ratio * extend_ratio);
    
};

bool Kalman_filter::point_in_slot(const Eigen::Vector3f& point,
                              float ratio_thr) const {
    Eigen::Vector3f vec = (point - GetSlotCenter())/1000;
    float length_ratio =
        std::fabs(vec.head<2>().dot(GetSlotLongDir().head<2>())) /
        GetSlotLength() * 2.f;
    float width_ratio =
        std::fabs(vec.head<2>().dot(GetSlotWideDir().head<2>())) /
        GetSlotWidth() * 2.f;

    return (length_ratio < ratio_thr && width_ratio < ratio_thr);
};

bool Kalman_filter::point_in_rect(const Eigen::Vector3f &point) const{
    // 创建旋转矩阵（将长方向对齐到 x 轴）
    Eigen::Matrix3f rotation;
    rotation.col(0) = long_dir_.normalized();  // 长方向单位向量
    rotation.col(1) = wide_dir_.normalized();  // 宽方向单位向量
    
    // 逆旋转矩阵
    Eigen::Matrix3f rotation_inv = rotation.transpose(); // 旋转矩阵是正交矩阵，其逆等于转置
    
    Eigen::Vector3f center;
    center<<slot_state_[0],slot_state_[1], 0.0;
    // 将点转换到矩形的局部坐标系
    Eigen::Vector3f local_point = (rotation_inv * (point - center));
    
    // 检查点是否在矩形的范围内
    float half_length = slot_state_[SLOT_LENGTH] / 2.0f * 1000;
    float half_width = slot_state_[SLOT_WIDTH] / 2.0f * 1000;

    if (this->type_ == SLOT_TYPE::PARALLELSLOT){
        std::swap(half_length, half_width);
    }

    return (local_point.x() >= -half_width && local_point.x() <= half_width &&
            local_point.y() >= -half_length  && local_point.y() <= half_length);
}

bool Kalman_filter::pre_update(const QuadInfoPtr& quad_info) {

    //统计有效角点（靠近边界）
    valid_quad_cnt_ = 0;
    for (const auto& near : quad_info->near_edge) {
        if (!near) ++valid_quad_cnt_;
    }
    if (valid_quad_cnt_ < 2) {
        return false;
    }


    rot_idx_ = 0UL;
    for (; rot_idx_ < max_rot_idx_; ++rot_idx_) {
        int match_number = 0;
        // 遍历测量的每个点，看旋转几次车位能匹配最好。只通过距离匹配同一角点
        for (int i = 0; i < int(quad_info->corners_world.cols()); ++i) {
            const auto& p = quad_info->corners_world.col(i);
            const auto& corner = this->corners_world_.at((i + rot_idx_) % 4);
            float dist = (corner.head<2>() - p.head<2>()).norm();
            float same_point_thr = isp_.same_point_thr;
            //时间间隔过长，允许2倍距离误差
            if (delta_frame_cnt_ > isp_.delta_frame_thr) {
                same_point_thr *= 2;
            }
            if (dist < same_point_thr) {
                ++match_number;
            }
        }
        // 有两个点匹配上，就认为匹配成功了
        if (match_number > 1) break;
    };

    if (rot_idx_ == max_rot_idx_) {
        return false;
    }

    // 测量点的旋转次数和车位旋转次数互补，和为max_rot_idx
    rot_idx_ = (max_rot_idx_ - rot_idx_) % max_rot_idx_;

    // 根据角点的真实性，拓展得到猜测的车位角点
    for (size_t i = 0; i < quad_info->near_edge.size(); ++i) {
        size_t j = (i + 1) % max_rot_idx_;

        //如果相邻两个角点都不在边缘不被截断，直接拿corners_world
        if (!quad_info->near_edge.at(i) && !quad_info->near_edge.at(j)) {
            quad_info->corners_extended.col(i) =
                quad_info->corners_world.col(i);
            quad_info->cov_extended.at(i) = quad_info->cov_world.at(i);
            quad_info->corners_extended.col(j) =
                quad_info->corners_world.col(j);
            quad_info->cov_extended.at(j) = quad_info->cov_world.at(j);
        } else if (!quad_info->near_edge.at(i) && quad_info->near_edge.at(j)) {
            extend_line(quad_info, i, j);
        } else if (quad_info->near_edge.at(i) && !quad_info->near_edge.at(j)) {
            extend_line(quad_info, j, i);
        }
    }
    return true;
};

int prev_slot_side = 0;

void Kalman_filter::Update(const QuadInfoPtr& quad_info, const padVehiclePose& vehicle_pose) {
    std::cout<<"Kalman_filter update!"<<std::endl;
    //补全角点
    if (!this->pre_update(quad_info)) {
        return;
    }
    
    Eigen::Vector2f temp_center;
    temp_center << this->slot_state_(SLOT_CENTER_X), this->slot_state_(SLOT_CENTER_Y);
    // 更新预测矩阵，状态量不变，P需要调整
    P_ = F_ * P_ * F_.transpose() + Q_ * fmax(1.0, delta_frame_cnt_ / 10.0);

    // 更新观测矩阵
    float hsa = 0.5 * std::sin(GetSlotLongAngle());
    float hca = 0.5 * std::cos(GetSlotLongAngle());
    float hsb = 0.5 * std::sin(GetSlotWideAngle());
    float hcb = 0.5 * std::cos(GetSlotWideAngle());
    float length = GetSlotLength();
    float width = GetSlotWidth();
    H_.row(TOP_LEFT_X) << 1, 0, -length * hsa, -width * hsb, hca, hcb;
    H_.row(TOP_LEFT_Y) << 0, 1, length * hca, width * hcb, hsa, hsb;
    H_.row(TOP_RIGHT_X) << 1, 0, -length * hsa, width * hsb, hca, -hcb;
    H_.row(TOP_RIGHT_Y) << 0, 1, length * hca, -width * hcb, hsa, -hsb;
    H_.row(BOTTOM_RIGHT_X) << 1, 0, length * hsa, width * hsb, -hca, -hcb;
    H_.row(BOTTOM_RIGHT_Y) << 0, 1, -length * hca, -width * hcb, -hsa, -hsb;
    H_.row(BOTTOM_LEFT_X) << 1, 0, length * hsa, -width * hsb, -hca, hcb;
    H_.row(BOTTOM_LEFT_Y) << 0, 1, -length * hca, width * hcb, -hsa, hsb;

    // 更新测量噪声矩阵
    R_.setIdentity();
    const float valid_measure_thr = 200;  // unit: mm. //车位检测抖动大，角点跳动，增大。粘滞，真实车位变化响应慢，减小。
    for (size_t i = 0UL; i < 4UL; ++i) {
        size_t cur = (i + rot_idx_) % max_rot_idx_;
        const auto& q = quad_info->corners_extended.col(cur);
        const auto& c = this->corners_world_.at(i);
        float dist = (q.head<2>() - c.head<2>()).norm();

        Eigen::Index idx = 2 * i;
        R_.block<2, 2>(idx, idx) = quad_info->cov_extended.at(cur);
        if (dist > isp_.valid_measure_thr) {
            //角点跳动明显，滤波不够平滑R增加。角点响应太慢，车位不跟随变化R减小
            R_.block<2, 2>(idx, idx) *= isp_.invalid_measure_enlarge_ratio; //增大R_减少测量值影响依赖预测；减小R_增加测量值依赖测量
        }
    }

    Eigen::Matrix<float, SLOT_MEASURE_SIZE, 1> measure;
    const auto& m = quad_info->corners_extended;
    measure << m.col(0UL + rot_idx_).x(), m.col(0UL + rot_idx_).y(),
        m.col((1UL + rot_idx_) % max_rot_idx_).x(),
        m.col((1UL + rot_idx_) % max_rot_idx_).y(),
        m.col((2UL + rot_idx_) % max_rot_idx_).x(),
        m.col((2UL + rot_idx_) % max_rot_idx_).y(),
        m.col((3UL + rot_idx_) % max_rot_idx_).x(),
        m.col((3UL + rot_idx_) % max_rot_idx_).y();

    // 将Predict()得到的估计值，转换到测量坐标系
    Eigen::Matrix<float, SLOT_MEASURE_SIZE, 1> measure_prediction;
    measure_prediction(TOP_LEFT_X) = corners_world_.at(0).x();
    measure_prediction(TOP_LEFT_Y) = corners_world_.at(0).y();
    measure_prediction(TOP_RIGHT_X) = corners_world_.at(1).x();
    measure_prediction(TOP_RIGHT_Y) = corners_world_.at(1).y();
    measure_prediction(BOTTOM_RIGHT_X) = corners_world_.at(2).x();
    measure_prediction(BOTTOM_RIGHT_Y) = corners_world_.at(2).y();
    measure_prediction(BOTTOM_LEFT_X) = corners_world_.at(3).x();
    measure_prediction(BOTTOM_LEFT_Y) = corners_world_.at(3).y();
    Eigen::Matrix<float, SLOT_MEASURE_SIZE, SLOT_MEASURE_SIZE> S;
    S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<float, SLOT_STATE_SIZE, SLOT_MEASURE_SIZE> kalman_gain;
    kalman_gain = P_ * H_.transpose() * S.inverse();
    Eigen::Matrix<float, SLOT_MEASURE_SIZE, 1> innovation;
    // innovation = (measure - measure_prediction) / 1000.0;   //跳变大（误差大），增大。跟踪慢（误差小），减小
    innovation = (measure - measure_prediction) / 1000.0;   //跳变大（误差大），增大。跟踪慢（误差小），减小

    if (innovation.norm() > isp_.invalid_innovation_thr &&
        delta_frame_cnt_ < isp_.delta_frame_thr) {
        return;
    }

    auto normalize_angle = [](const double& ang) {
        double angle = ang;
        if (angle >= M_PI) {
            while (angle > M_PI) {
                angle -= 2 * M_PI;
            }
        } else if (angle < -M_PI) {
            while (angle < -M_PI) {
                angle += 2 * M_PI;
            }
        }
        return angle;
    };

    auto align_angle = [](const double& ang_diff) {
        double angle = ang_diff;
        if (angle < -M_PI_2) {
            angle += M_PI;
        } else if (angle >= M_PI_2) {
            angle -= M_PI;
        }
        return angle;
    };

    innovation(SLOT_ALPHA) = normalize_angle(innovation(SLOT_ALPHA));
    innovation(SLOT_ALPHA) = align_angle(innovation(SLOT_ALPHA));

    innovation(SLOT_BETA) = normalize_angle(innovation(SLOT_BETA));
    innovation(SLOT_BETA) = align_angle(innovation(SLOT_BETA));

    // 更新状态量和矩阵
    auto original_length = this->GetSlotLength();
    Eigen::Matrix<float, SLOT_STATE_SIZE, 1> change;
    auto test = kalman_gain * innovation;
    change << test[0], test[1], 0.0, 0.0, 0.0, 0.0;
    this->slot_state_ = this->slot_state_ + change;

    if (valid_quad_cnt_ < 3) {
        this->slot_state_(SLOT_LENGTH) = original_length;
    }
    P_ = P_ - kalman_gain * H_ * P_;

    this->slot_state_(SLOT_ALPHA) = normalize_angle(slot_state_(SLOT_ALPHA));
    this->slot_state_(SLOT_BETA) = normalize_angle(slot_state_(SLOT_BETA));

    // 更新车位角点信息
    auto center = GetSlotCenter();
    auto len_cur = GetSlotLength() * 666;
    auto wid_cur = GetSlotWidth() * 666;
 
    long_dir_ << std::cos(GetSlotLongAngle()), std::sin(GetSlotLongAngle()),0.0;
    wide_dir_ << std::cos(GetSlotWideAngle()), std::sin(GetSlotWideAngle()),0.0;


    if (type_ == SLOT_TYPE::VERTICALSLOT){
        if(this->slot_state_(SLOT_CENTER_X) > 0){
             corners_world_.at(0) =
                center + 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
            corners_world_.at(1) =
                center + 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
            corners_world_.at(2) =
                center - 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
            corners_world_.at(3) =
                center - 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
        }else{
            corners_world_.at(0) =
                center + 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
            corners_world_.at(1) =
                center + 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
            corners_world_.at(2) =
                center - 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
            corners_world_.at(3) =
                center - 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
        }
            
    }else if (type_ == SLOT_TYPE::PARALLELSLOT) {
    // 在平行车位中，长边是车位的宽度方向，宽边是车位的长度方向
        if(this->slot_state_(SLOT_CENTER_X) > 0){
            corners_world_.at(0) =
                center - 0.5 * wid_cur * wide_dir_ + 0.5 * len_cur * long_dir_;
            corners_world_.at(1) =
                center + 0.5 * wid_cur * wide_dir_ + 0.5 * len_cur * long_dir_;
            corners_world_.at(2) =
                center + 0.5 * wid_cur * wide_dir_ - 0.5 * len_cur * long_dir_;
            corners_world_.at(3) =
                center - 0.5 * wid_cur * wide_dir_ - 0.5 * len_cur * long_dir_;
        }else{
            corners_world_.at(0) =
                center + 0.5 * wid_cur * wide_dir_ + 0.5 * len_cur * long_dir_;
            corners_world_.at(1) =
                center - 0.5 * wid_cur * wide_dir_ + 0.5 * len_cur * long_dir_;
            corners_world_.at(2) =
                center - 0.5 * wid_cur * wide_dir_ - 0.5 * len_cur * long_dir_;
            corners_world_.at(3) =
                center + 0.5 * wid_cur * wide_dir_ - 0.5 * len_cur * long_dir_;
        }
    }

    bool leftSide = (this->slot_state_(SLOT_CENTER_X) < 0);
    adjustRectOrder(leftSide, corners_world_);

    // 更新占用状态
    bool occupy_filter = false;

    if (occupy_filter) {
        // 占用均值滤波
        int occupy_window_size = 5;
        this->occupies_.push(quad_info->occupy);
        occupy_sum_ += quad_info->occupy;
        LOGD("[PSD_occupy]slot_id: %d, occupies_size: %d", this->GetSlotApaId(), occupies_.size());
        if (occupies_.size() > occupy_window_size) {
            occupy_sum_ -= occupies_.front();
            LOGD("[PSD_occupy]occupy_sum_: %d", occupy_sum_);
            occupies_.pop();
            if (occupy_sum_ / (occupy_window_size * 1.0) >= 0.8) {
                this->occupy_ = 1;
            } else {
                this->occupy_ = 0;
            }
        } else {
            // LOGD("[PSD_occupy]slot%d occupies size: %d", this->GetSlotApaId(), occupies_.size());
            this->occupy_ = 1;
        }
    } else {
        // 直接更新RD占用信息
        this->occupy_ = quad_info->occupy;
    }



    // //***************************************左右判断调整顺序
    // Eigen::Vector2f vehicle_pos = {vehicle_pose.coord.x/1000,vehicle_pose.coord.y/1000};  // 获取车辆当前位置
    // Eigen::Vector2f slot_center = GetSlotCenter().head<2>();  // 获取车位中心 
    // printf("Vehicle Position: (%.2f, %.2f)\n", vehicle_pos.x(), vehicle_pos.y());
    // printf("Slot Center Position: (%.2f, %.2f)\n", slot_center.x(), slot_center.y());
    // // 计算车位相对于车辆的方向
    // Eigen::Vector2f relative_pos = slot_center - vehicle_pos;
    // printf("Relative Position to Vehicle: (%.2f, %.2f)\n", relative_pos.x(), relative_pos.y());
    // // 获取车辆的横向方向向量（车身垂直方向）
    // Eigen::Vector2f vehicle_right;
    // float vehicle_yaw = - vehicle_pose.yaw * M_PI / 180.0;
    // vehicle_right << std::cos(vehicle_yaw + M_PI_2), std::sin(vehicle_yaw + M_PI_2);
    // printf("Vehicle Yaw: %.4f\n", vehicle_yaw);
    // printf("Vehicle Right Direction: (%.4f, %.4f)\n", vehicle_right.x(), vehicle_right.y());
    // // 计算点积，判断车位在车辆的左侧还是右侧
    // float dot_product = relative_pos.dot(vehicle_right);
    // // 判断车位相对于车辆的位置
    // int slot_side = (dot_product < 0) ? -1 : 1;
    // printf("Dot Product: %.4f\n", dot_product);
    // printf("Slot Side: %d (Previous: %d)\n", slot_side, prev_slot_side);
    // // 角点顺序调整
    // if (slot_side != prev_slot_side) {
    //     printf("Slot side changed! Swapping corner points...\n");
    //     std::swap(corners_world_.at(0), corners_world_.at(1));
    //     std::swap(corners_world_.at(2), corners_world_.at(3));
    // }
    // prev_slot_side = slot_side;
    // // 打印调整后的角点
    // for (size_t i = 0; i < corners_world_.size(); ++i) {
    //     printf("Corner[%zu]: (%.2f, %.2f)\n", i, corners_world_.at(i).x(), corners_world_.at(i).y());
    // }
    // //***************************************



    // // 20250306没做左右判断
    // if (type_ == SLOT_TYPE::VERTICALSLOT){
    //     if(this->slot_state_(SLOT_CENTER_X) > 0){
    //          corners_world_.at(0) =
    //             center + 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
    //         corners_world_.at(1) =
    //             center + 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
    //         corners_world_.at(2) =
    //             center - 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
    //         corners_world_.at(3) =
    //             center - 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
    //     }else{
    //         corners_world_.at(0) =
    //             center + 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
    //         corners_world_.at(1) =
    //             center + 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
    //         corners_world_.at(2) =
    //             center - 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
    //         corners_world_.at(3) =
    //             center - 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
    //     }
    // }else if (type_ == SLOT_TYPE::PARALLELSLOT) {
    // // 在平行车位中，长边是车位的宽度方向，宽边是车位的长度方向
    //     if(this->slot_state_(SLOT_CENTER_X) > 0){
    //         corners_world_.at(0) =
    //             center - 0.5 * wid_cur * wide_dir_ + 0.5 * len_cur * long_dir_;
    //         corners_world_.at(1) =
    //             center + 0.5 * wid_cur * wide_dir_ + 0.5 * len_cur * long_dir_;
    //         corners_world_.at(2) =
    //             center + 0.5 * wid_cur * wide_dir_ - 0.5 * len_cur * long_dir_;
    //         corners_world_.at(3) =
    //             center - 0.5 * wid_cur * wide_dir_ - 0.5 * len_cur * long_dir_;
    //     }else{
    //         corners_world_.at(0) =
    //             center + 0.5 * wid_cur * wide_dir_ + 0.5 * len_cur * long_dir_;
    //         corners_world_.at(1) =
    //             center - 0.5 * wid_cur * wide_dir_ + 0.5 * len_cur * long_dir_;
    //         corners_world_.at(2) =
    //             center - 0.5 * wid_cur * wide_dir_ - 0.5 * len_cur * long_dir_;
    //         corners_world_.at(3) =
    //             center + 0.5 * wid_cur * wide_dir_ - 0.5 * len_cur * long_dir_;
    //     }
    // }
    
    
    
    
    

    
    
    // 20250228之前的水平车位计算公式
    
    // else if(type_ == SLOT_TYPE::PARALLELSLOT){
    //     // if (this->slot_state_(SLOT_CENTER_X) > 0){
    //         corners_world_.at(0) =
    //             center - 0.5 * wid_cur * long_dir_ + 0.5 * len_cur * wide_dir_;
    //         corners_world_.at(1) =
    //             center + 0.5 * wid_cur * long_dir_ + 0.5 * len_cur * wide_dir_;
    //         corners_world_.at(2) =
    //             center + 0.5 * wid_cur * long_dir_ - 0.5 * len_cur * wide_dir_;
    //         corners_world_.at(3) =
    //             center - 0.5 * wid_cur * long_dir_ - 0.5 * len_cur * wide_dir_;
    //     // }else{
    //     //     corners_world_.at(0) =
    //     //         center + 0.5 * wid_cur * long_dir_ + 0.5 * len_cur * wide_dir_;
    //     //     corners_world_.at(1) =
    //     //         center - 0.5 * wid_cur * long_dir_ + 0.5 * len_cur * wide_dir_;
    //     //     corners_world_.at(2) =
    //     //         center - 0.5 * wid_cur * long_dir_ - 0.5 * len_cur * wide_dir_;
    //     //     corners_world_.at(3) =
    //     //         center + 0.5 * wid_cur * long_dir_ - 0.5 * len_cur * wide_dir_;
    //     // }
        
    // }  

    // 更新车位状态信息

    this->age_++;
    if (SLOT_STATUS::TENTATIVE == this->status_ &&
        this->age_ > isp_.confirm_age) {
        this->status_ = SLOT_STATUS::CONFIRMED;
    }
    if (this->missing_time_ > 0) --this->missing_time_;
};



void Kalman_filter::adjustRectOrder(bool isleft, std::array<Eigen::Vector3f, 4> cornerswolrd)
{
    // 按照 x 轴排序，先排左边的两个点，再排右边的两个点
    std::sort(cornerswolrd.begin(), cornerswolrd.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
        return a.x() < b.x();
    });

    // 左侧两个点 (left1, left2)，右侧两个点 (right1, right2)
    std::array<Eigen::Vector3f, 2> left = {cornerswolrd[0], cornerswolrd[1]};
    std::array<Eigen::Vector3f, 2> right = {cornerswolrd[2], cornerswolrd[3]};

    // 按 y 轴排序，确保 top 和 bottom
    std::sort(left.begin(), left.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
        return a.y() > b.y();  // y 值大的在前
    });

    std::sort(right.begin(), right.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
        return a.y() > b.y();  // y 值大的在前
    });

    if (isleft) {
        // 当长方形位于原点左侧
        cornerswolrd = {right[1], right[0], left[0], left[1]}; // A, B, C, D
    } else {
        // 当长方形位于原点右侧
        cornerswolrd = {left[1], left[0], right[0], right[1]}; // A, B, C, D
    }
}

























// // Origin 448 Update
// void Kalman_filter::Update(const QuadInfoPtr& quad_info) {
//     std::cout<<"Kalman_filter update!"<<std::endl;
//     if (!this->pre_update(quad_info)) {
//         return;
//     }
//     Eigen::Vector2f temp_center;
//     temp_center << this->slot_state_(SLOT_CENTER_X), this->slot_state_(SLOT_CENTER_Y);
//     // 更新预测矩阵，状态量不变，P需要调整
//     P_ = F_ * P_ * F_.transpose() + Q_ * fmax(1.0, delta_frame_cnt_ / 10.0);

//     // 更新观测矩阵
//     float hsa = 0.5 * std::sin(GetSlotLongAngle());
//     float hca = 0.5 * std::cos(GetSlotLongAngle());
//     float hsb = 0.5 * std::sin(GetSlotWideAngle());
//     float hcb = 0.5 * std::cos(GetSlotWideAngle());
//     float length = GetSlotLength();
//     float width = GetSlotWidth();
//     H_.row(TOP_LEFT_X) << 1, 0, -length * hsa, -width * hsb, hca, hcb;
//     H_.row(TOP_LEFT_Y) << 0, 1, length * hca, width * hcb, hsa, hsb;
//     H_.row(TOP_RIGHT_X) << 1, 0, -length * hsa, width * hsb, hca, -hcb;
//     H_.row(TOP_RIGHT_Y) << 0, 1, length * hca, -width * hcb, hsa, -hsb;
//     H_.row(BOTTOM_RIGHT_X) << 1, 0, length * hsa, width * hsb, -hca, -hcb;
//     H_.row(BOTTOM_RIGHT_Y) << 0, 1, -length * hca, -width * hcb, -hsa, -hsb;
//     H_.row(BOTTOM_LEFT_X) << 1, 0, length * hsa, -width * hsb, -hca, hcb;
//     H_.row(BOTTOM_LEFT_Y) << 0, 1, -length * hca, width * hcb, -hsa, hsb;

//     // 更新测量噪声矩阵
//     R_.setIdentity();
//     const float valid_measure_thr = 200;  // unit: mm
//     for (size_t i = 0UL; i < 4UL; ++i) {
//         size_t cur = (i + rot_idx_) % max_rot_idx_;
//         const auto& q = quad_info->corners_extended.col(cur);
//         const auto& c = this->corners_world_.at(i);
//         float dist = (q.head<2>() - c.head<2>()).norm();

//         Eigen::Index idx = 2 * i;
//         R_.block<2, 2>(idx, idx) = quad_info->cov_extended.at(cur);
//         if (dist > isp_.valid_measure_thr) {
//             R_.block<2, 2>(idx, idx) *= isp_.invalid_measure_enlarge_ratio;
//         }
//     }

//     Eigen::Matrix<float, SLOT_MEASURE_SIZE, 1> measure;
//     const auto& m = quad_info->corners_extended;
//     measure << m.col(0UL + rot_idx_).x(), m.col(0UL + rot_idx_).y(),
//         m.col((1UL + rot_idx_) % max_rot_idx_).x(),
//         m.col((1UL + rot_idx_) % max_rot_idx_).y(),
//         m.col((2UL + rot_idx_) % max_rot_idx_).x(),
//         m.col((2UL + rot_idx_) % max_rot_idx_).y(),
//         m.col((3UL + rot_idx_) % max_rot_idx_).x(),
//         m.col((3UL + rot_idx_) % max_rot_idx_).y();

//     // 将Predict()得到的估计值，转换到测量坐标系
//     Eigen::Matrix<float, SLOT_MEASURE_SIZE, 1> measure_prediction;
//     measure_prediction(TOP_LEFT_X) = corners_world_.at(0).x();
//     measure_prediction(TOP_LEFT_Y) = corners_world_.at(0).y();
//     measure_prediction(TOP_RIGHT_X) = corners_world_.at(1).x();
//     measure_prediction(TOP_RIGHT_Y) = corners_world_.at(1).y();
//     measure_prediction(BOTTOM_RIGHT_X) = corners_world_.at(2).x();
//     measure_prediction(BOTTOM_RIGHT_Y) = corners_world_.at(2).y();
//     measure_prediction(BOTTOM_LEFT_X) = corners_world_.at(3).x();
//     measure_prediction(BOTTOM_LEFT_Y) = corners_world_.at(3).y();
//     Eigen::Matrix<float, SLOT_MEASURE_SIZE, SLOT_MEASURE_SIZE> S;
//     S = H_ * P_ * H_.transpose() + R_;
//     Eigen::Matrix<float, SLOT_STATE_SIZE, SLOT_MEASURE_SIZE> kalman_gain;
//     kalman_gain = P_ * H_.transpose() * S.inverse();
//     Eigen::Matrix<float, SLOT_MEASURE_SIZE, 1> innovation;
//     innovation = (measure - measure_prediction)/1000;

//     if (innovation.norm() > isp_.invalid_innovation_thr &&
//         delta_frame_cnt_ < isp_.delta_frame_thr) {
//         return;
//     }

//     auto normalize_angle = [](const double& ang) {
//         double angle = ang;
//         if (angle >= M_PI) {
//             while (angle > M_PI) {
//                 angle -= 2 * M_PI;
//             }
//         } else if (angle < -M_PI) {
//             while (angle < -M_PI) {
//                 angle += 2 * M_PI;
//             }
//         }
//         return angle;
//     };

//     auto align_angle = [](const double& ang_diff) {
//         double angle = ang_diff;
//         if (angle < -M_PI_2) {
//             angle += M_PI;
//         } else if (angle >= M_PI_2) {
//             angle -= M_PI;
//         }
//         return angle;
//     };

//     innovation(SLOT_ALPHA) = normalize_angle(innovation(SLOT_ALPHA));
//     innovation(SLOT_ALPHA) = align_angle(innovation(SLOT_ALPHA));

//     innovation(SLOT_BETA) = normalize_angle(innovation(SLOT_BETA));
//     innovation(SLOT_BETA) = align_angle(innovation(SLOT_BETA));

//     // 更新状态量和矩阵
//     auto original_length = this->GetSlotLength();
//     Eigen::Matrix<float, SLOT_STATE_SIZE, 1> change;
//     auto test = kalman_gain * innovation;
//     change << test[0], test[1], 0.0, 0.0, 0.0, 0.0;
//     this->slot_state_ = this->slot_state_ + change;

//     if (valid_quad_cnt_ < 3) {
//         this->slot_state_(SLOT_LENGTH) = original_length;
//     }
//     P_ = P_ - kalman_gain * H_ * P_;

//     this->slot_state_(SLOT_ALPHA) = normalize_angle(slot_state_(SLOT_ALPHA));
//     this->slot_state_(SLOT_BETA) = normalize_angle(slot_state_(SLOT_BETA));

//     // 更新车位角点信息
//     auto center = GetSlotCenter();
//     auto len_cur = GetSlotLength() * 1000;
//     auto wid_cur = GetSlotWidth() * 1000;
 
//     long_dir_ << std::cos(GetSlotLongAngle()), std::sin(GetSlotLongAngle()),0.0;
//     wide_dir_ << std::cos(GetSlotWideAngle()), std::sin(GetSlotWideAngle()),0.0;

//     if (type_ == SLOT_TYPE::VERTICALSLOT){
//         // if(this->slot_state_(SLOT_CENTER_X) > 0){
//              corners_world_.at(0) =
//                 center + 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
//             corners_world_.at(1) =
//                 center + 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
//             corners_world_.at(2) =
//                 center - 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
//             corners_world_.at(3) =
//                 center - 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
//         // }else{
//         //     corners_world_.at(0) =
//         //         center + 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
//         //     corners_world_.at(1) =
//         //         center + 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
//         //     corners_world_.at(2) =
//         //         center - 0.5 * len_cur * wide_dir_ - 0.5 * wid_cur * long_dir_;
//         //     corners_world_.at(3) =
//         //         center - 0.5 * len_cur * wide_dir_ + 0.5 * wid_cur * long_dir_;
//         // }
       
        
            
//     }else if(type_ == SLOT_TYPE::PARALLELSLOT){
//         // if (this->slot_state_(SLOT_CENTER_X) > 0){
//             corners_world_.at(0) =
//                 center - 0.5 * wid_cur * long_dir_ + 0.5 * len_cur * wide_dir_;
//             corners_world_.at(1) =
//                 center + 0.5 * wid_cur * long_dir_ + 0.5 * len_cur * wide_dir_;
//             corners_world_.at(2) =
//                 center + 0.5 * wid_cur * long_dir_ - 0.5 * len_cur * wide_dir_;
//             corners_world_.at(3) =
//                 center - 0.5 * wid_cur * long_dir_ - 0.5 * len_cur * wide_dir_;
//         // }else{
//         //     corners_world_.at(0) =
//         //         center + 0.5 * wid_cur * long_dir_ + 0.5 * len_cur * wide_dir_;
//         //     corners_world_.at(1) =
//         //         center - 0.5 * wid_cur * long_dir_ + 0.5 * len_cur * wide_dir_;
//         //     corners_world_.at(2) =
//         //         center - 0.5 * wid_cur * long_dir_ - 0.5 * len_cur * wide_dir_;
//         //     corners_world_.at(3) =
//         //         center + 0.5 * wid_cur * long_dir_ - 0.5 * len_cur * wide_dir_;
//         // }
        
//     }  

//     // 更新车位状态信息
//     this->age_++;
//     if (SLOT_STATUS::TENTATIVE == this->status_ &&
//         this->age_ > isp_.confirm_age) {
//         this->status_ = SLOT_STATUS::CONFIRMED;
//     }
//     if (this->missing_time_ > 0) --this->missing_time_;
// };
