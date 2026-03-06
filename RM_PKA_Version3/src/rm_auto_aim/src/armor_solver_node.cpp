#include "rm_auto_aim/solver/armor_solver_node.hpp"

#include <cmath>

namespace rm_auto_aim {

ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions& options)
    : Node("armor_solver", options)
{
    RCLCPP_INFO(get_logger(), "ArmorSolverNode 初始化中...");

    declareParameters();
    loadParams();

    // 初始化跟踪器
    tracker_ = std::make_unique<ArmorTracker>();
    // 从参数加载tracker配置（在loadParams中完成）

    // 订阅装甲板检测结果
    armors_sub_ = this->create_subscription<rm_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),
        std::bind(&ArmorSolverNode::armorsCallback, this, std::placeholders::_1));

    // 发布目标信息
    target_pub_ = this->create_publisher<rm_interfaces::msg::Target>(
        "/solver/target", rclcpp::SensorDataQoS());

    // 发布云台控制命令
    gimbal_cmd_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
        "/solver/gimbal_cmd", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "ArmorSolverNode 初始化完成");
}

void ArmorSolverNode::declareParameters() {
    // EKF过程噪声
    this->declare_parameter("ekf.sigma2_q_x", 0.008);
    this->declare_parameter("ekf.sigma2_q_y", 0.008);
    this->declare_parameter("ekf.sigma2_q_z", 0.008);
    this->declare_parameter("ekf.sigma2_q_yaw", 1.30);
    this->declare_parameter("ekf.sigma2_q_r", 98.0);
    // EKF观测噪声
    this->declare_parameter("ekf.r_x", 0.0005);
    this->declare_parameter("ekf.r_y", 0.0005);
    this->declare_parameter("ekf.r_z", 0.0005);
    this->declare_parameter("ekf.r_yaw", 0.005);
    // 跟踪器参数
    this->declare_parameter("tracker.max_match_distance", 0.5);
    this->declare_parameter("tracker.max_match_yaw_diff", 0.67);
    this->declare_parameter("tracker.tracking_thres", 3);
    this->declare_parameter("tracker.lost_time_thres", 3.05);
    // 弹道参数
    this->declare_parameter("solver.bullet_speed", 30.0);
    this->declare_parameter("solver.gravity", 9.82);
    this->declare_parameter("solver.resistance", 0.092);
    // 反陀螺参数
    this->declare_parameter("solver.max_tracking_v_yaw", 60.0);
    this->declare_parameter("solver.side_angle", 15.0);
    this->declare_parameter("solver.coming_angle", 1.222);
    this->declare_parameter("solver.leaving_angle", 0.524);
    // 调试
    this->declare_parameter("debug", false);
}

void ArmorSolverNode::loadParams() {
    // 弹道参数
    bullet_speed_ = this->get_parameter("solver.bullet_speed").as_double();
    gravity_ = this->get_parameter("solver.gravity").as_double();
    resistance_ = this->get_parameter("solver.resistance").as_double();
    trajectory_compensator_.setParams(bullet_speed_, gravity_, resistance_);

    // 反陀螺
    max_tracking_v_yaw_ = this->get_parameter("solver.max_tracking_v_yaw").as_double();
    side_angle_ = this->get_parameter("solver.side_angle").as_double();
    coming_angle_ = this->get_parameter("solver.coming_angle").as_double();
    leaving_angle_ = this->get_parameter("solver.leaving_angle").as_double();

    debug_ = this->get_parameter("debug").as_bool();
}

void ArmorSolverNode::armorsCallback(
    const rm_interfaces::msg::Armors::ConstSharedPtr& msg)
{
    // 计算dt
    rclcpp::Time now = msg->header.stamp;
    double dt = 0.01;  // 默认10ms
    if (!first_frame_) {
        dt = (now - last_time_).seconds();
        if (dt <= 0 || dt > 1.0) dt = 0.01;
    }
    first_frame_ = false;
    last_time_ = now;

    // 更新跟踪器（含EKF预测+更新）
    tracker_->update(*msg, dt);

    // 构造Target消息
    rm_interfaces::msg::Target target_msg;
    target_msg.header = msg->header;

    auto tracker_state = tracker_->state();
    if (tracker_state == TrackerState::TRACKING ||
        tracker_state == TrackerState::TEMP_LOST) {
        target_msg.tracking = true;
        target_msg.id = tracker_->trackedId();
        target_msg.armors_num = tracker_->targetArmorsNum();

        auto state = tracker_->getState();
        // EKF状态: [xc, v_xc, yc, v_yc, zc, v_zc, yaw, v_yaw, r, d_zc]
        target_msg.position.x = state(0);
        target_msg.position.y = state(2);
        target_msg.position.z = state(4);
        target_msg.velocity.x = state(1);
        target_msg.velocity.y = state(3);
        target_msg.velocity.z = state(5);
        target_msg.yaw = state(6);
        target_msg.v_yaw = state(7);
        target_msg.radius_1 = state(8);
        target_msg.d_zc = state(9);

        // 计算瞄准点
        double v_yaw = state(7);
        Eigen::Vector3d aim_point = calcAimPoint(state, v_yaw);

        // 弹道补偿：计算pitch
        double compensated_pitch = trajectory_compensator_.compensate(
            aim_point.x(), aim_point.y(), aim_point.z());

        // 计算yaw
        double yaw_cmd = std::atan2(aim_point.x(), aim_point.z());

        // 手动补偿
        double dist = aim_point.norm();
        auto manual_comp = manual_compensator_.getCompensation(dist);
        compensated_pitch += manual_comp.pitch_offset;
        yaw_cmd += manual_comp.yaw_offset;

        // 发布云台控制命令
        rm_interfaces::msg::GimbalCmd gimbal_cmd;
        gimbal_cmd.header = msg->header;
        gimbal_cmd.yaw = yaw_cmd;
        gimbal_cmd.pitch = compensated_pitch;
        gimbal_cmd.fire = (tracker_state == TrackerState::TRACKING);
        gimbal_cmd_pub_->publish(gimbal_cmd);

    } else {
        target_msg.tracking = false;
    }

    target_pub_->publish(target_msg);
}

Eigen::Vector3d ArmorSolverNode::calcAimPoint(
    const Eigen::VectorXd& state, double v_yaw)
{
    if (isSmallGyro(v_yaw)) {
        // 小陀螺模式：选择最优装甲板
        return selectBestArmor(state, tracker_->targetArmorsNum());
    }

    // 直接瞄准预测的装甲板位置
    double xc = state(0), yc = state(2), zc = state(4);
    double yaw = state(6), r = state(8), d_zc = state(9);

    Eigen::Vector3d aim;
    aim.x() = xc - r * std::cos(yaw);
    aim.y() = yc - r * std::sin(yaw);
    aim.z() = zc + d_zc;

    return aim;
}

bool ArmorSolverNode::isSmallGyro(double v_yaw) const {
    return std::abs(v_yaw) > max_tracking_v_yaw_;
}

Eigen::Vector3d ArmorSolverNode::selectBestArmor(
    const Eigen::VectorXd& state, int armors_num)
{
    double xc = state(0), yc = state(2), zc = state(4);
    double yaw = state(6), r = state(8), d_zc = state(9);

    // 计算所有装甲板位置
    double angle_step = 2.0 * M_PI / armors_num;
    double min_yaw_diff = std::numeric_limits<double>::max();
    Eigen::Vector3d best_point;

    for (int i = 0; i < armors_num; i++) {
        double armor_yaw = yaw + i * angle_step;

        // 归一化到[-pi, pi]
        while (armor_yaw > M_PI) armor_yaw -= 2 * M_PI;
        while (armor_yaw < -M_PI) armor_yaw += 2 * M_PI;

        Eigen::Vector3d armor_pos;
        armor_pos.x() = xc - r * std::cos(armor_yaw);
        armor_pos.y() = yc - r * std::sin(armor_yaw);
        armor_pos.z() = zc + ((i % 2 == 0) ? d_zc : -d_zc);

        // 选择正对相机的那块装甲板（yaw最接近0的）
        double yaw_to_cam = std::atan2(armor_pos.x(), armor_pos.z());
        double yaw_diff = std::abs(yaw_to_cam);

        if (yaw_diff < min_yaw_diff) {
            min_yaw_diff = yaw_diff;
            best_point = armor_pos;
        }
    }

    return best_point;
}

}  

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorSolverNode)
