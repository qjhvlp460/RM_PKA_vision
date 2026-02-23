#pragma once

#include <rclcpp/rclcpp.hpp>

#include "rm_auto_aim/solver/armor_tracker.hpp"
#include "rm_auto_aim/solver/utils/trajectory_compensator.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"

namespace rm_auto_aim {

/**
 * @brief 装甲板解算ROS2节点
 *
 * 订阅装甲板检测结果，执行：
 * 1. EKF跟踪与预测
 * 2. 瞄准目标选择（反陀螺策略）
 * 3. 弹道补偿
 * 4. 输出云台控制指令
 */
class ArmorSolverNode : public rclcpp::Node {
public:
    explicit ArmorSolverNode(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief 装甲板检测结果回调
     */
    void armorsCallback(const rm_interfaces::msg::Armors::ConstSharedPtr& msg);

    /**
     * @brief 声明并加载参数
     */
    void declareParameters();
    void loadParams();

    /**
     * @brief 根据EKF状态计算瞄准点
     * @param state EKF状态向量
     * @param v_yaw 目标角速度
     * @return 瞄准点三维坐标
     */
    Eigen::Vector3d calcAimPoint(const Eigen::VectorXd& state, double v_yaw);

    /**
     * @brief 判断是否为小陀螺状态
     */
    bool isSmallGyro(double v_yaw) const;

    /**
     * @brief 选择最优装甲板（反陀螺策略）
     * @param state EKF状态向量
     * @param armors_num 目标装甲板数量
     * @return 选中的装甲板三维坐标
     */
    Eigen::Vector3d selectBestArmor(
        const Eigen::VectorXd& state, int armors_num);

    // 跟踪器
    std::unique_ptr<ArmorTracker> tracker_;

    // 弹道补偿器
    TrajectoryCompensator trajectory_compensator_;
    ManualCompensator manual_compensator_;

    // 订阅与发布
    rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;

    // 时间戳管理
    rclcpp::Time last_time_;
    bool first_frame_ = true;

    // 弹道参数
    double bullet_speed_ = 30.0;
    double gravity_ = 9.82;
    double resistance_ = 0.092;

    // 反陀螺参数
    double max_tracking_v_yaw_ = 60.0;   // 角速度阈值
    double side_angle_ = 15.0;            // 切换角度阈值(度)
    double coming_angle_ = 1.222;         // 小陀螺出现角 (70°)
    double leaving_angle_ = 0.524;        // 小陀螺消失角 (30°)

    // 调试
    bool debug_ = false;
};

}  // namespace rm_auto_aim
