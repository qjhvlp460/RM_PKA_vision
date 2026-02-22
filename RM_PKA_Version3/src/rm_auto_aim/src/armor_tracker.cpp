#include "rm_auto_aim/solver/armor_tracker.hpp"

#include <cmath>
#include <limits>

namespace rm_auto_aim {

Eigen::VectorXd ArmorTracker::predictFunc(const Eigen::VectorXd& x, double dt) {
    // 使用更简洁的向量运算实现匀速运动模型
    Eigen::VectorXd x_pred = x;
    for (int i = 0; i < 5; ++i) {
        x_pred(2 * i) += x(2 * i + 1) * dt;  // 位置更新
    }
    return x_pred;
}

Eigen::Vector4d ArmorTracker::measureFunc(const Eigen::VectorXd& x) {
    Eigen::Vector4d z;
    double cos_yaw = std::cos(x(6));
    double sin_yaw = std::sin(x(6));
    
    z(0) = x(0) - cos_yaw * x(8);    // x_armor
    z(1) = x(2) - sin_yaw * x(8);    // y_armor
    z(2) = x(4) + x(9);              // z_armor
    z(3) = x(6);                     // yaw
    return z;
}

int ArmorTracker::matchArmor(const rm_interfaces::msg::Armors& armors) {
    auto state = ekf_->state();
    Eigen::Vector4d predicted = measureFunc(state);

    double min_distance = std::numeric_limits<double>::max();
    int best_match = -1;

    for (size_t idx = 0; idx < armors.armors.size(); ++idx) {
        const auto& armor = armors.armors[idx];
        
        Eigen::Vector3d det_pos(
            armor.pose.position.x,
            armor.pose.position.y,
            armor.pose.position.z
        );
        Eigen::Vector3d pred_pos(predicted(0), predicted(1), predicted(2));
        
        double distance = (det_pos - pred_pos).norm();
        
        double detected_yaw = 2.0 * std::atan2(armor.pose.orientation.z,
                                                armor.pose.orientation.w);
        double yaw_diff = std::abs(std::remainder(detected_yaw - predicted(3), 2.0 * M_PI));

        if (distance < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
            if (distance < min_distance) {
                min_distance = distance;
                best_match = static_cast<int>(idx);
            }
        }
    }
    return best_match;
}
}