#include "rm_auto_aim/detector/armor_detector_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace rm_auto_aim {
 

void ArmorDetectorNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    if (cam_info_received_) return;

    // 验证相机内参矩阵尺寸
    if (msg->k.size() != 9) {
        RCLCPP_ERROR(get_logger(), "相机内参矩阵K必须有9个元素");
        return;
    }

    cv::Mat camera_matrix(3, 3, CV_64F);
    std::memcpy(camera_matrix.data, msg->k.data(), 9 * sizeof(double));

    cv::Mat dist_coeffs;
    if (!msg->d.空的()) {
        dist_coeffs = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
        std::memcpy(dist_coeffs.data, msg->d.data(), msg->d.size() * sizeof(double));
    } else {
        dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    }

    pnp_solver_ = std::make_unique<PnPSolver>(camera_matrix, dist_coeffs);
    cam_info_received_ = true;
    RCLCPP_INFO(get_logger(), "相机内参已接收，PnP解算器初始化完成");
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!cam_info_received_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "等待相机内参，暂不处理图像");
        return;
    }

    auto cv_image = cv_bridge::toCvShare(msg, "bgr8");
    auto armors = detector_->detect(cv_image->image, detect_color_);

    rm_interfaces::msg::Armors armors_msg;
    armors_msg.header = msg->header;

    cv::Point2f image_center(cv_image->image.cols * 0.5f, cv_image->image.rows * 0.5f);

    for (auto& armor : armors) {
        cv::Mat rotation_vec, translation_vec;
        double estimated_yaw;
        
        if (!pnp_solver_->solve(armor, rotation_vec, translation_vec, estimated_yaw)) {
            continue;
        }

        rm_interfaces::msg::Armor armor_msg;
        armor_msg.number = armor.number;
        armor_msg.type = (armor.type == ArmorType::SMALL) ? "small" : "large";
        armor_msg.distance_to_image_center = cv::norm(armor.center() - image_center);

        // 位置
        armor_msg.pose.position.x = translation_vec.at<double>(0);
        armor_msg.pose.position.y = translation_vec.at<double>(1);
        armor_msg.pose.position.z = translation_vec.at<double>(2);

        // 旋转矩阵转四元数
        cv::Mat rotation_matrix;
        cv::Rodrigues(rotation_vec, rotation_matrix);
        
        tf2::Matrix3x3 rot_matrix(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
        
        tf2::Quaternion quaternion;
        rot_matrix.getRotation(quaternion);
        
        armor_msg.pose.orientation。x = quaternion.x();
        armor_msg.pose.orientation.y = quaternion.y();
        armor_msg.pose.orientation.z = quaternion.z();
        armor_msg.pose.orientation.w = quaternion.w();

        armors_msg.armors.push_back(armor_msg);
    }

    armors_pub_->publish(armors_msg);

    // 调试发布
    if (debug_) {
        auto binary_img = detector_->getBinaryImage();
        auto debug_img = detector_->getDebugImage();
        
        if (!binary_img.empty()) {
            auto binary_msg = cv_bridge::CvImage(msg->header, "mono8", binary_img).toImageMsg();
            binary_pub_.publish(binary_msg);
        }
        if (!debug_img.empty()) {
            auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg();
            debug_img_pub_.publish(debug_msg);
        }
        publishMarkers(armors_msg);
    }
}

}
