#include "rm_hardware_driver/camera_driver_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

namespace rm_auto_aim {


bool PnPSolver::solve(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec, double& yaw) {
    auto image_points = armor.corners();
    auto object_points = getObjectPoints(armor.type);
    
    std::vector<cv::Mat> rotation_vecs, translation_vecs;
    bool success = cv::solvePnPGeneric(
        object_points, image_points,
        camera_matrix_, dist_coeffs_,
        rotation_vecs, translation_vecs,
        false, cv::SOLVEPNP_IPPE_SQUARE);
    
    if (!success || rotation_vecs.empty()) {
        return false;
    }
    
    // 优先选择Z坐标为正（相机前方）且重投影误差较小的解
    int selected_index = 0;
    double min_combined_error = std::numeric_limits<double>::max();
    
    for (size_t idx = 0; idx < rotation_vecs.size(); ++idx) {
        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(object_points, rotation_vecs[idx], translation_vecs[idx],
                         camera_matrix_, dist_coeffs_, reprojected_points);
        
        double reprojection_error = 0.0;
        for (size_t pt_idx = 0; pt_idx < image_points.size(); ++pt_idx) {
            double dx = image_points[pt_idx].x - reprojected_points[pt_idx].x;
            double dy = image_points[pt_idx].y - reprojected_points[pt_idx].y;
            reprojection_error += std::sqrt(dx * dx + dy * dy);
        }
        reprojection_error /= image_points.size();
        
        // 检查解是否在相机前方（Z>0）
        double z_coordinate = translation_vecs[idx].at<double>(2);
        double forward_penalty = (z_coordinate > 0) ? 0.0 : 1000.0;
        
        double combined_error = reprojection_error + forward_penalty;
        
        if (combined_error < min_combined_error) {
            min_combined_error = combined_error;
            selected_index = static_cast<int>(idx);
        }
    }
    
    rvec = rotation_vecs[selected_index];
    tvec = translation_vecs[selected_index];
    
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    yaw = extractYaw(rotation_matrix);
    
    return true;
}

double PnPSolver::calculateDistance(const cv::Mat& tvec) {
    double x = tvec.at<double>(0);
    double y = tvec.at<double>(1);
    double z = tvec.at<double>(2);
    return std::hypot(x, y, z);  // 使用std::hypot计算欧氏距离
}
}