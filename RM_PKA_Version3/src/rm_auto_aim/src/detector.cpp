#include "rm_auto_aim/detector/detector.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace rm_auto_aim {
  

Color ArmorDetector::classifyLightColor(const cv::Mat& input, const cv::RotatedRect& rect) const {
    // 获取灯条区域的最小外接矩形
    cv::Rect bounding_rect = rect.boundingRect();
    bounding_rect &= cv::Rect(0, 0, input.cols, input.rows);
    
    if (bounding_rect.area() <= 0) {
        return Color::RED;
    }

    cv::Mat roi = input(bounding_rect);
    cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    
    cv::Point2f offset(static_cast<float>(bounding_rect.x), 
                       static_cast<float>(bounding_rect.y));
    cv::Point2f adjusted_pts[4];
    rect.points(adjusted_pts);
    
    std::vector<cv::Point> polygon_pts;
    for (int i = 0; i < 4; ++i) {
        adjusted_pts[i] -= offset;
        polygon_pts.emplace_back(static_cast<int>(adjusted_pts[i].x), 
                                 static_cast<int>(adjusted_pts[i].y));
    }
    
    cv::fillConvexPoly(mask, polygon_pts, cv::Scalar(255));
    
    cv::Scalar mean_color = cv::mean(roi, mask);
    double blue_intensity = mean_color[0];
    double red_intensity = mean_color[2];
    
    if (red_intensity - blue_intensity > params_.light_color_diff_thresh) {
        return Color::RED;
    } else if (blue_intensity - red_intensity > params_.light_color_diff_thresh) {
        return Color::BLUE;
    }
    
    return (red_intensity > blue_intensity) ? Color::RED : Color::BLUE;
}

std::vector<Armor> ArmorDetector::matchArmors(const std::vector<Light>& lights) {
    std::vector<Armor> detected_armors;
    size_t light_count = lights.size();
    
    for (size_t i = 0; i < light_count; ++i) {
        for (size_t j = i + 1; j < light_count; ++j) {
            const Light& left_light = lights[i];
            const Light& right_light = lights[j];
            
            if (!isValidArmor(left_light, right_light)) {
                continue;
            }
            
            // 检查是否被其他灯条干扰
            bool has_interference = false;
            float left_x = left_light.center.x;
            float right_x = right_light.center.x;
            
            for (size_t k = 0; k < light_count; ++k) {
                if (k == i || k == j) continue;
                float current_x = lights[k].center.x;
                if (current_x > std::min(left_x, right_x) && 
                    current_x < std::max(left_x, right_x)) {
                    has_interference = true;
                    break;
                }
            }
            
            if (has_interference) continue;
            
            Armor new_armor;
            new_armor.left_light = left_light;
            new_armor.right_light = right_light;
            
            double center_distance = cv::norm(left_light.center - right_light.center);
            double average_length = (left_light.length + right_light.length) * 0.5;
            double distance_ratio = center_distance / average_length;
            
            if (distance_ratio < params_.armor_max_small_center_distance) {
                new_armor.type = ArmorType::SMALL;
            } else {
                new_armor.type = ArmorType::LARGE;
            }
            
            new_armor.number = "unknown";
            detected_armors.push_back(new_armor);
        }
    }
    
    return detected_armors;
}
}