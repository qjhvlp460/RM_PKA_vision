#include "rm_auto_aim/solver/utils/extended_kalman_filter.hpp"

#include <cmath>

namespace rm_auto_aim {

ExtendedKalmanFilter::MatXX ExtendedKalmanFilter::computeF(const VecX& x, double dt) {
    // 使用中心差分法计算状态转移雅可比矩阵，精度更高
    MatXX F = MatXX::Identity(n_states_, n_states_);
    const double eps = 1e-5;

    for (int i = 0; i < n_states_; ++i) {
        VecX x_plus = x;
        VecX x_minus = x;
        x_plus(i) += eps;
        x_minus(i) -= eps;
        
        VecX f_plus = f_(x_plus, dt);
        VecX f_minus = f_(x_minus, dt);
        
        F.col(i) = (f_plus - f_minus) / (2.0 * eps);
    }
    return F;
}

Eigen::Matrix<double, 4, Eigen::Dynamic> ExtendedKalmanFilter::computeH(const VecX& x) {
    Eigen::Matrix<double, 4, Eigen::Dynamic> H = 
        Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, n_states_);
    const double eps = 1e-5;

    for (int i = 0; i < n_states_; ++i) {
        VecX x_plus = x;
        VecX x_minus = x;
        x_plus(i) += eps;
        x_minus(i) -= eps;
        
        Eigen::Vector4d h_plus = h_(x_plus);
        Eigen::Vector4d h_minus = h_(x_minus);
        
        H.col(i) = (h_plus - h_minus) / (2.0 * eps);
    }
    return H;
}

ExtendedKalmanFilter::VecX ExtendedKalmanFilter::update(const Eigen::Vector4d& z) {
    if (!initialized_) return x_;

    auto H = computeH(x_);
    Eigen::Vector4d innovation = z - h_(x_);

    // 优化角度归一化：使用标准库函数确保yaw残差在[-π, π]范围内
    innovation(3) = std::remainder(innovation(3), 2.0 * M_PI);

    Eigen::Matrix4d S = H * P_ * H.transpose() + R_;
    auto K = P_ * H.transpose() * S.inverse();

    x_ += K * innovation;
    
    MatXX I = MatXX::Identity(n_states_, n_states_);
    MatXX I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();

    return x_;
}// ... (头部包含和命名空间保持不变)

ExtendedKalmanFilter::MatXX ExtendedKalmanFilter::computeF(const VecX& x, double dt) {
    // 使用中心差分法计算状态转移雅可比矩阵，精度更高
    MatXX F = MatXX::Identity(n_states_, n_states_);
    const double eps = 1e-5;

    for (int i = 0; i < n_states_; ++i) {
        VecX x_plus = x;
        VecX x_minus = x;
        x_plus(i) += eps;
        x_minus(i) -= eps;
        
        VecX f_plus = f_(x_plus, dt);
        VecX f_minus = f_(x_minus, dt);
        
        F.col(i) = (f_plus - f_minus) / (2.0 * eps);
    }
    return F;
}

Eigen::Matrix<double, 4, Eigen::Dynamic> ExtendedKalmanFilter::computeH(const VecX& x) {
    Eigen::Matrix<double, 4, Eigen::Dynamic> H = 
        Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, n_states_);
    const double eps = 1e-5;

    for (int i = 0; i < n_states_; ++i) {
        VecX x_plus = x;
        VecX x_minus = x;
        x_plus(i) += eps;
        x_minus(i) -= eps;
        
        Eigen::Vector4d h_plus = h_(x_plus);
        Eigen::Vector4d h_minus = h_(x_minus);
        
        H.col(i) = (h_plus - h_minus) / (2.0 * eps);
    }
    return H;
}

ExtendedKalmanFilter::VecX ExtendedKalmanFilter::update(const Eigen::Vector4d& z) {
    if (!initialized_) return x_;

    auto H = computeH(x_);
    Eigen::Vector4d innovation = z - h_(x_);

    // 优化角度归一化：使用标准库函数确保yaw残差在[-π, π]范围内
    innovation(3) = std::remainder(innovation(3), 2.0 * M_PI);

    Eigen::Matrix4d S = H * P_ * H.transpose() + R_;
    auto K = P_ * H.transpose() * S.inverse();

    x_ += K * innovation;
    
    MatXX I = MatXX::Identity(n_states_, n_states_);
    MatXX I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();

    return x_;
 }
}