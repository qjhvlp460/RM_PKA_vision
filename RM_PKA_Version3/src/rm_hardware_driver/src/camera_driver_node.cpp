#include "rm_hardware_driver/camera_driver_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <thread>
#include <string>
#include <vector>
#include <rclcpp_components/register_node_macro.hpp>

namespace rm_auto_aim {

CameraDriverNode::CameraDriverNode(const rclcpp::NodeOptions& options)
    : Node("camera_driver", options),
      camera_pub_(image_transport::create_camera_publisher(this, "/image_raw")),
      running_(false)
{
    RCLCPP_INFO(this->get_logger(), "开始初始化 CameraDriverNode...");

    // 步骤1：声明并获取所有节点参数
    if (!initializeParameters()) {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败，节点即将退出。");
        return; // 关键参数失败，不继续初始化
    }

    // 步骤2：加载并配置相机内参信息
    if (!loadAndSetupCameraInfo()) {
        RCLCPP_WARN(this->get_logger(), "相机内参配置不完整，PnP解算可能不准确。");
        // 仍可继续，但功能可能受限
    }

    // 步骤3：尝试打开视频源（相机或视频文件）
    if (!openVideoSource()) {
        RCLCPP_FATAL(this->get_logger(), "无法打开指定的视频源，节点启动失败。");
        return; // 视频源打开失败，节点无法工作
    }

    // 步骤4：成功打开后，启动图像采集线程
    running_ = true;
    capture_thread_ = std::thread(&CameraDriverNode::captureLoop, this);
    RCLCPP_INFO(this->get_logger(), "CameraDriverNode 初始化完成，正在从 %s 采集图像。",
                video_path_.empty() ? ("相机 ID: " + std::to_string(camera_id_)).c_str() : video_path_.c_str());
}

CameraDriverNode::~CameraDriverNode() {
    // 步骤1：通知采集线程退出
    running_ = false;
    // 步骤2：等待采集线程结束
    if (capture_thread_.joinable()) {
        capture_thread_.join();
        RCLCPP_DEBUG(this->get_logger(), "图像采集线程已安全退出。");
    }
    // 步骤3：释放相机/视频资源
    if (cap_.isOpened()) {
        cap_.release();
        RCLCPP_DEBUG(this->get_logger(), "视频捕获设备已释放。");
    }
}

bool CameraDriverNode::initializeParameters() {
    // 声明基础视频源参数
    this->declare_parameter<int>("camera_id", 0);
    this->declare_parameter<std::string>("video_path", "");
    this->declare_parameter<int>("frame_width", 640);
    this->declare_parameter<int>("frame_height", 480);
    this->declare_parameter<int>("fps", 30);

    // 声明相机内参及畸变参数（提供合理默认值）
    this->declare_parameter<std::vector<double>>("camera_matrix",
        std::vector<double>{640.0, 0.0, 320.0,
                            0.0, 640.0, 240.0,
                            0.0, 0.0, 1.0});
    this->declare_parameter<std::string>("distortion_model", "plumb_bob");
    this->declare_parameter<std::vector<double>>("distortion_coefficients",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

    // 获取参数值
    try {
        camera_id_ = this->get_parameter("camera_id").as_int();
        video_path_ = this->get_parameter("video_path").as_string();
        frame_width_ = this->get_parameter("frame_width").as_int();
        frame_height_ = this->get_parameter("frame_height").as_int();
        fps_ = this->get_parameter("fps").as_int();

        // 参数有效性基础检查
        if (fps_ <= 0) {
            RCLCPP_ERROR(this->get_logger(), "参数 'fps' 必须为正整数，当前为: %d", fps_);
            return false;
        }
        if (frame_width_ <= 0 || frame_height_ <= 0) {
            RCLCPP_ERROR(this->get_logger(), "分辨率参数必须为正数，当前为: %dx%d", frame_width_, frame_height_);
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "获取节点参数时发生异常: %s", e.what());
        return false;
    }
    return true;
}

bool CameraDriverNode::loadAndSetupCameraInfo() {
    bool params_valid = true;

    try {
        camera_matrix_ = this->get_parameter("camera_matrix").as_double_array();
        dist_coeffs_ = this->get_parameter("distortion_coefficients").as_double_array();
        distortion_model_ = this->get_parameter("distortion_model").as_string();
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "读取相机内参参数时出错，将使用单位矩阵和零畸变系数。错误: %s", e.what());
        params_valid = false;
    }

    // 配置 camera_info_msg_
    camera_info_msg_.width = frame_width_;
    camera_info_msg_.height = frame_height_;
    camera_info_msg_.distortion_model = distortion_model_;

    // 填充内参矩阵 K (3x3)
    if (params_valid && camera_matrix_.size() == 9) {
        std::copy(camera_matrix_.begin(), camera_matrix_.end(), camera_info_msg_.k.begin());
    } else {
        RCLCPP_WARN(this->get_logger(), "camera_matrix 参数无效或缺失，使用单位矩阵。");
        // 设置一个简单的单位矩阵作为默认值 (fx=fy=焦距假设值, cx,cy=图像中心)
        camera_info_msg_.k[0] = 600.0; camera_info_msg_.k[1] = 0.0; camera_info_msg_.k[2] = frame_width_ / 2.0;
        camera_info_msg_.k[3] = 0.0; camera_info_msg_.k[4] = 600.0; camera_info_msg_.k[5] = frame_height_ / 2.0;
        camera_info_msg_.k[6] = 0.0; camera_info_msg_.k[7] = 0.0; camera_info_msg_.k[8] = 1.0;
    }

    // 填充畸变系数 D
    camera_info_msg_.d = dist_coeffs_; // 如果获取失败，这里会是空的vector

    // 填充投影矩阵 P (3x4). 对于单目相机，通常 P = [K | 0]
    // 前3列直接复制K矩阵
    camera_info_msg_.p[0] = camera_info_msg_.k[0]; camera_info_msg_.p[1] = camera_info_msg_.k[1]; camera_info_msg_.p[2] = camera_info_msg_.k[2]; camera_info_msg_.p[3] = 0.0;
    camera_info_msg_.p[4] = camera_info_msg_.k[3]; camera_info_msg_.p[5] = camera_info_msg_.k[4]; camera_info_msg_.p[6] = camera_info_msg_.k[5]; camera_info_msg_.p[7] = 0.0;
    camera_info_msg_.p[8] = camera_info_msg_.k[6]; camera_info_msg_.p[9] = camera_info_msg_.k[7]; camera_info_msg_.p[10] = camera_info_msg_.k[8]; camera_info_msg_.p[11] = 0.0;

    // 填充旋转矩阵 R (3x3, 单位矩阵，表示未发生旋转)
    std::fill(camera_info_msg_.r.begin(), camera_info_msg_.r.end(), 0.0);
    camera_info_msg_.r[0] = 1.0; camera_info_msg_.r[4] = 1.0; camera_info_msg_.r[8] = 1.0;

    return params_valid; // 返回内参是否被正确加载
}

bool CameraDriverNode::openVideoSource() {
    bool is_opened = false;
    std::string source_description;

    if (!video_path_.empty()) {
        // 模式1：从视频文件读取
        RCLCPP_INFO(this->get_logger(), "尝试打开视频文件: %s", video_path_.c_str());
        is_opened = cap_.open(video_path_);
        source_description = "视频文件: " + video_path_;
    } else {
        // 模式2：从物理相机读取
        RCLCPP_INFO(this->get_logger(), "尝试打开相机设备，ID: %d", camera_id_);
#ifdef WITH_V4L2
        is_opened = cap_.open(camera_id_, cv::CAP_V4L2);
#else
        is_opened = cap_.open(camera_id_);
#endif
        if (is_opened) {
            // 设置相机参数
            bool width_ok = cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
            bool height_ok = cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
            bool fps_ok = cap_.set(cv::CAP_PROP_FPS, fps_);
            // 尝试设置MJPG格式以获得更高帧率（如果支持）
            bool fourcc_ok = cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

            if (!width_ok || !height_ok) {
                RCLCPP_WARN(this->get_logger(), "相机可能不支持请求的分辨率 %dx%d，将使用默认分辨率。", frame_width_, frame_height_);
            }
            if (!fps_ok) {
                RCLCPP_WARN(this->get_logger(), "相机可能不支持请求的帧率 %d FPS，将使用默认帧率。", fps_);
            }
            RCLCPP_DEBUG(this->get_logger(), "相机参数设置: 分辨率设置%s, 帧率设置%s, 编码设置%s.",
                        width_ok && height_ok ? "成功" : "失败",
                        fps_ok ? "成功" : "失败",
                        fourcc_ok ? "成功" : "失败或忽略");
        }
        source_description = "相机设备 ID: " + std::to_string(camera_id_);
    }

    if (!is_opened || !cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开视频源: %s", source_description.c_str());
        return false;
    }

    // 获取实际打开的参数，用于日志输出
    double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);
    RCLCPP_INFO(this->get_logger(), "视频源已成功打开。实际参数: 分辨率 %.0fx%.0f @ %.2f FPS",
                actual_width, actual_height, actual_fps);
    return true;
}

void CameraDriverNode::captureLoop() {
    cv::Mat current_frame;
    const auto target_frame_interval = std::chrono::microseconds(1'000'000 / fps_); // 计算目标帧间隔（微秒）
    auto last_log_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    RCLCPP_DEBUG(this->get_logger(), "图像采集线程启动，目标帧间隔: %lld us。", target_frame_interval.count());

    while (running_ && rclcpp::ok()) {
        auto loop_start_time = std::chrono::steady_clock::now();

        // 步骤1：从视频源读取一帧
        if (!cap_.read(current_frame) || current_frame.empty()) {
            if (!video_path_.empty()) {
                // 对于视频文件，读到结尾则循环播放
                RCLCPP_DEBUG(this->get_logger(), "视频文件播放完毕，重置到开头。");
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            } else {
                // 对于物理相机，读取失败则等待后重试
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "从相机读取图像失败，可能是连接断开，等待后重试...");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }

        // 步骤2：准备ROS消息
        rclcpp::Time current_stamp = this->now();
        sensor_msgs::msg::Image::SharedPtr img_msg;
        try {
            img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", current_frame).toImageMsg();
            img_msg->header.stamp = current_stamp;
            img_msg->header.frame_id = "camera_optical_frame";
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "在转换图像为ROS消息时发生cv_bridge异常: %s", e.what());
            continue; // 跳过这一帧
        }

        // 更新 camera_info 的时间戳和帧ID
        camera_info_msg_.header.stamp = current_stamp;
        camera_info_msg_.header.frame_id = "camera_optical_frame";

        // 步骤3：发布图像和相机信息
        camera_pub_.publish(*img_msg, camera_info_msg_);
        frame_count++;

        // 步骤4：简单的帧率统计（每秒打印一次）
        auto now = std::chrono::steady_clock::now();
        auto elapsed_since_log = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();
        if (elapsed_since_log >= 2) { // 每2秒打印一次统计信息
            double actual_fps = frame_count / elapsed_since_log;
            RCLCPP_DEBUG(this->get_logger(), "图像发布统计: 过去 %.1f 秒内平均帧率: %.2f FPS",
                        elapsed_since_log, actual_fps);
            frame_count = 0;
            last_log_time = now;
        }

        // 步骤5：精确的帧率控制
        auto processing_time = std::chrono::steady_clock::now() - loop_start_time;
        if (processing_time < target_frame_interval) {
            // 处理速度比目标快，休眠剩余时间以稳定帧率
            std::this_thread::sleep_for(target_frame_interval - processing_time);
        } else {
            // 处理速度比目标慢，记录警告
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "图像处理循环超时！处理耗时: %.2f ms，目标帧间隔: %.2f ms。可能无法达到目标帧率 %d FPS。",
                                std::chrono::duration<double, std::milli>(processing_time).count(),
                                std::chrono::duration<double, std::milli>(target_frame_interval).count(),
                                fps_);
        }
    } // while (running_ && rclcpp::ok())

    RCLCPP_INFO(this->get_logger(), "图像采集线程正常退出。");
}

} // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::CameraDriverNode)
