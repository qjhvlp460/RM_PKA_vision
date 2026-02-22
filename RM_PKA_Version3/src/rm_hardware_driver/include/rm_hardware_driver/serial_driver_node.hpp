#pragma once

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <atomic>
#include <string>
#include <mutex>

#include "rm_hardware_driver/fixed_packet.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"

namespace rm_auto_aim {

/**
 * @brief 串口通信驱动节点
 *
 * 功能:
 * 1. 接收 GimbalCmd 消息，打包发送给下位机
 * 2. 从下位机接收数据，发布 SerialReceiveData 消息
 *
 * 协议: 固定长度包, Infantry协议
 */
class SerialDriverNode : public rclcpp::Node {
public:
    explicit SerialDriverNode(const rclcpp::NodeOptions& options);
    ~SerialDriverNode() override;

private:
    /**
     * @brief 打开串口
     */
    bool openPort();

    /**
     * @brief 关闭串口
     */
    void closePort();

    /**
     * @brief 发送线程
     */
    void sendThread();

    /**
     * @brief 接收线程
     */
    void receiveThread();

    /**
     * @brief 云台命令回调
     */
    void gimbalCmdCallback(const rm_interfaces::msg::GimbalCmd::ConstSharedPtr& msg);

    /**
     * @brief 打包云台控制数据
     */
    Packet16 packGimbalCmd(double yaw, double pitch, bool fire);

    /**
     * @brief 解包接收数据
     */
    void unpackReceiveData(const Packet16& packet);

    // 串口参数
    std::string port_name_;
    int baud_rate_;
    int fd_ = -1;  // 串口文件描述符

    // 线程控制
    std::atomic<bool> running_{false};
    std::thread receive_thread_;

    // 最新的发送数据（线程安全）
    std::mutex send_mutex_;
    Packet16 send_packet_;
    bool has_new_data_ = false;

    // ROS接口
    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr receive_pub_;

    // 定时发送
    rclcpp::TimerBase::SharedPtr send_timer_;
};

}  // namespace rm_auto_aim
