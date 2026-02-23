#include "rm_hardware_driver/serial_driver_node.hpp"

#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>
#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#endif

namespace rm_auto_aim {

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions& options)
    : Node("serial_driver", options),
      running_(false),
      has_new_data_(false),
      fd_(-1)
{
    RCLCPP_INFO(this->get_logger(), "正在初始化 SerialDriverNode...");

    // 步骤1：配置节点参数
    if (!configureParameters()) {
        RCLCPP_ERROR(this->get_logger(), "串口参数配置失败，节点无法启动。");
        return;
    }

    // 步骤2：初始化通信接口
    initializeCommunications();

    // 步骤3：尝试建立串口连接
    if (!establishSerialConnection()) {
        RCLCPP_FATAL(this->get_logger(), "串口连接建立失败，节点即将退出。");
        return;
    }

    // 步骤4：启动数据处理线程
    startProcessingThreads();
    RCLCPP_INFO(this->get_logger(), "SerialDriverNode 初始化完成，串口 %s 已就绪。", port_name_.c_str());
}

SerialDriverNode::~SerialDriverNode() {
    RCLCPP_DEBUG(this->get_logger(), "开始析构 SerialDriverNode...");
    
    // 步骤1：停止所有运行中的线程
    running_ = false;
    
    // 步骤2：等待接收线程安全退出
    if (receive_thread_.joinable()) {
        receive_thread_.join();
        RCLCPP_DEBUG(this->get_logger(), "串口接收线程已成功停止。");
    }
    
    // 步骤3：关闭串口连接
    closeSerialPort();
    RCLCPP_DEBUG(this->get_logger(), "串口连接已安全关闭。");
}

bool SerialDriverNode::configureParameters() {
    try {
        // 声明并获取串口通信参数
        this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<bool>("enable_data_print", false);
        
        port_name_ = this->get_parameter("port_name").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        enable_data_print_ = this->get_parameter("enable_data_print").as_bool();
        
        // 验证参数有效性
        if (port_name_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "串口设备名称不能为空。");
            return false;
        }
        
        // 检查波特率是否在支持范围内
        const std::vector<int> supported_baud_rates = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
        if (std::find(supported_baud_rates.begin(), supported_baud_rates.end(), baud_rate_) == supported_baud_rates.end()) {
            RCLCPP_WARN(this->get_logger(), "波特率 %d 可能不被硬件完全支持，建议使用标准值。", baud_rate_);
        }
        
        RCLCPP_INFO(this->get_logger(), "串口配置: 设备=%s, 波特率=%d, 数据打印=%s", 
                    port_name_.c_str(), baud_rate_, enable_data_print_ ? "启用" : "禁用");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "参数配置过程中发生异常: %s", e.what());
        return false;
    }
}

void SerialDriverNode::initializeCommunications() {
    // 创建订阅器：接收云台控制指令
    gimbal_cmd_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "/solver/gimbal_cmd", 
        rclcpp::SensorDataQoS(),
        std::bind(&SerialDriverNode::handleGimbalCommand, this, std::placeholders::_1));
    RCLCPP_DEBUG(this->get_logger(), "已订阅云台控制话题: /solver/gimbal_cmd");
    
    // 创建发布器：发布从串口接收的数据
    receive_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
        "/serial/receive", 
        rclcpp::SensorDataQoS());
    RCLCPP_DEBUG(this->get_logger(), "已创建串口数据发布话题: /serial/receive");
}

bool SerialDriverNode::establishSerialConnection() {
#ifdef __linux__
    RCLCPP_INFO(this->get_logger(), "正在打开串口设备: %s", port_name_.c_str());
    
    // 步骤1：以读写和非阻塞模式打开串口设备
    fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口设备 %s: %s", 
                     port_name_.c_str(), strerror(errno));
        return false;
    }
    
    // 步骤2：获取当前串口配置
    struct termios serial_config;
    std::memset(&serial_config, 0, sizeof(serial_config));
    
    if (tcgetattr(fd_, &serial_config) != 0) {
        RCLCPP_ERROR(this->get_logger(), "获取串口配置失败: %s", strerror(errno));
        close(fd_);
        fd_ = -1;
        return false;
    }
    
    // 步骤3：配置串口参数
    if (!configureSerialPort(serial_config)) {
        close(fd_);
        fd_ = -1;
        return false;
    }
    
    // 步骤4：应用配置
    if (tcsetattr(fd_, TCSANOW, &serial_config) != 0) {
        RCLCPP_ERROR(this->get_logger(), "应用串口配置失败: %s", strerror(errno));
        close(fd_);
        fd_ = -1;
        return false;
    }
    
    // 步骤5：清空输入输出缓冲区
    tcflush(fd_, TCIOFLUSH);
    
    RCLCPP_INFO(this->get_logger(), "串口 %s 配置成功，波特率: %d", 
                port_name_.c_str(), baud_rate_);
    return true;
    
#else
    // 非Linux平台（如Windows或macOS）的备用实现
    RCLCPP_WARN(this->get_logger(), "串口通信功能仅完整支持Linux平台，当前以模拟模式运行。");
    RCLCPP_INFO(this->get_logger(), "模拟模式: 将接收云台指令但不实际发送串口数据。");
    return true; // 返回true使节点继续运行，但实际无串口通信
#endif
}

#ifdef __linux__
bool SerialDriverNode::configureSerialPort(struct termios& config) {
    // 将波特率转换为系统定义
    speed_t baud_speed = B115200; // 默认值
    switch (baud_rate_) {
        case 9600:    baud_speed = B9600; break;
        case 19200:   baud_speed = B19200; break;
        case 38400:   baud_speed = B38400; break;
        case 57600:   baud_speed = B57600; break;
        case 115200:  baud_speed = B115200; break;
        case 230400:  baud_speed = B230400; break;
        case 460800:  baud_speed = B460800; break;
        case 921600:  baud_speed = B921600; break;
        default:
            RCLCPP_WARN(this->get_logger(), "不支持的波特率 %d，将使用默认值115200", baud_rate_);
            baud_speed = B115200;
    }
    
    // 设置输入输出波特率
    if (cfsetispeed(&config, baud_speed) != 0 || cfsetospeed(&config, baud_speed) != 0) {
        RCLCPP_ERROR(this->get_logger(), "设置波特率失败");
        return false;
    }
    
    // 配置数据位、停止位、校验位
    config.c_cflag &= ~PARENB;   // 禁用奇偶校验
    config.c_cflag &= ~CSTOPB;   // 使用1个停止位
    config.c_cflag &= ~CSIZE;    // 清除数据位设置
    config.c_cflag |= CS8;       // 设置8个数据位
    config.c_cflag &= ~CRTSCTS;  // 禁用硬件流控
    config.c_cflag |= CREAD | CLOCAL; // 启用接收器，忽略调制解调器控制线
    
    // 配置输入模式
    config.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控
    config.c_iflag &= ~(INLCR | ICRNL | IGNCR); // 禁用特殊字符处理
    
    // 配置输出模式
    config.c_oflag &= ~OPOST; // 使用原始输出
    
    // 配置本地模式
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 非规范模式，禁用回显和信号
    
    // 配置超时控制: 100ms超时，最小读取0字节
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 1; // 0.1秒超时
    
    return true;
}
#endif

void SerialDriverNode::startProcessingThreads() {
    running_ = true;
    
    // 启动接收线程
    receive_thread_ = std::thread(&SerialDriverNode::serialReceiveTask, this);
    RCLCPP_DEBUG(this->get_logger(), "串口接收线程已启动。");
    
    // 创建定时发送器（1kHz）
    send_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1), // 1ms周期对应1kHz
        std::bind(&SerialDriverNode::serialSendTask, this));
    RCLCPP_DEBUG(this->get_logger(), "串口发送定时器已创建，频率: 1kHz");
}

void SerialDriverNode::closeSerialPort() {
#ifdef __linux__
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
        RCLCPP_DEBUG(this->get_logger(), "串口文件描述符已关闭。");
    }
#endif
}

void SerialDriverNode::handleGimbalCommand(const rm_interfaces::msg::GimbalCmd::ConstSharedPtr& command_msg) {
    // 使用互斥锁保护共享数据
    std::lock_guard<std::mutex> lock(send_mutex_);
    
    // 打包云台控制指令
    send_packet_ = encodeGimbalCommand(command_msg->yaw, command_msg->pitch, command_msg->fire);
    has_new_data_ = true;
    
    if (enable_data_print_) {
        RCLCPP_DEBUG(this->get_logger(), 
                    "收到云台指令: Yaw=%.2f°, Pitch=%.2f°, Fire=%s",
                    command_msg->yaw * 180.0 / M_PI,
                    command_msg->pitch * 180.0 / M_PI,
                    command_msg->fire ? "True" : "False");
    }
}

Packet16 SerialDriverNode::encodeGimbalCommand(double yaw_rad, double pitch_rad, bool fire_command) {
    Packet16 command_packet;
    
    // 将弧度转换为度并放大100倍，以适应16位整数传输
    auto yaw_encoded = static_cast<int16_t>(yaw_rad * 180.0 / M_PI * 100.0);
    auto pitch_encoded = static_cast<int16_t>(pitch_rad * 180.0 / M_PI * 100.0);
    uint8_t fire_encoded = fire_command ? 1 : 0;
    
    // 按照协议格式打包数据
    // 协议格式: [帧头][Yaw高字节][Yaw低字节][Pitch高字节][Pitch低字节][开火标志]...[校验和][帧尾]
    command_packet.load<int16_t>(1, yaw_encoded);     // 位置1-2: Yaw角度
    command_packet.load<int16_t>(3, pitch_encoded);   // 位置3-4: Pitch角度
    command_packet.load<uint8_t>(5, fire_encoded);    // 位置5: 开火指令
    command_packet.setChecksum();                     // 计算并设置校验和
    
    return command_packet;
}

void SerialDriverNode::serialSendTask() {
#ifdef __linux__
    if (fd_ < 0) {
        // 串口未打开，无需发送
        return;
    }
    
    // 检查是否有新数据需要发送
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (!has_new_data_) {
        return;
    }
    
    // 发送数据包
    ssize_t bytes_sent = write(fd_, send_packet_.data.data(), Packet16::SIZE);
    if (bytes_sent < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "串口数据发送失败: %s", strerror(errno));
    } else if (bytes_sent != Packet16::SIZE) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "串口数据发送不完整: 期望%d字节，实际发送%zd字节", 
                            Packet16::SIZE, bytes_sent);
    } else if (enable_data_print_) {
        RCLCPP_DEBUG(this->get_logger(), "成功发送 %zd 字节数据到串口", bytes_sent);
    }
    
    // 清除发送标志
    has_new_data_ = false;
#endif
}

void SerialDriverNode::serialReceiveTask() {
#ifdef __linux__
    constexpr size_t BUFFER_SIZE = 256;
    std::array<uint8_t, BUFFER_SIZE> read_buffer;
    std::array<uint8_t, Packet16::SIZE> packet_buffer;
    size_t packet_position = 0;
    
    RCLCPP_INFO(this->get_logger(), "串口接收线程开始运行。");
    
    while (running_ && rclcpp::ok()) {
        if (fd_ < 0) {
            // 串口未打开，等待后重试
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // 尝试从串口读取数据
        ssize_t bytes_read = read(fd_, read_buffer.data(), read_buffer.size());
        
        if (bytes_read < 0) {
            // 读取错误（可能暂时无数据）
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "串口读取错误: %s", strerror(errno));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        } else if (bytes_read == 0) {
            // 无数据可读
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        
        // 处理读取到的每个字节
        for (ssize_t i = 0; i < bytes_read; ++i) {
            uint8_t current_byte = read_buffer[i];
            
            // 查找数据包起始位置（帧头）
            if (packet_position == 0 && current_byte != Packet16::HEADER) {
                continue; // 跳过非帧头字节
            }
            
            // 将字节存入数据包缓冲区
            packet_buffer[packet_position++] = current_byte;
            
            // 检查是否接收到完整数据包
            if (packet_position >= Packet16::SIZE) {
                // 重构数据包
                Packet16 received_packet;
                std::memcpy(received_packet.data.data(), packet_buffer.data(), Packet16::SIZE);
                
                // 验证数据包有效性
                if (received_packet.isValid()) {
                    processReceivedPacket(received_packet);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                        "接收到无效数据包，校验失败");
                }
                
                // 重置缓冲区位置
                packet_position = 0;
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "串口接收线程正常退出。");
#else
    // 非Linux平台的模拟接收逻辑
    RCLCPP_INFO(this->get_logger(), "串口接收线程在模拟模式下运行。");
    while (running_ && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // 在模拟模式下，可以生成虚拟数据用于测试
        if (enable_data_print_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "模拟模式: 虚拟串口数据接收");
        }
    }
#endif
}

void SerialDriverNode::processReceivedPacket(const Packet16& packet) {
    rm_interfaces::msg::SerialReceiveData received_data;
    received_data.header.stamp = this->now();
    
    // 解析弹速数据（4字节浮点数）
    received_data.bullet_speed = static_cast<double>(packet.read<float>(1));
    
    // 解析当前云台姿态（16位整数，放大100倍）
    auto encoded_yaw = packet.read<int16_t>(5);
    auto encoded_pitch = packet.read<int16_t>(7);
    
    // 转换为弧度值
    const double scaling_factor = 100.0 * 180.0 / M_PI; // 编码时放大的倍数
    received_data.cur_yaw = static_cast<double>(encoded_yaw) / scaling_factor;
    received_data.cur_pitch = static_cast<double>(encoded_pitch) / scaling_factor;
    
    // 解析颜色和模式信息
    received_data.color = packet.read<uint8_t>(9);
    received_data.mode = packet.read<uint8_t>(10);
    
    // 发布解析后的数据
    receive_pub_->publish(received_data);
    
    // 如果启用数据打印，记录接收信息
    if (enable_data_print_) {
        RCLCPP_DEBUG(this->get_logger(), 
                    "收到串口数据: 弹速=%.2f m/s, Yaw=%.2f°, Pitch=%.2f°, Color=%u, Mode=%u",
                    received_data.bullet_speed,
                    received_data.cur_yaw * 180.0 / M_PI,
                    received_data.cur_pitch * 180.0 / M_PI,
                    received_data.color,
                    received_data.mode);
    }
}

} // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::SerialDriverNode)
