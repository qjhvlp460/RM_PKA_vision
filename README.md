Auto Aim System for RoboMaster
基于 ROS2 的 RoboMaster 自动瞄准系统，集成装甲板识别、多目标跟踪、弹道实时补偿、反陀螺策略与高速串口通信等核心功能。采用模块化设计，便于二次开发与调试，为机器人竞赛和科研应用提供完整的视觉瞄准解决方案。

主要特性
鲁棒的目标识别：采用自适应阈值二值化与多维度几何约束，实现复杂环境下的装甲板稳定检测

高精度状态估计：基于10维扩展卡尔曼滤波，实现旋转目标的实时跟踪与预测

智能反陀螺策略：根据目标角速度动态切换瞄准模式，有效应对高速旋转目标

物理级弹道补偿：结合空气阻力模型与抛物线迭代求解，实现高精度弹道下坠补偿

实时控制闭环：基于定长协议包的1kHz串口通信，确保云台控制的低延迟与稳定性

系统架构
项目结构
复制
auto-aim/src/
├── rm_interfaces/          # 自定义消息与服务接口
│   ├── msg/                # Armor, Armors, Target, GimbalCmd 等消息定义
│   └── srv/                # 系统模式设置服务
├── rm_auto_aim/            # 核心算法模块
│   ├── detector/           # 视觉识别模块
│   │   ├── types.hpp       # 数据结构与参数配置
│   │   ├── detector.*      # 灯条检测与装甲板匹配算法
│   │   ├── pnp_solver.*    # 基于PnP的三维坐标解算
│   │   └── armor_detector_node.*  # ROS2识别节点实现
│   └── solver/             # 解算与控制模块
│       ├── extended_kalman_filter.*  # 扩展卡尔曼滤波实现
│       ├── armor_tracker.*          # 跟踪状态机与目标管理
│       ├── trajectory_compensator.hpp  # 弹道补偿算法
│       └── armor_solver_node.*      # ROS2解算节点实现
├── rm_hardware_driver/     # 硬件通信层
│   ├── fixed_packet.hpp    # 定长串口协议封装
│   ├── serial_driver_node.*  # 串口通信节点
│   └── camera_driver_node.*  # 相机驱动节点
├── rm_bringup/             # 系统启动与配置
│   ├── launch/bringup.launch.py  # 主启动文件
│   └── config/node_params/       # 节点参数配置文件
└── rm_robot_description/   # 机器人模型描述文件
核心数据流
系统采用多节点松耦合设计，数据流转路径清晰：




















核心算法模块
视觉识别流程
图像预处理：BGR转灰度图 → 自适应阈值二值化 → 形态学操作降噪

灯条检测：轮廓查找 → 旋转矩形拟合 → 长宽比/角度/面积多级筛选

装甲板匹配：灯条对间距比约束 → 连线角度验证 → 长度一致性检查 → 包含关系排除

三维解算：基于solvePnP(IPPE_SQUARE)求解双解 → 重投影误差择优 → 坐标转换输出

扩展卡尔曼滤波
状态向量（10维）：[xc, vx, yc, vy, zc, vz, yaw, vyaw, r, z_offset]

预测模型：匀速运动 + 匀速旋转

雅可比计算：中心差分法数值近似

观测模型：装甲板三维位置与偏航角

跟踪状态机
复制
LOST (初始状态)
  ↓ (检测到目标)
DETECTING (短暂检测)
  ↓ (连续帧检测)
TRACKING (稳定跟踪)
  ↕ (短暂丢失/恢复)
TEMP_LOST (临时丢失)
弹道补偿
物理模型：抛物线运动 + 空气阻力修正

迭代求解：二分法计算发射仰角补偿量

实时更新：基于目标距离动态调整

反陀螺策略
状态判断：根据目标角速度区分正常跟踪/陀螺模式

装甲板选择：陀螺状态下选择正对相机的装甲板作为瞄准点

预测补偿：结合旋转半径与角速度进行提前量计算

通信协议
数据包格式：[0xFF][17字节数据][XOR校验][0x0D]

发送频率：1kHz稳定发送

数据内容：云台偏航/俯仰角、射击指令、模式标识

快速开始
环境要求
Ubuntu 22.04 / 20.04

ROS2 Humble / Foxy

OpenCV 4.5+

Eigen3

安装与编译
bash
复制
# 创建工作空间并克隆代码
mkdir -p ~/auto_aim_ws/src
cd ~/auto_aim_ws/src
git clone <repository-url>

# 安装依赖
sudo apt update
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport

# 编译项目（支持配置文件热更新）
cd ~/auto_aim_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载环境变量
source install/setup.bash
系统启动
bash
复制
# 启动完整系统（所有节点）
ros2 launch rm_bringup bringup.launch.py

# 或单独启动模块
ros2 launch rm_auto_aim detector.launch.py  # 仅启动识别模块
ros2 launch rm_auto_aim solver.launch.py    # 仅启动解算模块
参数配置
相机标定（关键步骤）
修改 rm_bringup/config/node_params/camera_driver_params.yaml：

yaml
复制
camera_matrix: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
distortion_coefficients: [k1, k2, p1, p2, k3]
注意：需使用实际相机标定结果替换上述参数，标定精度直接影响PnP解算效果。

EKF参数调优
在 armor_solver_params.yaml中调整过程噪声与观测噪声协方差：

yaml
复制
ekf:
  sigma2_q_x: 0.01      # X轴过程噪声
  sigma2_q_y: 0.01      # Y轴过程噪声
  sigma2_q_yaw: 0.001   # 偏航角过程噪声
  r_x: 0.1              # X轴观测噪声
  r_yaw: 0.05           # 偏航角观测噪声
识别参数调整
根据实际环境调整 detector_params.yaml：

yaml
复制
light:
  min_ratio: 0.1        # 灯条最小长宽比
  max_ratio: 0.4        # 灯条最大长宽比
  max_angle: 45.0       # 灯条最大倾斜角度

armor:
  min_light_ratio: 0.7  # 配对灯条最小长度比
  max_light_ratio: 1.5  # 配对灯条最大长度比
调试工具
RViz可视化
bash
复制
# 启动RViz查看检测结果
ros2 run rviz2 rviz2 -d src/rm_auto_aim/config/visualization.rviz
话题监控
bash
复制
# 查看系统话题列表
ros2 topic list

# 实时查看装甲板检测结果
ros2 topic echo /detector/armors

# 查看云台控制指令
ros2 topic echo /solver/gimbal_cmd
性能评估
系统内置性能统计模块，通过设置 debug: true可输出：

单帧处理时间

检测成功率

跟踪稳定性指标

高级功能
多目标跟踪
支持同时跟踪多个装甲板目标，基于匈牙利算法进行数据关联，适用于多机器人对战场景。

自适应参数
部分关键参数支持运行时动态调整：

bash
复制
ros2 param set /armor_solver_node max_match_distance 2.0
离线回放模式
支持录制数据包与图像序列，便于算法调试与性能分析：

bash
复制
ros2 bag record -o session1 /image_raw /detector/armors
故障排除
常见问题
检测不稳定

检查相机曝光与增益设置

调整二值化阈值参数

验证灯条筛选条件

PnP解算误差大

确认相机内参标定精度

检查装甲板3D模型尺寸

验证图像坐标提取准确性

跟踪丢失频繁

调整EKF噪声协方差

优化状态机切换阈值

检查目标遮挡情况

性能优化建议
CPU优化：启用NEON指令集编译，使用Eigen矩阵运算

内存优化：避免动态内存分配，预分配缓冲区

I/O优化：使用零拷贝消息传递，减少数据复制
