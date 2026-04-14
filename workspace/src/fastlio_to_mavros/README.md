# fastlio_to_mavros

这是一个ROS功能包，用于将FAST-LIO的里程计输出转换为MAVROS的vision_pose格式，实现PX4飞控的视觉惯性融合定位。

## 功能说明

该节点订阅FAST-LIO输出的里程计信息（`nav_msgs/Odometry`），并将其转换为PX4可以理解的ENU（东-北-天）坐标系下的位姿信息，通过MAVROS的`vision_pose`话题发送给PX4飞控。

### 主要特性

1. **坐标系对齐**: 将LIO坐标系转换到ENU坐标系，使用PX4本地里程计的yaw角进行初始化对齐
2. **滑动窗口平滤**: 使用滑动窗口平均PX4的yaw角，减少初始化时的抖动
3. **角度连续化处理**: 通过unwrap算法处理yaw角的跳变，确保平均值计算的准确性
4. **可选外参标定**: 支持配置雷达到机体坐标系的外参（旋转和平移）
5. **智能初始化**: 等待滑动窗口填满并延迟1秒后才进行初始化，确保传感器数据稳定

## 依赖项

- ROS Noetic (或其他ROS 1版本)
- Eigen3
- MAVROS
- FAST-LIO

## 编译

```bash
cd ~/uav_workspace/base_ws/lidar_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="fastlio_to_mavros"
# 或者编译整个工作空间
catkin_make
source devel/setup.bash
```

## 使用方法

### 1. 基本启动

确保以下节点已经运行：
- MAVROS（连接到PX4飞控）
- FAST-LIO

然后启动本节点：

```bash
roslaunch fastlio_to_mavros fastlio_to_mavros.launch
```

### 2. 参数配置

可以在launch文件中或通过ROS参数服务器配置以下参数：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `window_size` | int | 8 | 滑动窗口大小，用于平滑yaw角 |
| `fastlio_topic` | string | `/Odometry` | FAST-LIO输出的里程计话题 |
| `px4_odom_topic` | string | `/mavros/local_position/odom` | PX4本地位置里程计话题 |
| `vision_topic` | string | `/mavros/vision_pose/pose` | 发布到MAVROS的视觉位姿话题 |
| `vision_frame_id` | string | `map` | 视觉位姿的坐标系ID（可选`map`或`odom`） |
| `use_lidar_extrinsic` | bool | false | 是否使用雷达外参 |
| `q_base_from_lidar` | array[4] | [1,0,0,0] | 机体到雷达的旋转四元数(w,x,y,z) |
| `t_base_from_lidar` | array[3] | [0,0,0] | 机体到雷达的平移向量(x,y,z) |

### 3. 外参标定（可选）

如果雷达不是安装在机体中心或存在旋转偏差，需要配置外参。在launch文件中取消注释相关行：

```xml
<param name="use_lidar_extrinsic" value="true"/>
<rosparam param="q_base_from_lidar">[1.0, 0.0, 0.0, 0.0]</rosparam>
<rosparam param="t_base_from_lidar">[0.0, 0.0, 0.1]</rosparam>
```

## 话题说明

### 订阅话题

- `~/fastlio_topic` (nav_msgs/Odometry): FAST-LIO输出的里程计
- `~/px4_odom_topic` (nav_msgs/Odometry): PX4的本地位置里程计

### 发布话题

- `~/vision_topic` (geometry_msgs/PoseStamped): 发送给MAVROS的视觉位姿

## 工作原理

1. **初始化阶段**:
   - 订阅PX4的里程计话题，提取yaw角
   - 使用unwrap算法处理yaw角的连续性
   - 通过滑动窗口平滑yaw角
   - 等待窗口填满且延迟1秒后，计算平均yaw角作为初始对齐角度

2. **运行阶段**:
   - 接收FAST-LIO的位姿数据
   - （可选）应用雷达外参转换到机体坐标系
   - 使用初始化时计算的yaw角将位姿转换到ENU坐标系
   - 发布转换后的位姿到MAVROS

## PX4配置

在PX4端需要启用视觉位姿融合。通过QGroundControl或参数文件设置：

```
EKF2_AID_MASK = 24  # 启用视觉位置和视觉yaw融合
EKF2_HGT_MODE = 3   # 使用视觉高度
```

## 注意事项

1. 确保MAVROS和FAST-LIO正常运行后再启动本节点
2. 初始化需要约1秒时间，期间不会发布位姿数据
3. 建议在飞行器静止时完成初始化
4. 如果环境中存在磁干扰，建议增大`window_size`以获得更稳定的初始yaw估计

## 故障排查

- **节点启动后无输出**: 检查FAST-LIO和MAVROS是否正常发布数据
- **位姿跳变**: 检查雷达外参配置是否正确
- **高度漂移**: 检查PX4的EKF2参数配置

## 许可证

MIT License

## 作者

UAV Development Team

