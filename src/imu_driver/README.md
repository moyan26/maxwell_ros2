# IMU Driver ROS2 节点

这是 RoboCup 机器人 IMU 传感器的 ROS2 驱动节点，从原始代码迁移而来。

## 功能

1. **串口通信**：通过串口读取 IMU 数据
2. **数据发布**：
   - `/imu/data` - 标准 `sensor_msgs/Imu` 消息
   - `/imu/fall_direction` - 摔倒检测（Int8）
3. **摔倒检测**：根据俯仰角和横滚角判断机器人是否摔倒

## 原始代码 vs ROS2 对比

| 特性 | 原始代码 | ROS2 版本 |
|------|---------|-----------|
| 基类 | `Sensor` (Publisher) | `rclcpp::Node` |
| 配置 | `CONF` 单例 | ROS2 参数系统 |
| 日志 | 自定义 `LOG` 宏 | `RCLCPP_*` 宏 |
| 通信 | 观察者模式 `notify()` | ROS2 Topics |
| 时间戳 | 自定义 `CLOCK` | `rclcpp::Time` |

## 编译

```bash
# 在你的 ROS2 工作空间
cd ~/ros2_ws/src
# 复制此包到 src 目录
cd ..
colcon build --base-paths src/imu_driver
source install/setup.bash
```

## 运行

### 1. 使用默认参数(必须在连接机器人的条件下，目前还未测试)
```bash
ros2 run imu_driver imu_node
```
### 2.创建虚拟串口（仿真测试时使用）
```bash
ros2 run imu_driver imu_node --ros-args -p mock_mode:=true
```
### 其他方法：（以下方法中的命令行内容还需调整）
### 3. 使用 launch 文件
```bash
ros2 launch imu_driver imu.launch.py
```

### 4. 使用自定义参数
```bash
ros2 run imu_driver imu_node --ros-args \
  -p dev_name:="/dev/ttyACM0" \
  -p baudrate:=921600 \
  -p pitch_range:="[-45.0, 50.0]" \
  -p roll_range:="[-50.0, 50.0]"
```

### 5. 使用参数文件
```bash
ros2 run imu_driver imu_node --ros-args --params-file config/imu_params.yaml
```

## 查看数据

```bash
# 查看 IMU 数据
topic echo /imu/data

#这里可能因为我ROS2 Python 库兼容性问题，topic echo /imu/data会报错，所以单独创建了一个接收者节点用来显示信息
#再单独开一个终端，输入一下命令，即可运行接收者节点
cd ~/unirobot_ros2
source install/setup.bash
ros2 run imu_driver imu_listener

# 查看摔倒检测
topic echo /imu/fall_direction

# 图形化查看
rviz2  # 添加 Imu 显示插件
```

## 话题说明

### 发布的话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/imu/data` | `sensor_msgs/Imu` | IMU 原始数据（四元数、角速度、加速度）|
| `/imu/fall_direction` | `std_msgs/Int8` | 摔倒方向：0=正常, 1=前倒, -1=后倒, 2=左倒, -2=右倒 |

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `dev_name` | string | `/dev/ttyTHS2` | 串口设备路径 |
| `baudrate` | int | 115200 | 串口波特率 |
| `pitch_range` | float[] | `[-30.0, 40.0]` | 正常俯仰角范围（度）|
| `roll_range` | float[] | `[-40.0, 40.0]` | 正常横滚角范围（度）|
| `frame_id` | string | `imu_link` | TF 坐标系名称 |

## 权限设置

如果访问串口权限不足：

```bash
# 临时方案
sudo chmod 666 /dev/ttyTHS2

# 永久方案（推荐）
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

## 代码结构

```
imu_driver/
├── include/imu_driver/
│   └── imu_node.hpp      # 节点头文件
├── src/
│   ├── imu_node.cpp      # 节点实现（核心）
│   ├── imu_driver_main.cpp   # 主函数
│   └── imu_subscriber_example.cpp  # 订阅示例
├── launch/
│   └── imu.launch.py     # 启动文件
├── config/
│   └── imu_params.yaml   # 参数配置
├── CMakeLists.txt
└── package.xml
```

## 迁移说明

### 保留的原始代码
- 串口通信协议（`Packet_Decode` 状态机）
- CRC16 校验算法
- IMU 数据解析逻辑
- 摔倒检测算法

### 修改的部分
- 继承 `rclcpp::Node` 替代 `Sensor`
- 使用 `rclcpp::Publisher` 替代 `notify()`
- 使用 ROS2 参数替代 `CONF`
- 使用 `sensor_msgs/Imu` 标准消息格式
- 使用 `tf2` 进行欧拉角到四元数的转换
