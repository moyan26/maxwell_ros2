# Maxwell Robot — ROS2 工作空间

## 工程结构
```
ros2_ws/src/
├── maxwell_msgs/              # 自定义消息类型
├── maxwell_core/              # 公共库（待填充）
├── imu_driver/                # IMU传感器节点（A组）
├── camera_driver/             # 摄像头节点（A组）
├── game_controller_driver/    # 裁判盒节点（A组）
├── motor_driver/              # 电机驱动节点（A组）
├── maxwell_vision/            # 视觉节点（B组，开发中）
├── maxwell_world/             # 世界模型节点（B组，开发中）
├── maxwell_behavior/          # 决策节点（C组）
├── maxwell_motion/            # 步态节点（C组）
├── maxwell_remote/            # 远程控制节点（待填充）
└── maxwell_bringup/           # launch文件 + 参数配置
```

## 数据流
```
[camera_node]            --/camera/image_raw-----------> [vision_node(B)]
[imu_node]               --/imu/data------------------> [world_node(B)]
                         --/imu/fall_direction---------> [world_node(B)]
[motor_node]             --/joint_states--------------> [world_node(B)]
[game_controller_node]   --/game_controller/state-----> [world_node(B)]

[world_node]  --/world/state-----------> [fsm_node]
[fsm_node]    --/behavior/cmd----------> [walk_node]
[walk_node]   --/joint_command---------> [motor_node]
```

## 话题列表

### 传感器层（A组）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/imu/data` | `sensor_msgs/Imu` | IMU原始数据 |
| `/imu/fall_direction` | `std_msgs/Int8` | 摔倒方向(0=无/1=前/-1=后/2=左/-2=右) |
| `/camera/image_raw` | `sensor_msgs/Image` | 摄像头图像 |
| `/joint_states` | `sensor_msgs/JointState` | 关节实际角度（弧度） |
| `/game_controller/state` | `std_msgs/String` | 比赛状态字符串 |
| `/game_controller/state_code` | `std_msgs/Int8` | 比赛状态码(0-4) |
| `/game_controller/score` | `std_msgs/Int8` | 本队得分 |
| `/game_controller/time_remaining` | `std_msgs/Int16` | 剩余时间(秒) |
| `/game_controller/secondary_time` | `std_msgs/Int16` | 次要时间 |
| `/game_controller/first_half` | `std_msgs/Bool` | 是否上半场 |
| `/game_controller/kick_off_team` | `std_msgs/Int8` | 开球队伍编号 |
| `/game_controller/secondary_state` | `std_msgs/String` | 次要状态 |

### 感知层（B组）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/world/state` | `maxwell_msgs/WorldState` | 融合后的世界状态 |

### 决策层（C组）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/behavior/cmd` | `maxwell_msgs/BehaviorCmd` | 行为指令 |

### 执行层（C组）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/joint_command` | `maxwell_msgs/JointCommand` | 关节目标角度（弧度） |

## 自定义消息类型

### `maxwell_msgs/WorldState`
```
int32 fall_direction      # 0=无 1=前 -1=后 2=左 -2=右
bool localization_time    # 定位时间戳
float32 self_x            # 机器人自身X坐标
float32 self_y            # 机器人自身Y坐标
float32 self_dir_deg      # 机器人朝向（度）
bool ball_seen            # 是否看到球
float32 ball_self_x       # 球相对机器人X
float32 ball_self_y       # 球相对机器人Y
float32 ball_global_x     # 球全局X坐标
float32 ball_global_y     # 球全局Y坐标
float32 opp_post_left_x   # 对方左门柱X
float32 opp_post_left_y   # 对方左门柱Y
float32 opp_post_right_x  # 对方右门柱X
float32 opp_post_right_y  # 对方右门柱Y
```

### `maxwell_msgs/BehaviorCmd`
```
uint8 type          # 行为类型
float32 x           # 目标X
float32 y           # 目标Y
float32 dir         # 目标方向
bool enable         # 是否启用
string action_name  # 动作名称
```

### `maxwell_msgs/JointCommand`
```
float32[] positions    # 关节目标角度（弧度）
float32[] velocities   # 关节目标速度
```

## 编译方法
```bash
# 编译所有包
cd ~/ros2_ws
colcon build

# 编译指定包
colcon build --packages-select maxwell_msgs

# 编译后激活环境
source ~/ros2_ws/install/setup.bash
```

## 启动方法
```bash
# 不带裁判盒（调试用）
ros2 launch maxwell_bringup play_without_gc.launch.py

# 带裁判盒（比赛用）
ros2 launch maxwell_bringup play_with_gc.launch.py
```

## 单节点调试（仿真模式）
```bash
# 注意：mock_mode 参数类型为 bool，用 true/false
ros2 run imu_driver imu_node --ros-args -p mock_mode:=true
ros2 run camera_driver camera_node --ros-args -p mock_mode:=true
ros2 run game_controller_driver game_controller_node --ros-args -p mock_mode:=true
ros2 run motor_driver motor_node --ros-args -p mock_mode:=true
ros2 run maxwell_behavior fsm_node
ros2 run maxwell_motion walk_node
```

## 各组开发须知

### 使用自定义消息
在 `package.xml` 中添加：
```xml
<depend>maxwell_msgs</depend>
```

在 `CMakeLists.txt` 中添加：
```cmake
find_package(maxwell_msgs REQUIRED)
```

在 C++ 代码中引用：
```cpp
#include "maxwell_msgs/msg/world_state.hpp"
#include "maxwell_msgs/msg/behavior_cmd.hpp"
#include "maxwell_msgs/msg/joint_command.hpp"
```

## 当前进度

| 包 | 状态 | 负责人 |
|----|------|--------|
| maxwell_msgs | ✅ 完成 | D组 |
| maxwell_bringup | ✅ 完成 | D组 |
| imu_driver | ✅ 验证通过 | A组 |
| camera_driver | ✅ 验证通过 | A组 |
| game_controller_driver | ✅ 验证通过 | A组 |
| motor_driver | ✅ 验证通过 | A组 |
| maxwell_behavior | ✅ 验证通过 | C组 |
| maxwell_motion | ✅ 验证通过 | C组 |
| maxwell_world | ⏳ 开发中 | B组 |
| maxwell_vision | ⏳ 开发中 | B组 |
| maxwell_core | ⏳ 待填充 | D组 |
| maxwell_remote | ⏳ 待填充 | D组 |
