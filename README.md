# Maxwell Robot — ROS2 工作空间

## 工程结构
```
ros2_ws/src/
├── maxwell_msgs/      # 自定义消息类型
├── maxwell_core/      # 公共库（配置/日志/robot模型）
├── maxwell_sensors/   # 传感器节点（A组）
├── maxwell_vision/    # 视觉节点（B组）
├── maxwell_world/     # 世界模型节点（B组）
├── maxwell_behavior/  # 决策节点（C组）
├── maxwell_motion/    # 步态/动作节点（C组）
├── maxwell_remote/    # 远程控制节点
└── maxwell_bringup/   # launch文件 + 参数配置
```

## 数据流
```
[camera_node] --/camera/image_raw-----------> [vision_node]
[imu_node]    --/imu/data------------------->  |
              --/imu/fall_direction---------->  |
[motor_node]  --/joint_states--------------> [world_node]
[game_ctrl]   --/game_controller/state------>  |
[vision_node] --/vision/ball--------------->   |
                                               |
[world_node]  --/world/state-------------> [fsm_node]
[fsm_node]    --/behavior/cmd-----------> [walk_node]
[walk_node]   --/joint_command----------> [motor_node]
```

## 话题列表

### 传感器层（A组发布）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/imu/data` | `sensor_msgs/Imu` | IMU原始数据 |
| `/imu/fall_direction` | `std_msgs/Int8` | 摔倒方向(0=无/1=前/2=后) |
| `/camera/image_raw` | `sensor_msgs/Image` | 摄像头图像 |
| `/joint_states` | `sensor_msgs/JointState` | 关节实际角度 |
| `/game_controller/state` | `std_msgs/String` | 比赛状态 |
| `/game_controller/state_code` | `std_msgs/Int8` | 比赛状态码 |
| `/game_controller/score` | `std_msgs/Int8` | 本队得分 |
| `/game_controller/time_remaining` | `std_msgs/Int16` | 剩余时间(秒) |
| `/game_controller/secondary_time` | `std_msgs/Int16` | 次要时间 |
| `/game_controller/first_half` | `std_msgs/Bool` | 是否上半场 |
| `/game_controller/kick_off_team` | `std_msgs/Int8` | 开球队伍 |
| `/game_controller/secondary_state` | `std_msgs/String` | 次要状态 |

### 感知层（B组发布）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/vision/ball` | `maxwell_msgs/WorldState` | 视觉识别结果 |

### 状态层（B组发布）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/world/state` | `maxwell_msgs/WorldState` | 融合后的世界状态 |

### 决策层（C组发布）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/behavior/cmd` | `maxwell_msgs/BehaviorCmd` | 行为指令 |

### 执行层（C组发布）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/joint_command` | `sensor_msgs/JointState` | 关节目标角度 |

## 自定义消息类型

### `maxwell_msgs/WorldState`
```
int8 fall_direction    # 0=没摔倒 1=向前摔 2=向后摔
bool ball_seen         # 是否看到球
float32 ball_x         # 球的X坐标（米）
float32 ball_y         # 球的Y坐标（米）
float32 ball_distance  # 球的距离（米）
```

### `maxwell_msgs/BehaviorCmd`
```
string action   # "search_ball" / "goto_ball" / "get_up"
```

## 编译方法
```bash
# 第一次编译，或新增包后
cd ~/ros2_ws
colcon build

# 只编译某个包
colcon build --packages-select maxwell_msgs

# 编译后必须执行，让终端识别新包
source ~/ros2_ws/install/setup.bash
```

## 启动方法
```bash
# 不带裁判盒（调试用）
ros2 launch maxwell_bringup play_without_gc.launch.py

# 带裁判盒（比赛用）
ros2 launch maxwell_bringup play_with_gc.launch.py
```

## 各组开发须知

### 如何使用自定义消息
在你的包的 `package.xml` 中添加：
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
```

### 仿真模式启动单个节点
```bash
ros2 run <包名> <节点名> --ros-args -p mock_model:=0
```
