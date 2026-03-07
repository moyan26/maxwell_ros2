# Maxwell Robot — ROS2 工作空间

## 工程结构

\`\`\`
ros2_ws/src/
├── maxwell_msgs/              # 自定义消息类型
├── maxwell_core/              # 公共库（待填充）
├── imu_driver/                # IMU传感器节点（A组）
├── camera_driver/             # 摄像头节点（A组）
├── game_controller_driver/    # 裁判盒节点（A组）
├── motor_driver/              # 电机驱动节点（A组）
├── maxwell_vision/            # 视觉节点（B组，需CUDA，仅机器人可编译）
├── maxwell_world/             # 世界模型节点（B组）
├── maxwell_behavior/          # 决策节点（C组）
├── maxwell_motion/            # 步态节点（C组）
├── maxwell_remote/            # 远程控制节点（待填充）
└── maxwell_bringup/           # launch文件 + 参数配置
\`\`\`

## 数据流

\`\`\`
[camera_node]            --/camera/image_raw-----------> [vision_node(B)]
[imu_node]               --/imu/data------------------> [world_node(B)]
                         --/imu/fall_direction---------> [world_node(B)]
[motor_node]             --/joint_states--------------> [world_node(B)]
[game_controller_node]   --/game_controller/state-----> [world_node(B)]
[vision_node]            --/vision/ball---------------> [world_node(B)]

[world_node]  --/world/state-----------> [fsm_node]
[fsm_node]    --/behavior/cmd----------> [walk_node]
[walk_node]   --/joint_command---------> [motor_node]
              --/walk/self_pose-------->  (定位输出)
\`\`\`

## 话题列表

### 传感器层（A组）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| \`/imu/data\` | \`sensor_msgs/Imu\` | IMU原始数据 |
| \`/imu/fall_direction\` | \`std_msgs/Int8\` | 摔倒方向(0=无/1=前/-1=后/2=左/-2=右) |
| \`/camera/image_raw\` | \`sensor_msgs/Image\` | 摄像头图像 |
| \`/joint_states\` | \`sensor_msgs/JointState\` | 关节实际角度（弧度） |
| \`/game_controller/state\` | \`std_msgs/String\` | 比赛状态字符串 |
| \`/game_controller/state_code\` | \`std_msgs/Int8\` | 比赛状态码(0-4) |
| \`/game_controller/score\` | \`std_msgs/Int8\` | 本队得分 |
| \`/game_controller/time_remaining\` | \`std_msgs/Int16\` | 剩余时间(秒) |
| \`/game_controller/secondary_time\` | \`std_msgs/Int16\` | 次要时间 |
| \`/game_controller/first_half\` | \`std_msgs/Bool\` | 是否上半场 |
| \`/game_controller/kick_off_team\` | \`std_msgs/Int8\` | 开球队伍编号 |
| \`/game_controller/secondary_state\` | \`std_msgs/String\` | 次要状态 |

### 感知层（B组）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| \`/vision/ball\` | \`maxwell_msgs/Ball\` | 视觉识别球的位置 |
| \`/world/state\` | \`maxwell_msgs/WorldState\` | 融合后的世界状态 |

### 决策层（C组）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| \`/behavior/cmd\` | \`maxwell_msgs/BehaviorCmd\` | 行为指令 |
| \`/walk/self_pose\` | \`maxwell_msgs/Odom\` | 步态输出的自身位姿 |

### 执行层（C组）
| 话题 | 消息类型 | 说明 |
|------|---------|------|
| \`/joint_command\` | \`maxwell_msgs/JointCommand\` | 关节目标角度（弧度） |

## 自定义消息类型

### \`maxwell_msgs/WorldState\`
\`\`\`
std_msgs/Header header
geometry_msgs/Pose2D self_pose   # 机器人自身位姿(x/y/theta)
bool self_pose_sure              # 位姿是否可信
maxwell_msgs/Ball ball           # 球的信息（嵌套）
int8 fall_direction              # 0=无 1=前 -1=后 2=左 -2=右
uint8 support_foot               # 0=双脚 1=左脚 2=右脚
\`\`\`

### \`maxwell_msgs/Ball\`
\`\`\`
std_msgs/Header header
bool visible           # 是否看到球
float32 global_x       # 球全局X坐标（米）
float32 global_y       # 球全局Y坐标（米）
float32 self_x         # 球相对机器人X（米）
float32 self_y         # 球相对机器人Y（米）
int32 pixel_x          # 像素坐标X（-1表示未知）
int32 pixel_y          # 像素坐标Y（-1表示未知）
float32 alpha          # 摄像头水平角
float32 beta           # 摄像头垂直角
\`\`\`

### \`maxwell_msgs/Odom\`
\`\`\`
std_msgs/Header header
float32 x      # 机器人X坐标
float32 y      # 机器人Y坐标
float32 yaw    # 机器人朝向
bool sure      # 位姿是否可信
\`\`\`

### \`maxwell_msgs/BehaviorCmd\`
\`\`\`
uint8 type          # 行为类型
float32 x           # 目标X
float32 y           # 目标Y
float32 dir         # 目标方向
bool enable         # 是否启用
string action_name  # 动作名称
\`\`\`

### \`maxwell_msgs/JointCommand\`
\`\`\`
float32[] positions    # 关节目标角度（弧度）
float32[] velocities   # 关节目标速度
\`\`\`

## 编译方法

\`\`\`bash
# 编译所有包（maxwell_vision 需要CUDA，在开发机上跳过）
cd ~/ros2_ws
colcon build --packages-ignore maxwell_vision

# 编译指定包
colcon build --packages-select maxwell_msgs

# 编译后激活环境
source ~/ros2_ws/install/setup.bash
\`\`\`

## 启动方法

\`\`\`bash
# 不带裁判盒（调试用）
ros2 launch maxwell_bringup play_without_gc.launch.py

# 带裁判盒（比赛用）
ros2 launch maxwell_bringup play_with_gc.launch.py
\`\`\`

## 单节点调试（仿真模式）

\`\`\`bash
# 注意：mock_mode 参数类型为 bool，用 true/false
ros2 run imu_driver imu_node --ros-args -p mock_mode:=true
ros2 run camera_driver camera_node --ros-args -p mock_mode:=true
ros2 run game_controller_driver game_controller_node --ros-args -p mock_mode:=true
ros2 run motor_driver motor_node --ros-args -p mock_mode:=true
ros2 run maxwell_world worldmodel_node
ros2 run maxwell_behavior fsm_node
ros2 run maxwell_motion walk_node
\`\`\`

## 各组开发须知

### 使用自定义消息
在 \`package.xml\` 中添加：
\`\`\`xml
<depend>maxwell_msgs</depend>
\`\`\`

在 \`CMakeLists.txt\` 中添加：
\`\`\`cmake
find_package(maxwell_msgs REQUIRED)
\`\`\`

在 C++ 代码中引用：
\`\`\`cpp
#include "maxwell_msgs/msg/world_state.hpp"
#include "maxwell_msgs/msg/behavior_cmd.hpp"
#include "maxwell_msgs/msg/joint_command.hpp"
#include "maxwell_msgs/msg/ball.hpp"
#include "maxwell_msgs/msg/odom.hpp"
\`\`\`

### 注意事项
- \`maxwell_vision\` 依赖 CUDA/Darknet，只能在机器人上编译
- \`mock_mode\` 参数类型为 bool，启动时用 \`--ros-args -p mock_mode:=true\`
- 编译完成后必须重新 \`source ~/ros2_ws/install/setup.bash\`


