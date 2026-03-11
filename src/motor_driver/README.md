# motor_driver

该包用于接收关节目标并发布关节状态。

## 话题

- 订阅：`/joint_commands`（`sensor_msgs/msg/JointState`）
- 发布：`/joint_states`（`sensor_msgs/msg/JointState`）

说明：角度均采用弧度制。

## 运行

```bash
colcon build --base-paths src/motor_driver
source install/setup.bash
ros2 run motor_driver motor_node
```

## 调试

```bash
ros2 topic pub /joint_commands sensor_msgs/msg/JointState "{
  name: ['jhead1', 'jhead2'],
  position: [0.5, -0.3]
}"
ros2 run motor_driver joint_state_viewer
ros2 topic echo /joint_states
```
