# game_controller_driver

该包用于接收 RoboCup Game Controller 数据并发布比赛状态话题。

## 发布话题

- `/game_controller/state`：`std_msgs/String`
- `/game_controller/state_code`：`std_msgs/Int8`
- `/game_controller/score`：`std_msgs/Int8`
- `/game_controller/time_remaining`：`std_msgs/Int16`
- `/game_controller/secondary_time`：`std_msgs/Int16`
- `/game_controller/first_half`：`std_msgs/Bool`
- `/game_controller/kick_off_team`：`std_msgs/Int8`
- `/game_controller/secondary_state`：`std_msgs/String`

## 运行

```bash
colcon build --base-paths src/game_controller_driver
source install/setup.bash
ros2 run game_controller_driver game_controller_node
```

## 调试

```bash
ros2 run game_controller_driver game_controller_mock
ros2 topic echo /game_controller/state
ros2 topic echo /game_controller/score
```
