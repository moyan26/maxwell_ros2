# camera_driver

该包用于发布摄像头图像话题。当前支持模拟模式，便于无硬件联调。

## 发布话题

- `/camera/image_raw`
- 消息类型：`sensor_msgs/msg/Image`（BGR8）

## 查看消息

```bash
ros2 topic echo /camera/image_raw --no-arr
ros2 topic echo /camera/image_raw --once --no-arr
```

## 可视化

```bash
ros2 run rqt_image_view rqt_image_view
ros2 run rviz2 rviz2
```

## 运行

```bash
colcon build --base-paths src/camera_driver
source install/setup.bash
ros2 run camera_driver camera_node --ros-args -p mock_mode:=true -p mock_pattern:="color_bar"
```

说明：
- `mock_mode` 默认 `true`
- `mock_pattern` 默认 `color_bar`，也可设置为 `noise`
