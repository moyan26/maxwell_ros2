#这是camera ros节点
#由于目前手头没有摄像头硬件，无法验证图像的采集，故采用仿真的形式，随机生成图像数据(mock_mode=true，采用仿真模式)
#这是一个简化版本，没有调用MVSDK，使用的是 OpenCV 的 cv::VideoCapture，用来进行简单验证。这是为了保证topic能够正常发布，后续其他节点能够正常订阅，以确保整条路线能够正常运行。

它发布Topic：camera/image_raw
消息类型：sensor_msgs/msg/image,是ros官方定义的消息类型，用来传输图像数据（BGR8 格式）。

#查看消息内容
```
#只看消息结构，不打印巨大的 data 数组（--no-arr 是关键）
ros2 topic echo /camera/image_raw --no-arr

# 或只看一次
ros2 topic echo /camera/image_raw --once --no-arr
```

#图像可视化工具
```
# 最轻量，自动识别所有图像话题
ros2 run rqt_image_view rqt_image_view

# 或用你之前问的 rviz2，添加 Image 显示
ros2 run rviz2 rviz2
```

#如何开启该节点
先cd到工作区，然后编译：
colcon build --base-paths src/camera_driver
编译成功后，在命令行输入：
source install/setup.bash
最后输入，启动该节点：
ros2 run camera_driver camera_node --ros-args -p mock_mode:=true -p mock_pattern:="color_bar"

注：mock_mode默认为true,也就是采用仿真模式，为false时，需要接入一个USB摄像机设备（目前还未验证）
mock_pattern默认为"color_bar"，仿真为彩色条纹，也可以设置为"noise"，仿真为随机噪声。

