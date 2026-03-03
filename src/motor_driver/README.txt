#这是motor ros 节点
#它订阅/joint_commands话题（关节的目标角度）
#它发布/joint_states话题（关节实际角度）
注：角度均采用弧度制

#启动该节点
1.cd 工作空间
2.编译该ros包：colcon build --base-paths src/motor_driver
3.编译成功后： source install/setup.bash
4.启动该节点：ros2 run motor_driver motor_node

#调试
1.人工发布/joint_commands命令：
ros2 topic pub /joint_commands sensor_msgs/msg/JointState "{
  name: ['jhead1', 'jhead2'],
  position: [0.5, -0.3]
}"
2.查看关节的角度：
ros2 run motor_driver joint_state_viewer #推荐
ros2 topic echo /joint_states #这个不太容易看
