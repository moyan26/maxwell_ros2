#这是Game Controller节点
#下面是这个节点发布的话题
Topic	消息类型	说明
/game_controller/state	std_msgs/String	比赛状态字符串<br>(INITIAL/READY/SET/PLAYING/FINISHED)
/game_controller/state_code	std_msgs/Int8	比赛状态码<br>(0/1/2/3/4)
/game_controller/score	std_msgs/Int8	本队当前得分
/game_controller/time_remaining	std_msgs/Int16	半场剩余时间（秒）
/game_controller/secondary_time	std_msgs/Int16	次要时间（如准备时间）
/game_controller/first_half	std_msgs/Bool	是否上半场
/game_controller/kick_off_team	std_msgs/Int8	开球队伍编号
/game_controller/secondary_state	std_msgs/String	次要状态<br>(NORMAL/PENALTY_SHOOT 等)

#启动节点步骤
1.切换到工作空间目录
2.单独编译这个ros包：
colcon build --base-paths src/game_controller_driver 
3.编译成功后，在命令行输入：source install/setup.bash
4.启动节点：ros2 run game_controller_driver game_controller_node
5.启动模拟发送器（调试时使用）： ros2 run game_controller_driver game_controller_mock

#查看话题
ros2 topic echo /game_controller/state
ros2 topic echo /game_controller/score
...

注：我是在ubuntu22.04的环境下进行的编译。
