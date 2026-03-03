# WorldState：从旧工程 WorldModel 抽取的“跨节点状态接口”
# 目标：先让 fsm_node 按旧FSM逻辑做决策，跑通闭环

# 摔倒方向（来自 imu.hpp 的 FallDirection）
# 0=FALL_NONE, 1=FALL_FORWARD, -1=FALL_BACKWARD, 2=FALL_LEFT, -2=FALL_RIGHT
int32 fall_direction

# 旧工程 WM->localization_time_
bool localization_time

# self_block（旧工程 self().global + self().dir）
float32 self_x
float32 self_y
float32 self_dir_deg

# ball_block（旧工程 ball().can_see + ball().self/global）
bool ball_seen
float32 ball_self_x
float32 ball_self_y
float32 ball_global_x
float32 ball_global_y

# 球门柱（旧工程 WM->opp_post_left/right）
float32 opp_post_left_x
float32 opp_post_left_y
float32 opp_post_right_x
float32 opp_post_right_y