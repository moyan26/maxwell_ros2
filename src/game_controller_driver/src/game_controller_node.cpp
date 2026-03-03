#include "game_controller_driver/game_controller_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace game_controller_driver
{

GameControllerNode::GameControllerNode(const rclcpp::NodeOptions &options)
    : Node("game_controller_node", options)
    , socket_fd_(-1)
    , last_state_(255)
    , last_secondary_state_(255)
    , last_score_(-1)
{
    // ============== 声明并获取参数 ==============
    this->declare_parameter<int>("team_number", 1);
    this->declare_parameter<int>("player_number", 1);
    this->declare_parameter<int>("receive_port", GAMECONTROLLER_DATA_PORT);
    this->declare_parameter<int>("return_port", GAMECONTROLLER_RETURN_PORT);
    this->declare_parameter<bool>("send_alive", true);
    this->declare_parameter<double>("alive_interval", 1.0);
    this->declare_parameter<bool>("mock_mode", false);

    team_number_ = this->get_parameter("team_number").as_int();
    player_number_ = this->get_parameter("player_number").as_int();
    receive_port_ = this->get_parameter("receive_port").as_int();
    return_port_ = this->get_parameter("return_port").as_int();
    send_alive_ = this->get_parameter("send_alive").as_bool();
    alive_interval_ = this->get_parameter("alive_interval").as_double();

    // ============== 创建发布者 ==============
    game_state_pub_ = this->create_publisher<std_msgs::msg::String>("game_controller/state", 10);
    game_state_code_pub_ = this->create_publisher<std_msgs::msg::Int8>("game_controller/state_code", 10);
    score_pub_ = this->create_publisher<std_msgs::msg::Int8>("game_controller/score", 10);
    time_remaining_pub_ = this->create_publisher<std_msgs::msg::Int16>("game_controller/time_remaining", 10);
    secondary_time_pub_ = this->create_publisher<std_msgs::msg::Int16>("game_controller/secondary_time", 10);
    first_half_pub_ = this->create_publisher<std_msgs::msg::Bool>("game_controller/first_half", 10);
    kick_off_team_pub_ = this->create_publisher<std_msgs::msg::Int8>("game_controller/kick_off_team", 10);
    secondary_state_pub_ = this->create_publisher<std_msgs::msg::String>("game_controller/secondary_state", 10);

    RCLCPP_INFO(this->get_logger(), "Game Controller Node initialized");
    RCLCPP_INFO(this->get_logger(), "Team: %d, Player: %d", team_number_, player_number_);
    RCLCPP_INFO(this->get_logger(), "Receive port: %d, Return port: %d", receive_port_, return_port_);
}

GameControllerNode::~GameControllerNode()
{
    stop();
}

bool GameControllerNode::start()
{
    if (is_running_) {
        return true;
    }

    // 创建 UDP 套接字
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
        return false;
    }

    // 设置地址可重用
    int reuse = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set socket option");
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // 绑定接收地址
    memset(&recv_addr_, 0, sizeof(recv_addr_));
    recv_addr_.sin_family = AF_INET;
    recv_addr_.sin_addr.s_addr = INADDR_ANY;
    recv_addr_.sin_port = htons(receive_port_);

    if (bind(socket_fd_, (struct sockaddr *)&recv_addr_, sizeof(recv_addr_)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind socket to port %d", receive_port_);
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // 设置非阻塞模式
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    RCLCPP_INFO(this->get_logger(), "Socket bound to port %d", receive_port_);

    // 启动接收线程
    is_running_ = true;
    receive_thread_ = std::thread(&GameControllerNode::receiveLoop, this);

    // 启动存活消息线程
    if (send_alive_) {
        alive_thread_ = std::thread([this]() {
            while (is_running_) {
                sendAliveMessage();
                std::this_thread::sleep_for(std::chrono::duration<double>(alive_interval_));
            }
        });
    }

    RCLCPP_INFO(this->get_logger(), "Game Controller Node started successfully");
    return true;
}

void GameControllerNode::stop()
{
    is_running_ = false;

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    if (alive_thread_.joinable()) {
        alive_thread_.join();
    }

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    RCLCPP_INFO(this->get_logger(), "Game Controller Node stopped");
}

void GameControllerNode::receiveLoop()
{
    RoboCupGameControlData data;
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    while (is_running_) {
        ssize_t recv_len = recvfrom(socket_fd_, &data, sizeof(data), 0,
                                    (struct sockaddr *)&sender_addr, &sender_len);

        if (recv_len == sizeof(RoboCupGameControlData)) {
            // 验证头部
            if (strncmp(data.header, GAMECONTROLLER_STRUCT_HEADER, 4) == 0) {
                processData(data);
                
                // 保存回复地址
                return_addr_ = sender_addr;
                return_addr_.sin_port = htons(return_port_);
            }
        }

        // 短暂休眠避免 CPU 占用过高
        std::this_thread::sleep_for(1ms);
    }
}

void GameControllerNode::sendAliveMessage()
{
    if (socket_fd_ < 0) {
        return;
    }

    RoboCupGameControlReturnData return_data;
    memcpy(return_data.header, GAMECONTROLLER_RETURN_STRUCT_HEADER, 4);
    return_data.version = 2;
    return_data.team = static_cast<uint8_t>(team_number_);
    return_data.player = static_cast<uint8_t>(player_number_);
    return_data.message = 2;  // ALIVE

    sendto(socket_fd_, &return_data, sizeof(return_data), 0,
           (struct sockaddr *)&return_addr_, sizeof(return_addr_));
}

void GameControllerNode::processData(const RoboCupGameControlData &data)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 判断本队是 teams[0] 还是 teams[1]
    int my_team_idx = -1;
    int opponent_idx = -1;
    for (int i = 0; i < 2; i++) {
        if (data.teams[i].teamNumber == team_number_) {
            my_team_idx = i;
            opponent_idx = 1 - i;
            break;
        }
    }

    if (my_team_idx == -1) {
        // 本队不在数据中
        return;
    }

    // 检查状态是否变化
    bool state_changed = (data.state != last_state_) || 
                         (data.secondaryState != last_secondary_state_);
    
    int current_score = data.teams[my_team_idx].score;
    bool score_changed = (current_score != last_score_);

    // 发布比赛状态（仅在变化时）
    if (state_changed) {
        auto state_msg = std_msgs::msg::String();
        state_msg.data = stateToString(data.state);
        game_state_pub_->publish(state_msg);

        auto state_code_msg = std_msgs::msg::Int8();
        state_code_msg.data = static_cast<int8_t>(data.state);
        game_state_code_pub_->publish(state_code_msg);

        auto sec_state_msg = std_msgs::msg::String();
        sec_state_msg.data = secondaryStateToString(data.secondaryState);
        secondary_state_pub_->publish(sec_state_msg);

        RCLCPP_INFO(this->get_logger(), 
            "State changed: %s (%d), Secondary: %s (%d)",
            state_msg.data.c_str(), data.state,
            sec_state_msg.data.c_str(), data.secondaryState);
    }

    // 发布比分（仅在变化时）
    if (score_changed) {
        auto score_msg = std_msgs::msg::Int8();
        score_msg.data = static_cast<int8_t>(current_score);
        score_pub_->publish(score_msg);

        RCLCPP_INFO(this->get_logger(), 
            "Score: %d (us) - %d (opponent)",
            current_score, data.teams[opponent_idx].score);
    }

    // 发布时间信息（每秒更新）
    static auto last_time_pub = this->now();
    if ((this->now() - last_time_pub).seconds() >= 1.0) {
        auto time_msg = std_msgs::msg::Int16();
        time_msg.data = static_cast<int16_t>(data.secsRemaining);
        time_remaining_pub_->publish(time_msg);

        auto sec_time_msg = std_msgs::msg::Int16();
        sec_time_msg.data = static_cast<int16_t>(data.secondaryTime);
        secondary_time_pub_->publish(sec_time_msg);

        auto first_half_msg = std_msgs::msg::Bool();
        first_half_msg.data = (data.firstHalf != 0);
        first_half_pub_->publish(first_half_msg);

        auto kick_off_msg = std_msgs::msg::Int8();
        kick_off_msg.data = static_cast<int8_t>(data.kickOffTeam);
        kick_off_team_pub_->publish(kick_off_msg);

        last_time_pub = this->now();
    }

    // 更新缓存
    last_state_ = data.state;
    last_secondary_state_ = data.secondaryState;
    last_score_ = current_score;
}

const char *GameControllerNode::stateToString(uint8_t state)
{
    switch (state) {
        case STATE_INITIAL: return "INITIAL";
        case STATE_READY: return "READY";
        case STATE_SET: return "SET";
        case STATE_PLAYING: return "PLAYING";
        case STATE_FINISHED: return "FINISHED";
        default: return "UNKNOWN";
    }
}

const char *GameControllerNode::secondaryStateToString(uint8_t state)
{
    switch (state) {
        case STATE2_NORMAL: return "NORMAL";
        case STATE2_PENALTYSHOOT: return "PENALTY_SHOOT";
        case STATE2_OVERTIME: return "OVERTIME";
        case STATE2_TIMEOUT: return "TIMEOUT";
        case STATE2_DIRECT_FREEKICK: return "DIRECT_FREEKICK";
        case STATE2_INDIRECT_FREEKICK: return "INDIRECT_FREEKICK";
        case STATE2_PENALTYKICK: return "PENALTYKICK";
        default: return "UNKNOWN";
    }
}

} // namespace game_controller_driver
