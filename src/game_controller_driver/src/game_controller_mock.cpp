#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// 简化版数据结构（与主节点兼容）
#pragma pack(push, 1)
struct RobotInfo
{
    uint8_t penalty;
    uint8_t secsTillUnpenalised;
    uint8_t yellowCardCount;
    uint8_t redCardCount;
};

struct TeamInfo
{
    uint8_t teamNumber;
    uint8_t teamColour;
    uint8_t score;
    uint8_t penaltyShot;
    uint16_t singleShots;
    uint8_t coachSequence;
    uint8_t coachMessage[253];
    RobotInfo coach;
    RobotInfo players[11];
};

struct RoboCupGameControlData
{
    char header[4];
    uint16_t version;
    uint8_t packetNumber;
    uint8_t playersPerTeam;
    uint8_t gameType;
    uint8_t state;
    uint8_t firstHalf;
    uint8_t kickOffTeam;
    uint8_t secondaryState;
    char secondaryStateInfo[4];
    uint8_t dropInTeam;
    uint16_t dropInTime;
    uint16_t secsRemaining;
    uint16_t secondaryTime;
    TeamInfo teams[2];
};
#pragma pack(pop)

constexpr int GAMECONTROLLER_DATA_PORT = 3838;
constexpr char GAMECONTROLLER_STRUCT_HEADER[] = "RGme";

class GameControllerMock : public rclcpp::Node
{
public:
    GameControllerMock() : Node("game_controller_mock")
    {
        this->declare_parameter<int>("team_number", 1);
        this->declare_parameter<double>("state_interval", 10.0);  // 状态切换间隔

        team_number_ = this->get_parameter("team_number").as_int();
        state_interval_ = this->get_parameter("state_interval").as_double();

        // 创建 UDP 发送套接字
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        // 设置广播地址
        memset(&broadcast_addr_, 0, sizeof(broadcast_addr_));
        broadcast_addr_.sin_family = AF_INET;
        broadcast_addr_.sin_addr.s_addr = inet_addr("127.0.0.1");  // 本地回环
        broadcast_addr_.sin_port = htons(GAMECONTROLLER_DATA_PORT);

        // 启动发送线程
        is_running_ = true;
        send_thread_ = std::thread(&GameControllerMock::sendLoop, this);

        RCLCPP_INFO(this->get_logger(), "Game Controller Mock started");
        RCLCPP_INFO(this->get_logger(), "Simulating team %d, state changes every %.1f seconds",
                    team_number_, state_interval_);
        RCLCPP_INFO(this->get_logger(), "States: INITIAL -> READY -> SET -> PLAYING -> FINISHED");
    }

    ~GameControllerMock()
    {
        is_running_ = false;
        if (send_thread_.joinable()) {
            send_thread_.join();
        }
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
    }

private:
    void sendLoop()
    {
        RoboCupGameControlData data;
        uint8_t packet_num = 0;
        auto last_state_change = std::chrono::steady_clock::now();
        uint8_t current_state = 0;  // INITIAL

        // 初始化数据结构
        memset(&data, 0, sizeof(data));
        memcpy(data.header, GAMECONTROLLER_STRUCT_HEADER, 4);
        data.version = 12;
        data.playersPerTeam = 4;
        data.gameType = 0;  // round robin
        data.firstHalf = 1;
        data.kickOffTeam = team_number_;
        data.secondaryState = 0;  // NORMAL
        data.secsRemaining = 600;  // 10 minutes

        // 设置队伍信息
        data.teams[0].teamNumber = team_number_;
        data.teams[0].teamColour = 0;  // blue
        data.teams[0].score = 0;

        data.teams[1].teamNumber = (team_number_ == 1) ? 2 : 1;
        data.teams[1].teamColour = 1;  // red
        data.teams[1].score = 0;

        while (is_running_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - last_state_change).count();

            // 定期切换状态
            if (elapsed >= state_interval_) {
                current_state++;
                if (current_state > 4) {  // FINISHED
                    current_state = 0;  // 回到 INITIAL
                    // 切换下半场
                    data.firstHalf = (data.firstHalf == 1) ? 0 : 1;
                    data.secsRemaining = 600;
                    
                    // 增加比分（模拟进球）
                    if (data.teams[0].score < 5) {
                        data.teams[0].score++;
                    }
                }
                last_state_change = now;

                // 根据状态设置次要时间
                switch (current_state) {
                    case 1: data.secondaryTime = 45; break;  // READY: 45s
                    case 2: data.secondaryTime = 10; break;  // SET: 10s
                    default: data.secondaryTime = 0; break;
                }

                // 打印状态变化
                const char* state_names[] = {"INITIAL", "READY", "SET", "PLAYING", "FINISHED"};
                RCLCPP_INFO(this->get_logger(), "State changed to: %s", state_names[current_state]);
            }

            // 更新数据
            data.packetNumber = packet_num++;
            data.state = current_state;
            data.secsRemaining = (data.secsRemaining > 0) ? data.secsRemaining - 1 : 0;

            // 发送数据
            sendto(socket_fd_, &data, sizeof(data), 0,
                   (struct sockaddr *)&broadcast_addr_, sizeof(broadcast_addr_));

            // 10Hz 发送频率
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    int socket_fd_;
    struct sockaddr_in broadcast_addr_;
    int team_number_;
    double state_interval_;
    std::atomic_bool is_running_{false};
    std::thread send_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameControllerMock>());
    rclcpp::shutdown();
    return 0;
}
