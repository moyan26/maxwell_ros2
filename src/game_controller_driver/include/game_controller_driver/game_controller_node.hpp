#ifndef __GAME_CONTROLLER_NODE_HPP
#define __GAME_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

namespace game_controller_driver
{

// RoboCup Game Controller 数据结构（与原代码兼容，在源代码的udp_data/RoboCupGameControlData.h文件中）
#pragma pack(push, 1)

struct RobotInfo
{
  uint8_t penalty;              // 犯规状态
  uint8_t secsTillUnpenalised;  // 解除犯规估计时间
  uint8_t yellowCardCount;      // 黄牌数
  uint8_t redCardCount;         // 红牌数
};

struct TeamInfo
{
  uint8_t teamNumber;           // 队伍编号
  uint8_t teamColour;           // 队伍颜色
  uint8_t score;                // 得分
  uint8_t penaltyShot;          // 点球次数
  uint16_t singleShots;         // 点球成功位图
  uint8_t coachSequence;        // 教练消息序号
  uint8_t coachMessage[253];    // 教练消息
  RobotInfo coach;
  RobotInfo players[11];        // 队员信息
};

struct RoboCupGameControlData
{
  char header[4];               // "RGme"
  uint16_t version;             // 版本号
  uint8_t packetNumber;         // 包序号
  uint8_t playersPerTeam;       // 每队人数
  uint8_t gameType;             // 比赛类型
  uint8_t state;                // 比赛状态
  uint8_t firstHalf;            // 是否上半场
  uint8_t kickOffTeam;          // 开球队伍
  uint8_t secondaryState;       // 次要状态
  char secondaryStateInfo[4];   // 次要状态信息
  uint8_t dropInTeam;           // 坠球队伍
  uint16_t dropInTime;          // 坠球时间
  uint16_t secsRemaining;       // 半场剩余时间
  uint16_t secondaryTime;       // 次要时间
  TeamInfo teams[2];            // 两队信息
};

// 回复数据结构
struct RoboCupGameControlReturnData
{
  char header[4];               // "RGrt"
  uint8_t version;              // 版本号
  uint8_t team;                 // 队伍号
  uint8_t player;               // 队员号
  uint8_t message;              // 消息类型
};

#pragma pack(pop)

// 常量定义
constexpr int GAMECONTROLLER_DATA_PORT = 3838;
constexpr int GAMECONTROLLER_RETURN_PORT = 3939;
constexpr char GAMECONTROLLER_STRUCT_HEADER[] = "RGme";
constexpr char GAMECONTROLLER_RETURN_STRUCT_HEADER[] = "RGrt";

// 游戏状态枚举
enum GameState
{
  STATE_INITIAL = 0,
  STATE_READY = 1,
  STATE_SET = 2,
  STATE_PLAYING = 3,
  STATE_FINISHED = 4
};

// 次要状态枚举
enum SecondaryState
{
  STATE2_NORMAL = 0,
  STATE2_PENALTYSHOOT = 1,
  STATE2_OVERTIME = 2,
  STATE2_TIMEOUT = 3,
  STATE2_DIRECT_FREEKICK = 4,
  STATE2_INDIRECT_FREEKICK = 5,
  STATE2_PENALTYKICK = 6
};

/**
 * @brief Game Controller ROS2 节点类
 * 
 * 接收 RoboCup Game Controller 的 UDP 广播
 * 发布比赛状态到 ROS2 Topic
 */
class GameControllerNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    explicit GameControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief 析构函数
     */
    ~GameControllerNode();

    /**
     * @brief 启动接收
     */
    bool start();

    /**
     * @brief 停止接收
     */
    void stop();

private:
    /**
     * @brief 接收线程函数
     */
    void receiveLoop();

    /**
     * @brief 发送存活消息到 Game Controller
     */
    void sendAliveMessage();

    /**
     * @brief 处理接收到的数据
     */
    void processData(const RoboCupGameControlData &data);

    /**
     * @brief 将状态码转换为字符串
     */
    const char *stateToString(uint8_t state);

    /**
     * @brief 将次要状态码转换为字符串
     */
    const char *secondaryStateToString(uint8_t state);

    // ============== 发布者 ==============
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr game_state_pub_;      // 比赛状态字符串
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr game_state_code_pub_;   // 比赛状态码
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr score_pub_;             // 比分 [我方, 对方]
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr time_remaining_pub_;   // 剩余时间
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr secondary_time_pub_;   // 次要时间
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr first_half_pub_;        // 是否上半场
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr kick_off_team_pub_;     // 开球队伍
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr secondary_state_pub_; // 次要状态

    // ============== 参数 ==============
    int team_number_;           // 本队编号
    int player_number_;         // 本机器人编号
    int receive_port_;          // 接收端口
    int return_port_;           // 回复端口
    bool send_alive_;           // 是否发送存活消息
    double alive_interval_;     // 存活消息发送间隔（秒）

    // ============== 网络 ==============
    int socket_fd_;
    struct sockaddr_in recv_addr_;
    struct sockaddr_in return_addr_;

    // ============== 线程控制 ==============
    std::atomic_bool is_running_{false};
    std::thread receive_thread_;
    std::thread alive_thread_;

    // ============== 状态缓存 ==============
    uint8_t last_state_;
    uint8_t last_secondary_state_;
    int last_score_;
    std::mutex data_mutex_;
};

} // namespace game_controller_driver

#endif // __GAME_CONTROLLER_NODE_HPP
