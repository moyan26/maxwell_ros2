#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iomanip>
#include <map>

class JointStateViewer : public rclcpp::Node
{
public:
    JointStateViewer() : Node("joint_state_viewer")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                // 缓存最新数据
                latest_msg_ = *msg;
                has_new_data_ = true;
            });

        // 创建定时器，每秒刷新一次显示
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                if (has_new_data_) {
                    printJointStates();
                    has_new_data_ = false;
                }
            });

        RCLCPP_INFO(this->get_logger(), "Joint State Viewer started");
        RCLCPP_INFO(this->get_logger(), "Refresh rate: 1 Hz (press Ctrl+C to exit)");
        RCLCPP_INFO(this->get_logger(), "Waiting for /joint_states...");
    }

private:
    void printJointStates()
    {
        auto msg = std::make_shared<sensor_msgs::msg::JointState>(latest_msg_);
        
        // 清屏（ANSI escape code）
        std::cout << "\033[2J\033[H";
        
        std::cout << "╔══════════════════════════════════════════════════════════╗\n";
        std::cout << "║           Robot Joint States (18 DOF)                    ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════╣\n";
        std::cout << "║  关节名称          位置(度)    速度      力矩            ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════╣\n";

        // 按部位分组显示
        printGroup("头部 (Head)", msg, {"jhead1", "jhead2"});
        printGroup("左臂 (Left Arm)", msg, {"jlshoulder1", "jlelbow"});
        printGroup("右臂 (Right Arm)", msg, {"jrshoulder1", "jrelbow"});
        printGroup("左腿 (Left Leg)", msg, {"jlhip3", "jlhip2", "jlhip1", "jlknee", "jlankle2", "jlankle1"});
        printGroup("右腿 (Right Leg)", msg, {"jrhip3", "jrhip2", "jrhip1", "jrknee", "jrankle2", "jrankle1"});

        std::cout << "╚══════════════════════════════════════════════════════════╝\n";
        std::cout << "提示: 位置单位为度，ros2 topic pub /joint_commands 发送目标位置\n";
    }

    void printGroup(const std::string &group_name, 
                    const std::shared_ptr<sensor_msgs::msg::JointState> msg,
                    const std::vector<std::string> &joint_names)
    {
        std::cout << "║ [" << group_name << "]\n";
        
        for (const auto &name : joint_names) {
            // 查找关节索引
            auto it = std::find(msg->name.begin(), msg->name.end(), name);
            if (it != msg->name.end()) {
                size_t idx = std::distance(msg->name.begin(), it);
                
                // 获取数据（转换为度）
                double pos_deg = msg->position[idx] * 180.0 / M_PI;
                double vel = (msg->velocity.size() > idx) ? msg->velocity[idx] : 0.0;
                double eff = (msg->effort.size() > idx) ? msg->effort[idx] : 0.0;
                
                // 格式化输出
                std::cout << "║  " << std::left << std::setw(16) << name
                          << std::right << std::setw(8) << std::fixed << std::setprecision(1) << pos_deg
                          << std::setw(8) << std::setprecision(1) << vel
                          << std::setw(10) << std::setprecision(2) << eff
                          << "\n";
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState latest_msg_;
    bool has_new_data_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateViewer>());
    rclcpp::shutdown();
    return 0;
}
