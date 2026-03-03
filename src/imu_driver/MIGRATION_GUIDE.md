# ROS2 迁移指南：以 IMU 为例

本文档详细说明如何将原始代码中的感知模块迁移到 ROS2 架构。

## 1. 核心架构变化

### 1.1 原始代码架构

```cpp
// 观察者模式：Sensor 作为 Publisher，其他模块作为 Subscriber
class Sensor : public Publisher {
    void notify(const int &type);  // 通知所有订阅者
};

class Imu : public Sensor {
    void OnDataReceived(Packet_t &pkt) {
        // 解析数据...
        notify(SENSOR_IMU);  // 通知订阅者
    }
};

// 订阅者示例（WorldModel）
class WorldModel : public Subscriber {
    void updata(const pub_ptr &pub, const int &type) {
        if (type == Sensor::SENSOR_IMU) {
            auto imu = std::dynamic_pointer_cast<Imu>(pub);
            auto data = imu->data();  // 通过指针获取数据
            // 使用数据...
        }
    }
};
```

### 1.2 ROS2 架构

```cpp
// ROS2 节点：使用 Topic 发布/订阅
class ImuNode : public rclcpp::Node {
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    void onDataReceived(Packet_t &pkt) {
        // 解析数据...
        publishImuData(data);  // 直接发布到 Topic
    }
    
    void publishImuData(const ImuData &data) {
        auto msg = sensor_msgs::msg::Imu();
        // 填充数据...
        imu_pub_->publish(msg);  // ROS2 发布
    }
};

// 订阅者（任何节点都可以订阅）
class AnyNode : public rclcpp::Node {
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 直接使用 msg 数据
        RCLCPP_INFO(this->get_logger(), "Pitch: %.2f", 
                    msg->orientation.y);  // 示例
    }
};
```

## 2. 详细对比

### 2.1 类定义

| 方面 | 原始代码 | ROS2 |
|------|---------|------|
| **基类** | `Sensor` (自定义 Publisher) | `rclcpp::Node` |
| **头文件** | `#include "sensor.hpp"` | `#include <rclcpp/rclcpp.hpp>` |
| **节点名** | 无（不是节点） | 构造函数中指定 `"imu_node"` |

**原始代码：**
```cpp
#include "sensor.hpp"
#include "model.hpp"

class Imu: public Sensor {
    Imu();  // 构造时自动记录日志
    bool start();
    void stop();
    imu_data data() const { return imu_data_; }
};
```

**ROS2：**
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuNode : public rclcpp::Node {
    explicit ImuNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    bool start();  // 手动调用启动
    void stop();
    // 数据通过 topic 发布，不需要 data() 方法
};
```

### 2.2 配置获取

**原始代码（使用 CONF 单例）：**
```cpp
Imu::Imu(): Sensor("imu"), serial_(Imu_service) {
    Vector2f range = CONF->get_config_vector<float, 2>("not_fall_range.pitch");
    pitch_range_.x() = range[0];
    pitch_range_.y() = range[1];
}
```

**ROS2（使用参数系统）：**
```cpp
ImuNode::ImuNode(const rclcpp::NodeOptions &options)
    : Node("imu_node", options), serial_(io_service_) {
    // 声明参数（带默认值）
    this->declare_parameter<std::string>("dev_name", "/dev/ttyTHS2");
    this->declare_parameter<std::vector<double>>("pitch_range", {-30.0, 40.0});
    
    // 获取参数
    dev_name_ = this->get_parameter("dev_name").as_string();
    auto pitch_vec = this->get_parameter("pitch_range").as_double_array();
    pitch_range_ << pitch_vec[0], pitch_vec[1];
}
```

### 2.3 日志记录

**原始代码：**
```cpp
#include "logger.hpp"
LOG(LOG_WARN) << "imu: " << e.what() << endll;
LOG(LOG_INFO) << "imu started!" << endll;
```

**ROS2：**
```cpp
RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
RCLCPP_INFO(this->get_logger(), "IMU Node started successfully");
RCLCPP_WARN(this->get_logger(), "Fall detected!");
RCLCPP_DEBUG(this->get_logger(), "Pitch: %.2f", pitch);  // 仅 debug 模式输出
```

### 2.4 数据发布

**原始代码（观察者模式）：**
```cpp
// 通知所有订阅者
void Imu::OnDataReceived(Packet_t &pkt) {
    // 解析数据到 imu_data_...
    notify(SENSOR_IMU);  // 订阅者通过 data() 方法获取数据
}

// 订阅者收到通知后
void WorldModel::updata(const pub_ptr &pub, const int &type) {
    if (type == Sensor::SENSOR_IMU) {
        std::shared_ptr<Imu> sptr = std::dynamic_pointer_cast<Imu>(pub);
        imu_data_ = sptr->data();  // 从发布者获取数据
    }
}
```

**ROS2（Topic 发布）：**
```cpp
// 创建发布者（在构造函数中）
imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);

// 直接发布消息
void ImuNode::onDataReceived(Packet_t &pkt) {
    // 解析数据...
    publishImuData(data);
}

void ImuNode::publishImuData(const ImuData &data) {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();  // ROS2 时间戳
    msg.header.frame_id = frame_id_;
    // 填充数据...
    imu_pub_->publish(msg);  // 发布到 topic
}

// 订阅者（完全独立，不需要知道发布者是谁）
void AnyNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // 直接使用 msg 数据
    double wx = msg->angular_velocity.x;
}
```

### 2.5 时间戳

**原始代码：**
```cpp
#include "core/clock.hpp"
imu_data_.timestamp = CLOCK->get_timestamp();  // 自定义时钟
```

**ROS2：**
```cpp
// 使用 ROS2 内置时间
msg.header.stamp = this->now();  // rclcpp::Time

// 或获取当前时间
auto now = this->get_clock()->now();
int64_t ms = now.nanoseconds() / 1000000;
```

### 2.6 角度转换（重要）

**原始代码（欧拉角）：**
```cpp
struct imu_data {
    float pitch, roll, yaw;  // 欧拉角（度）
    float ax, ay, az;        // 加速度
    float wx, wy, wz;        // 角速度
};
// 使用者需要自己处理欧拉角
```

**ROS2（标准 Imu 消息使用四元数）：**
```cpp
// ROS 标准：orientation 必须是四元数
sensor_msgs::msg::Imu msg;

// 使用 tf2 转换欧拉角到四元数
tf2::Quaternion q;
q.setRPY(roll_rad, pitch_rad, yaw_rad);
msg.orientation.x = q.x();
msg.orientation.y = q.y();
msg.orientation.z = q.z();
msg.orientation.w = q.w();

// 订阅者可以再转回欧拉角
tf2::Matrix3x3 m(q);
double roll, pitch, yaw;
m.getRPY(roll, pitch, yaw);
```

## 3. 配置文件迁移

### 3.1 原始配置（JSON）
```json
{
    "hardware": {
        "imu": {
            "dev_name": "/dev/ttyTHS2",
            "baudrate": 115200
        }
    },
    "not_fall_range": {
        "pitch": [-30.0, 40.0],
        "roll": [-40.0, 40.0]
    }
}
```

### 3.2 ROS2 参数文件（YAML）
```yaml
imu_node:
  ros__parameters:
    dev_name: "/dev/ttyTHS2"
    baudrate: 115200
    pitch_range: [-30.0, 40.0]
    roll_range: [-40.0, 40.0]
    frame_id: "imu_link"
```

## 4. 多传感器同步（进阶）

在原始代码中，Vision 模块需要同步 Camera + IMU + 关节角度：

```cpp
// 原始代码：Vision 订阅多个传感器，用队列缓存
class Vision : public Subscriber {
    std::queue<Imu::imu_data> imu_datas_;
    std::queue<std::vector<double>> foot_degs_, head_degs_;
    
    void updata(const pub_ptr &pub, const int &type) {
        if (type == Sensor::SENSOR_CAMERA) {
            // 取最新的 IMU 和关节数据
            Imu::imu_data imu_data_ = imu_datas_.front();
            std::vector<double> foot_degs = foot_degs_.front();
            // 融合处理...
        }
    }
};
```

**ROS2 方案（使用 message_filters）：**
```cpp
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class VisionNode : public rclcpp::Node {
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> joint_sub_;
    
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Imu,
        sensor_msgs::msg::JointState
    >;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    void syncedCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
        const sensor_msgs::msg::JointState::ConstSharedPtr& joints) {
        // 三个数据已经时间同步
        process(image, imu, joints);
    }
};
```

## 5. 调试技巧

### 5.1 查看话题列表
```bash
ros2 topic list
```

### 5.2 查看话题数据
```bash
ros2 topic echo /imu/data
ros2 topic echo /imu/fall_direction
```

### 5.3 查看话题频率
```bash
ros2 topic hz /imu/data
```

### 5.4 记录和回放数据
```bash
# 记录
ros2 bag record /imu/data /imu/fall_direction

# 回放
ros2 bag play bag_file/
```

## 6. 下一步迁移建议

按照此模式，可以依次迁移：

1. **Camera** → `camera_node` → 发布 `/camera/image_raw`
2. **Motor** → `motor_node` → 发布 `/joint_states`, 订阅 `/motion/joint_targets`
3. **Button** → `button_node` → 发布 `/button/pressed`
4. **GameCtrl** → `gamectrl_node` → 发布 `/gamectrl/state`
5. **Hear** → `team_comm_node` → 发布 `/team/comm`
6. **Vision** → `vision_node` → 订阅多个话题，发布 `/vision/ball`, `/vision/goal`
7. **WorldModel** → `worldmodel_node` → 订阅所有感知话题，发布融合后数据
8. **Decision** → `decision_node` → 订阅世界模型，发布动作指令
