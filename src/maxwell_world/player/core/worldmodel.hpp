#ifndef __WORLD_MODEL_HPP
#define __WORLD_MODEL_HPP

#include <atomic>
#include <map>
#include <mutex>

#if __has_include(<Eigen/Dense>)
  #include <Eigen/Dense>
#elif __has_include(<eigen3/Eigen/Dense>)
  #include <eigen3/Eigen/Dense>
#endif

// ---------------- Minimal domain types (ROS2-friendly) ----------------

// Fall direction (match legacy values)
#ifndef MAXWELL_HAS_FALL_DIRECTION
enum FallDirection
{
    FALL_NONE = 0,
    FALL_FORWARD = 1,
    FALL_BACKWARD = -1,
    FALL_LEFT = 2,
    FALL_RIGHT = -2
};
#endif

// support_foot (prefer project header if exists, else fallback)
#if __has_include("robot/robot_define.hpp")
  #include "robot/robot_define.hpp"
#elif __has_include("robot_define.hpp")
  #include "robot_define.hpp"
#else
namespace robot {
  enum support_foot { DOUBLE_SUPPORT = 0, LEFT_SUPPORT = 1, RIGHT_SUPPORT = 2 };
}
#endif

// model types (prefer project model.hpp, else fallback)
#if __has_include("model.hpp")
  #include "model.hpp"
#else
struct player_info {
    int id = 0;
    float x = 0.0f, y = 0.0f, dir = 0.0f;
    float ball_x = 0.0f, ball_y = 0.0f;
    bool can_see = false;
    bool my_kick = false;
};
struct ball_block {
    Eigen::Vector2d global{0.0, 0.0};
    Eigen::Vector2d self{0.0, 0.0};
    Eigen::Vector2i pixel{-1, -1};
    float alpha = 0.0f, beta = 0.0f;
    bool can_see = false;
};
struct self_block {
    Eigen::Vector2d global{0.0, 0.0};
    double dir = 0.0;
    bool sure = false;
};
#endif

// ROS2 worldmodel_node will produce IMU data, so keep a lightweight struct here.
struct ImuData
{
    double pitch = 0.0, roll = 0.0, yaw = 0.0;
    double ax = 0.0, ay = 0.0, az = 0.0;
    double wx = 0.0, wy = 0.0, wz = 0.0;
    int64_t timestamp = 0; // ms
};

// Optional legacy GameController packet type (kept as placeholder)
#if __has_include("udp_data/RoboCupGameControlData.h")
  #include "udp_data/RoboCupGameControlData.h"
  using GameCtrlData = RoboCupGameControlData;
#else
  struct GameCtrlData { int dummy = 0; };
#endif

// ---------------- Optional legacy adapter (old Sensor/Observer) ----------------
#ifdef MAXWELL_LEGACY_SENSORS
  #if __has_include("observer.hpp")
    #include "observer.hpp"
  #endif
  #if __has_include("sensor/sensor.hpp")
    #include "sensor/sensor.hpp"
  #endif
  #if __has_include("sensor/imu.hpp")
    #include "sensor/imu.hpp"
  #endif
  #if __has_include("sensor/button.hpp")
    #include "sensor/button.hpp"
  #endif
  #if __has_include("sensor/gamectrl.hpp")
    #include "sensor/gamectrl.hpp"
  #endif
  #if __has_include("sensor/hear.hpp")
    #include "sensor/hear.hpp"
  #endif
#endif

class WorldModel
#ifdef MAXWELL_LEGACY_SENSORS
    : public Subscriber
#endif
{
public:
    WorldModel();

    // --- ROS2-friendly explicit update APIs (subscriber callbacks call these) ---
    void update_imu(const ImuData& data, int fall_dir);
    void update_buttons(bool bt1, bool bt2);
    void update_gamectrl(const GameCtrlData& data);
    void update_hear_info(const player_info& info);

    // Pose / ball setters used by worldmodel_node
    void set_my_pos(const Eigen::Vector2d& my);
    void set_my_pose(const Eigen::Vector2d& my, double dir, bool sure);
    void set_ball_pos(const Eigen::Vector2d& global,
                      const Eigen::Vector2d& my,
                      const Eigen::Vector2i& pix,
                      float alpha, float beta, bool can);
    void reset_hear_info();

#ifdef MAXWELL_LEGACY_SENSORS
    void updata(const pub_ptr& pub, const int& type);
#endif

    // --- getters ---
    inline int fall_data() const { return fall_direction_.load(); }

    inline bool button_status(const int& idx) const {
        if (idx == 1) return bt1_status_.load();
        if (idx == 2) return bt2_status_.load();
        return false;
    }

    inline robot::support_foot support_foot() const {
        return support_foot_.load();
    }
    inline void set_support_foot(const robot::support_foot& foot) {
        support_foot_.store(foot);
    }

    inline void set_my_kick(const bool& kick) {
        std::lock_guard<std::mutex> lk(info_mtx_);
        const int self_id = self_id_fallback_();
        player_infos_[self_id].my_kick = kick;
    }

    inline std::map<int, player_info> hear_info() const {
        std::lock_guard<std::mutex> lk(info_mtx_);
        return player_infos_;
    }

    inline ball_block ball() const {
        std::lock_guard<std::mutex> lk(ball_mtx_);
        return ball_block_;
    }

    inline self_block self() const {
        std::lock_guard<std::mutex> lk(self_mtx_);
        return self_block_;
    }

    inline ImuData imu() const {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        return imu_data_;
    }

    inline GameCtrlData gc() const {
        std::lock_guard<std::mutex> lk(gc_mtx_);
        return gc_data_;
    }

private:
    static inline int self_id_fallback_() {
    #if defined(CONF)
        return CONF->id();
    #else
        return 0;
    #endif
    }

private:
    // shared state blocks
    ball_block ball_block_;
    self_block self_block_;

    // heard team info
    std::map<int, player_info> player_infos_;

    // sensors
    ImuData imu_data_{};
    GameCtrlData gc_data_{};

    // atomic flags
    std::atomic<int> fall_direction_{FALL_NONE};
    std::atomic<robot::support_foot> support_foot_{robot::DOUBLE_SUPPORT};
    std::atomic_bool bt1_status_{false};
    std::atomic_bool bt2_status_{false};

    // mutexes
    mutable std::mutex ball_mtx_;
    mutable std::mutex self_mtx_;
    mutable std::mutex info_mtx_;
    mutable std::mutex imu_mtx_;
    mutable std::mutex gc_mtx_;

public:
    // Opponent goal posts (meters)
    Eigen::Vector2d opp_post_left{4.5, 0.75};
    Eigen::Vector2d opp_post_right{4.5, -0.75};
    Eigen::Vector2d opp_post_mid{4.5, 0.0};

    bool localization_time_ = false;
    bool no_power_ = true;
};

#endif
