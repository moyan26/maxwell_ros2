#include "worldmodel.hpp"

#if __has_include("localization/soccermap.h")
  #include "localization/soccermap.h"
  #define WM_HAS_SOCCERMAP 1
#else
  #define WM_HAS_SOCCERMAP 0
#endif

#include <mutex>

using Eigen::Vector2d;
using Eigen::Vector2i;

WorldModel::WorldModel()
{
    fall_direction_.store(FALL_NONE);
    support_foot_.store(robot::DOUBLE_SUPPORT);

    const int self_id = self_id_fallback_();
    player_infos_[self_id].id = self_id;

#if WM_HAS_SOCCERMAP && defined(SOCCERMAP)
    opp_post_left  = Vector2d(SOCCERMAP->width() / 200.0,  SOCCERMAP->goalWidth() / 200.0 - 0.3);
    opp_post_right = Vector2d(SOCCERMAP->width() / 200.0, -SOCCERMAP->goalWidth() / 200.0 - 0.1);
    opp_post_mid   = Vector2d(SOCCERMAP->width() / 200.0,  0.0);
#else
    // fallback values already set in header
#endif
}

void WorldModel::update_imu(const ImuData& data, int fall_dir)
{
    const int self_id = self_id_fallback_();

    std::scoped_lock lk(imu_mtx_, self_mtx_, info_mtx_);
    imu_data_ = data;

    fall_direction_.store(fall_dir);

    // yaw used as heading
    self_block_.dir = data.yaw;
    player_infos_[self_id].dir = static_cast<float>(data.yaw);
}

void WorldModel::update_buttons(bool bt1, bool bt2)
{
    bt1_status_.store(bt1);
    bt2_status_.store(bt2);
}

void WorldModel::update_gamectrl(const GameCtrlData& data)
{
    std::lock_guard<std::mutex> lk(gc_mtx_);
    gc_data_ = data;
}

void WorldModel::update_hear_info(const player_info& info)
{
    std::lock_guard<std::mutex> lk(info_mtx_);
    player_infos_[info.id] = info;
}

void WorldModel::set_my_pos(const Vector2d& my)
{
    const int self_id = self_id_fallback_();

    std::scoped_lock lk(info_mtx_, self_mtx_);
    player_infos_[self_id].x = static_cast<float>(my.x());
    player_infos_[self_id].y = static_cast<float>(my.y());
    self_block_.global = my;
}

void WorldModel::set_my_pose(const Vector2d& my, double dir, bool sure)
{
    const int self_id = self_id_fallback_();

    std::scoped_lock lk(info_mtx_, self_mtx_);
    player_infos_[self_id].x   = static_cast<float>(my.x());
    player_infos_[self_id].y   = static_cast<float>(my.y());
    player_infos_[self_id].dir = static_cast<float>(dir);

    self_block_.global = my;
    self_block_.dir    = dir;
    self_block_.sure   = sure;
}

void WorldModel::set_ball_pos(const Vector2d& global,
                              const Vector2d& my,
                              const Vector2i& pix,
                              float alpha, float beta, bool can)
{
    const int self_id = self_id_fallback_();

    std::scoped_lock lk(info_mtx_, ball_mtx_);
    player_infos_[self_id].ball_x  = static_cast<float>(global.x());
    player_infos_[self_id].ball_y  = static_cast<float>(global.y());
    player_infos_[self_id].can_see = can;

    ball_block_.global  = global;
    ball_block_.self    = my;
    ball_block_.pixel   = pix;
    ball_block_.alpha   = alpha;
    ball_block_.beta    = beta;
    ball_block_.can_see = can;
}

void WorldModel::reset_hear_info()
{
    const int self_id = self_id_fallback_();

    std::lock_guard<std::mutex> lk(info_mtx_);
    for (auto &item : player_infos_)
    {
        if (item.first != self_id)
            item.second.can_see = false;
    }
}

#ifdef MAXWELL_LEGACY_SENSORS

void WorldModel::updata(const pub_ptr& pub, const int& type)
{
    // Keep legacy compatibility if you still build old sensor framework.
    if (type == Sensor::SENSOR_IMU)
    {
        std::shared_ptr<Imu> sptr = std::dynamic_pointer_cast<Imu>(pub);
        if (!sptr) return;
        // Use legacy IMU's data struct if you want; here we map to lightweight ImuData.
        const auto d = sptr->data();
        ImuData id;
        id.pitch = d.pitch; id.roll = d.roll; id.yaw = d.yaw;
        id.ax = d.ax; id.ay = d.ay; id.az = d.az;
        id.wx = d.wx; id.wy = d.wy; id.wz = d.wz;
        id.timestamp = d.timestamp;
        this->update_imu(id, sptr->fall_direction());
        return;
    }

    if (type == Sensor::SENSOR_BUTTON)
    {
        std::shared_ptr<Button> sptr = std::dynamic_pointer_cast<Button>(pub);
        if (!sptr) return;
        this->update_buttons(sptr->button_1(), sptr->button_2());
        return;
    }

    if (type == Sensor::SENSOR_GC)
    {
        std::shared_ptr<GameCtrl> sptr = std::dynamic_pointer_cast<GameCtrl>(pub);
        if (!sptr) return;
        this->update_gamectrl(sptr->data());
        return;
    }

    if (type == Sensor::SENSOR_HEAR)
    {
        std::shared_ptr<Hear> sptr = std::dynamic_pointer_cast<Hear>(pub);
        if (!sptr) return;
        this->update_hear_info(sptr->info());
        return;
    }
}

#endif
