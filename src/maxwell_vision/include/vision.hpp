#ifndef __VISION_HPP
#define __VISION_HPP
#include <Eigen/Dense>
#include <atomic>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "timer.hpp"
#include "configuration.hpp"
#include "common.hpp"
#include "singleton.hpp"
#if defined(MAXWELL_VISION_RUNTIME) && MAXWELL_VISION_RUNTIME
  #include "darknet/network.h"
#else
  // compile-only: avoid pulling darknet headers
  struct network {};
#endif
#include "tcp.hpp"
#include "math.hpp"
#ifndef MAXWELL_ROS2
#include "camera.hpp"
#endif

#ifndef MAXWELL_ROS2
#include "observer.hpp"
#include "options.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "localization.hpp"
#endif

// Vision: legacy algorithm module.
// - Non-ROS2 mode keeps the original observer/updata + internal Timer scheduling.
// - ROS2 mode expects images to be fed via set_bgr_frame() and inference triggered via process_once().
class Vision :
    public Timer,
#ifndef MAXWELL_ROS2
    public Subscriber,
#endif
    public Singleton<Vision>
{
public:
    Vision();
    ~Vision();

    int w() const { return w_; }
    int h() const { return h_; }

#ifndef MAXWELL_ROS2
    void updata(const pub_ptr &pub, const int &type);
#endif

    bool start();
    void stop();

    void set_camera_info(const camera_info &para);
    void get_point_dis(int x, int y);

    void set_img_send_type(image_send_type t) { img_sd_type_ = t; }

#ifdef MAXWELL_ROS2
    // Feed latest frame. Contract:
    // - input is BGR8 (CV_8UC3) or compatible (will be converted in .cpp).
    // - thread-safe.
    void set_bgr_frame(const cv::Mat &bgr);

    // Trigger one inference step (call from ROS2 timer callback).
    void process_once() { run(); }

    // Access latest detections (copied out, thread-safe).
    bool get_best_ball(object_det &out) const
    {
        std::lock_guard<std::mutex> lk(det_mtx_);
        if (ball_dets_.empty()) return false;
        out = ball_dets_.front();
        return true;
    }

    bool get_best_post(object_det &out) const
    {
        std::lock_guard<std::mutex> lk(det_mtx_);
        if (post_dets_.empty()) return false;
        out = post_dets_.front();
        return true;
    }

    std::vector<object_det> get_ball_dets() const
    {
        std::lock_guard<std::mutex> lk(det_mtx_);
        return ball_dets_;
    }

    std::vector<object_det> get_post_dets() const
    {
        std::lock_guard<std::mutex> lk(det_mtx_);
        return post_dets_;
    }
#endif

public:
    std::atomic_bool localization_;
    std::atomic_bool can_see_post_;

private:
    Eigen::Vector2d odometry(const Eigen::Vector2i &pos, const robot_math::TransformMatrix &camera_matrix);
    Eigen::Vector2d camera2self(const Eigen::Vector2d &pos, double head_yaw);

#ifndef MAXWELL_ROS2
    // legacy-only helpers
    Imu::imu_data imu_data_vison;
    float dir = 0.0f;
#endif

private:
    void run();
    void send_image(const cv::Mat &src);

#ifndef MAXWELL_ROS2
    std::queue<Imu::imu_data> imu_datas_;
    std::queue<std::vector<double>> foot_degs_, head_degs_;
    std::queue<int> spfs_;

    float head_yaw_{0.0f}, head_pitch_{0.0f};
    Eigen::Vector2d odometry_offset_{0.0, 0.0};
#endif

    float d_w_h_{0.0f};

    bool detect_filed_{false};
    bool use_mv_{false};
    int p_count_{0};
    int w_{0}, h_{0};

    // legacy raw camera format sizes; in ROS2 mode camera_src_ stores BGR(w_ x h_ x 3)
    int camera_w_{0}, camera_h_{0};
    size_t camera_size_{0};

    std::map<std::string, camera_info> camera_infos_;
    camera_param params_;
#ifndef MAXWELL_ROS2
    robot_math::TransformMatrix camera_matrix_;
#endif

    std::vector<object_det> ball_dets_, post_dets_;
    int ball_id_{1}, post_id_{0};
    float ball_prob_{0.0f}, post_prob_{0.0f};
    int min_ball_w_{0}, min_ball_h_{0};
    int min_post_w_{0}, min_post_h_{0};

    // odometry compensation
    Eigen::Vector2d odometry_offset_;

    int cant_see_ball_count_{0};
    int can_see_post_count_{0};

    bool is_busy_{false};
    image_send_type img_sd_type_{IMAGE_SEND_RESULT};

    network net_;

    unsigned char *dev_src_{nullptr};
    unsigned char *dev_bgr_{nullptr};
    unsigned char *dev_ori_{nullptr};
    unsigned char *dev_sized_{nullptr};
    unsigned char *dev_undis_{nullptr};
    unsigned char *dev_yuyv_{nullptr};
    unsigned char *camera_src_{nullptr};
    float *dev_rgbfp_{nullptr};

    int src_size_{0};
    int bgr_size_{0};
    int ori_size_{0};
    int yuyv_size_{0};
    int sized_size_{0};
    int rgbf_size_{0};

    cv::Mat camK;
    cv::Mat newCamK;
    cv::Mat invCamK;
    cv::Mat D;
    cv::Mat R;

    cv::Mat mapx;
    cv::Mat mapy;

    float *pCamKData{nullptr};
    float *pInvNewCamKData{nullptr};
    float *pDistortData{nullptr};
    float *pMapxData{nullptr};
    float *pMapyData{nullptr};

    // mutexes
    mutable std::mutex frame_mtx_;
    mutable std::mutex det_mtx_;
#ifndef MAXWELL_ROS2
    mutable std::mutex imu_mtx_;
#endif
};

#define VISION Vision::instance()

#endif  // __VISION_HPP
