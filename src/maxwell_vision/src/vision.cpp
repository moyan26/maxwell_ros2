#include "vision.hpp"

#if __has_include("parser/parser.hpp")
#  include "parser/parser.hpp"
#elif __has_include("parser.hpp")
#  include "parser.hpp"
#else
#  error "parser header not found (need parser/parser.hpp or parser.hpp)"
#endif

#if defined(MAXWELL_VISION_RUNTIME) && MAXWELL_VISION_RUNTIME && defined(MAXWELL_ENABLE_CUDA) && MAXWELL_ENABLE_CUDA
#  include "darknet/parser.h"
#  include <cuda_runtime.h>
#  include "imageproc/imageproc.hpp"
#endif

#ifdef MAXWELL_WITH_SERVER
#  include "server/server.hpp"
#endif

#include <algorithm>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#ifndef MAXWELL_ROS2
#  include "core/worldmodel.hpp"
#endif

using namespace std;
using namespace cv;
using namespace robot_math;
using namespace Eigen;
using namespace robot;
#if defined(MAXWELL_VISION_RUNTIME) && MAXWELL_VISION_RUNTIME && defined(MAXWELL_ENABLE_CUDA) && MAXWELL_ENABLE_CUDA
using namespace imgproc;
#endif

Vision::Vision() : Timer(CONF->get_config_value<int>("vision_period"))
{
    p_count_ = 0;
    cant_see_ball_count_ = 0;
    can_see_post_count_ = 0;
    is_busy_ = false;
    w_ = CONF->get_config_value<int>("image.width");
    h_ = CONF->get_config_value<int>("image.height");
    img_sd_type_ = IMAGE_SEND_RESULT;
    camera_src_ = nullptr;

    parser::parse(CONF->get_config_value<string>(CONF->player() + ".camera_info_file"), camera_infos_);
    parser::parse(CONF->get_config_value<string>(CONF->player() + ".camera_params_file"), params_);
    LOG(LOG_INFO) << setw(12) << "algorithm:" << setw(18) << "[vision]"
                  << " started!" << endll;
    ball_id_ = 1;
    post_id_ = 0;
    ball_prob_ = CONF->get_config_value<float>("detection.ball");
    post_prob_ = CONF->get_config_value<float>("detection.post");
    min_ball_w_ = CONF->get_config_value<int>("detection.ball_w");
    min_ball_h_ = CONF->get_config_value<int>("detection.ball_h");
    min_post_w_ = CONF->get_config_value<int>("detection.post_w");
    min_post_h_ = CONF->get_config_value<int>("detection.post_h");
    odometry_offset_ = CONF->get_config_vector<double, 2>(CONF->player() + ".odometry_offset");
    ori_size_ = 0;
    yuyv_size_ = 0;
    rgbf_size_ = 0;
    sized_size_ = 0;
    camera_size_ = 0;
    src_size_ = 0;
    bgr_size_ = 0;
    camera_w_ = 0;
    camera_h_ = 0;
    use_mv_ = false;

    dev_ori_ = nullptr;
    dev_undis_ = nullptr;
    dev_yuyv_ = nullptr;
    dev_rgbfp_ = nullptr;
    dev_sized_ = nullptr;

    dev_src_ = nullptr;
    dev_bgr_ = nullptr;

    pCamKData = nullptr;
    pInvNewCamKData = nullptr;
    pDistortData = nullptr;
    pMapxData = nullptr;
    pMapyData = nullptr;

#ifndef MAXWELL_ROS2
    init_data();
#endif
}

Vision::~Vision()
{
}

void Vision::set_camera_info(const camera_info &para)
{
    if (camera_infos_.find(para.name) != camera_infos_.end())
    {
        camera_infos_[para.name].value = para.value;
    }
}

Vector2d Vision::odometry(const Vector2i &pos, const robot_math::TransformMatrix &camera_matrix)
{
    double Xw = 0.0, Yw = 0.0;
    double theta = deg2rad(camera_matrix.x_rotate() - 90.0);
    double roll = deg2rad(camera_matrix.y_rotate());

    double OC = camera_matrix.p().z();
    double Zc = OC * cos(theta);
    double Yc = OC * sin(theta);
    double O_C = pow(pow(Zc, 2) + pow(Yc, 2), 0.5);
    double gama = atan((params_.cy - pos.y()) / params_.fy);
    double O_C_P = M_PI_2 - theta + gama;
    if (!float_equals(O_C_P, M_PI_2))
    {
        Yw = O_C * tan(O_C_P);
        Xw = (pos.x() - params_.cx) * O_C * cos(gama) / (cos(O_C_P) * params_.fx);
    }
    Yw -= odometry_offset_.y();
    Xw -= odometry_offset_.x();
    return Vector2d(Xw, Yw);
}

Vector2d Vision::camera2self(const Vector2d &pos, double head_yaw)
{
    return rotation_mat_2d(head_yaw + 90.0) * pos;
}

#ifdef MAXWELL_ROS2
// ROS2 版本：由 vision_node 在订阅回调里把图像喂给 Vision。
// 约定：输入为 BGR8 (CV_8UC3)。若尺寸不是 (w_, h_) 则在 CPU 侧 resize 到 (w_, h_)。
void Vision::set_bgr_frame(const cv::Mat &bgr)
{
    if (!is_alive_) return;
    if (bgr.empty()) return;

    // start() 可能尚未被调用，此时 ori_size_ 还未初始化
    if (ori_size_ == 0)
        ori_size_ = w_ * h_ * 3;

    cv::Mat bgr8;
    if (bgr.type() == CV_8UC3)
    {
        bgr8 = bgr;
    }
    else if (bgr.type() == CV_8UC1)
    {
        cv::cvtColor(bgr, bgr8, cv::COLOR_GRAY2BGR);
    }
    else if (bgr.type() == CV_8UC4)
    {
        cv::cvtColor(bgr, bgr8, cv::COLOR_BGRA2BGR);
    }
    else
    {
        // 尝试兜底转换
        cv::Mat tmp;
        bgr.convertTo(tmp, CV_8U);
        if (tmp.channels() == 1)
            cv::cvtColor(tmp, bgr8, cv::COLOR_GRAY2BGR);
        else if (tmp.channels() == 4)
            cv::cvtColor(tmp, bgr8, cv::COLOR_BGRA2BGR);
        else
            bgr8 = tmp;
    }

    cv::Mat resized;
    if (bgr8.cols != w_ || bgr8.rows != h_)
        cv::resize(bgr8, resized, cv::Size(w_, h_), 0, 0, cv::INTER_LINEAR);
    else
        resized = bgr8;

    if (!resized.isContinuous())
        resized = resized.clone();

    frame_mtx_.lock();
    // 复用 camera_src_ 作为“最新一帧”的 Host 缓冲，存放 BGR (w_ x h_)
    if (camera_src_ == nullptr || camera_size_ != (size_t)ori_size_)
    {
        if (camera_src_ != nullptr) { free(camera_src_); camera_src_ = nullptr; }
        camera_size_ = ori_size_;
        src_size_ = ori_size_;
        camera_src_ = (unsigned char *)malloc(camera_size_);
    }
    memcpy(camera_src_, resized.data, ori_size_);
    frame_mtx_.unlock();
}
#endif

void Vision::get_point_dis(int x, int y)
{
#ifdef MAXWELL_WITH_SERVER
#ifndef MAXWELL_ROS2

    TransformMatrix camera_matrix;
    frame_mtx_.lock();
    camera_matrix = camera_matrix_;
    frame_mtx_.unlock();
    Vector2d dis(0.0, 0.0);
    dis = odometry(Vector2i(x, y), camera_matrix);
    float xx = static_cast<float>(dis[0]), yy = static_cast<float>(dis[1]);
    tcp_command cmd;
    cmd.type = REMOTE_DATA;
    cmd.size = 2 * enum_size + 2 * float_size;
    remote_data_type t1 = IMAGE_SEND_TYPE;
    image_send_type t2 = IMAGE_SEND_DIS;
    cmd.data.clear();
    cmd.data.append((char *)&t1, enum_size);
    cmd.data.append((char *)&t2, enum_size);
    cmd.data.append((char *)&xx, float_size);
    cmd.data.append((char *)&yy, float_size);
    SERVER->write(cmd);

#else
    (void)x; (void)y;
#endif
#else
    (void)x; (void)y;
#endif
}

void Vision::run()
{
#if !(defined(MAXWELL_VISION_RUNTIME) && MAXWELL_VISION_RUNTIME)
    // compile-only build (no third-party vision libs)
    return;
#elif !(defined(MAXWELL_ENABLE_CUDA) && MAXWELL_ENABLE_CUDA)
    // runtime build but CUDA disabled
    return;
#else

    if (is_alive_)
    {
        if (camera_src_ == nullptr)
            return;
        if (is_busy_)
            return;
        is_busy_ = true;
        p_count_++;
        cudaError_t err;

#ifndef MAXWELL_ROS2
        robot_math::TransformMatrix camera_matrix;
        float head_yaw, head_pitch;

        frame_mtx_.lock();
        err = cudaMemcpy(dev_src_, camera_src_, src_size_, cudaMemcpyHostToDevice);
        check_error(err);
        camera_matrix = camera_matrix_;
        head_yaw = head_yaw_;
        head_pitch = head_pitch_;
        frame_mtx_.unlock();

        if (use_mv_)
        {
            cudaBayer2BGR(dev_src_, dev_bgr_, camera_w_, camera_h_, camera_infos_["saturation"].value,
                          camera_infos_["exposure"].value, camera_infos_["brightness"].value,
                          camera_infos_["contrast"].value);
        }
        else
        {
            cudaYUYV2BGR(dev_src_, dev_bgr_, camera_w_, camera_h_);
        }

        cudaResizePacked(dev_bgr_, camera_w_, camera_h_, dev_ori_, w_, h_);

        if (CONF->get_config_value<bool>("is_undistort"))
        {
            cudaUndistored(dev_ori_, dev_undis_, pCamKData, pDistortData, pInvNewCamKData, pMapxData, pMapyData, w_, h_, 3);
        }
        else
        {
            cudaMemcpy(dev_undis_, dev_ori_, ori_size_, cudaMemcpyDeviceToDevice);
        }

#else
        // ROS2：camera_src_ 存放的是 BGR (w_ x h_ x 3)，由 set_bgr_frame() 更新
        frame_mtx_.lock();
        err = cudaMemcpy(dev_ori_, camera_src_, ori_size_, cudaMemcpyHostToDevice);
        frame_mtx_.unlock();
        check_error(err);

        if (CONF->get_config_value<bool>("is_undistort"))
        {
            cudaUndistored(dev_ori_, dev_undis_, pCamKData, pDistortData, pInvNewCamKData, pMapxData, pMapyData, w_, h_, 3);
        }
        else
        {
            cudaMemcpy(dev_undis_, dev_ori_, ori_size_, cudaMemcpyDeviceToDevice);
        }
#endif

        cudaResizePacked(dev_undis_, w_, h_, dev_sized_, net_.w, net_.h);
        cudaBGR2RGBfp(dev_sized_, dev_rgbfp_, net_.w, net_.h); // 转为浮点型供神经网络使用

        int nboxes = 0;
        network_predict(net_, dev_rgbfp_, 0);
        layer l = net_.layers[net_.n - 1];
        detection *dets = get_network_boxes(&net_, w_, h_, 0.5, 0.5, 0, 1, &nboxes, 0);
        do_nms_sort(dets, nboxes, l.classes, 0.45);

        vector<object_det> dets_ball;
        vector<object_det> dets_post;

        for (int i = 0; i < nboxes; i++)
        {
            int class_id = -1;
            float max_prob = 0;
            for (int j = 0; j < l.classes; j++)
            {
                if (dets[i].prob[j] > max_prob)
                {
                    max_prob = dets[i].prob[j];
                    class_id = j;
                }
            }
            if (class_id < 0)
                continue;

            box b = dets[i].bbox;
            int left = (b.x - b.w / 2.) * w_;
            int right = (b.x + b.w / 2.) * w_;
            int top = (b.y - b.h / 2.) * h_;
            int bot = (b.y + b.h / 2.) * h_;

            if (left < 0)
                left = 0;
            if (right > w_ - 1)
                right = w_ - 1;
            if (top < 0)
                top = 0;
            if (bot > h_ - 1)
                bot = h_ - 1;

            int obj_w = right - left;
            int obj_h = bot - top;

            if (class_id == ball_id_ && max_prob > ball_prob_ && obj_w > min_ball_w_ && obj_h > min_ball_h_)
            {
                dets_ball.emplace_back(class_id, max_prob, left, top, obj_w, obj_h);
            }
            else if (class_id == post_id_ && max_prob > post_prob_ && obj_w > min_post_w_ && obj_h > min_post_h_)
            {
                dets_post.emplace_back(class_id, max_prob, left, top, obj_w, obj_h);
            }
        }

        free_detections(dets, nboxes);

        // Sort by prob descending
        sort(dets_ball.begin(), dets_ball.end());
        reverse(dets_ball.begin(), dets_ball.end());
        sort(dets_post.begin(), dets_post.end());
        reverse(dets_post.begin(), dets_post.end());

        // Update cached detections (thread-safe)
        {
            lock_guard<mutex> lk(det_mtx_);
            ball_dets_.clear();
            post_dets_.clear();
            for (auto &d : dets_ball) ball_dets_.push_back(d);
            for (auto &d : dets_post) post_dets_.push_back(d);
        }

#ifdef MAXWELL_WITH_SERVER
        // Send image if required (debug)
        if (img_sd_type_ == IMAGE_SEND_ORIGIN || img_sd_type_ == IMAGE_SEND_RESULT)
        {
            cv::Mat bgr(h_, w_, CV_8UC3);
            if (bgr.isContinuous())
            {
                if (img_sd_type_ == IMAGE_SEND_ORIGIN)
                {
                    err = cudaMemcpy(bgr.data, dev_ori_, ori_size_, cudaMemcpyDeviceToHost);
                    check_error(err);
                }
                else
                {
                    err = cudaMemcpy(bgr.data, dev_undis_, ori_size_, cudaMemcpyDeviceToHost);
                    check_error(err);

                    // draw dets
                    for (auto &d : dets_ball)
                    {
                        rectangle(bgr, Rect(d.x, d.y, d.w, d.h), Scalar(0, 255, 0), 2);
                    }
                    for (auto &d : dets_post)
                    {
                        rectangle(bgr, Rect(d.x, d.y, d.w, d.h), Scalar(0, 0, 255), 2);
                    }
                }
            }
            send_image(bgr);
        }
#endif

        is_busy_ = false;
    }

#endif  // MAXWELL_VISION_RUNTIME && MAXWELL_ENABLE_CUDA
}

void Vision::send_image(const cv::Mat &src)
{
#ifdef MAXWELL_WITH_SERVER

    cv::Mat bgr;
    src.copyTo(bgr);
    std::vector<unsigned char> jpgbuf;
    cv::imencode(".jpg", bgr, jpgbuf);
    bgr.release();
    tcp_command cmd;
    cmd.type = IMG_DATA;
    cmd.size = jpgbuf.size();
    cmd.data.assign((char *)&(jpgbuf[0]), jpgbuf.size());
    SERVER->write(cmd);

#else
    (void)src;
#endif
}

#ifndef MAXWELL_ROS2
void Vision::updata(const pub_ptr &pub, const int &type)
{

    if (!is_alive_)
    {
        return;
    }
    if (type == Sensor::SENSOR_CAMERA)
    {
        shared_ptr<Camera> sptr = dynamic_pointer_cast<Camera>(pub);
        if (camera_src_ == nullptr)
        {
            camera_w_ = sptr->camera_w();
            camera_h_ = sptr->camera_h();
            camera_size_ = sptr->camera_size();
            use_mv_ = sptr->use_mv();
            src_size_ = camera_size_;
            bgr_size_ = camera_w_ * camera_h_ * 3;
            camera_src_ = (unsigned char *)malloc(camera_size_);
            cudaError_t err;
            err = cudaMalloc((void **)&dev_src_, src_size_);
            check_error(err);
            err = cudaMalloc((void **)&dev_bgr_, bgr_size_);
            check_error(err);
        }
        frame_mtx_.lock();
        memcpy(camera_src_, sptr->buffer(), src_size_);
        if (OPTS->use_robot())
        {
            camera_matrix_ = WM->robot().t_head() * WM->robot().camera_matrix();
            head_yaw_ = WM->robot().head_yaw();
            head_pitch_ = WM->robot().head_pitch();
        }
        frame_mtx_.unlock();
    }
    else if (type == Sensor::SENSOR_GAMECTRL)
    {
        shared_ptr<GameCtrl> sptr = dynamic_pointer_cast<GameCtrl>(pub);
        if (sptr->state() == GameCtrl::STATE_INITIAL || sptr->state() == GameCtrl::STATE_FINISH)
        {
            cant_see_ball_count_ = 0;
            can_see_post_count_ = 0;
        }
    }
}
#endif

bool Vision::start()
{
#if !(defined(MAXWELL_VISION_RUNTIME) && MAXWELL_VISION_RUNTIME)
    LOG(LOG_WARN) << setw(12) << "algorithm:" << setw(18) << "[vision]" << " built with MAXWELL_VISION_RUNTIME=OFF (compile-only)." << endll;
    is_alive_ = false;
    return false;
#elif !(defined(MAXWELL_ENABLE_CUDA) && MAXWELL_ENABLE_CUDA)
    LOG(LOG_WARN) << setw(12) << "algorithm:" << setw(18) << "[vision]" << " built without CUDA (MAXWELL_ENABLE_CUDA=OFF). start() disabled." << endll;
    is_alive_ = false;
    return false;
#else

    net_.gpu_index = 0;
    net_ = parse_network_cfg_custom((char *)CONF->get_config_value<string>("net_cfg_file").c_str(), 1);
    load_weights(&net_, (char *)CONF->get_config_value<string>("net_weights_file").c_str());
    set_batch_network(&net_, 1);
    fuse_conv_batchnorm(net_);
    calculate_binary_weights(net_);
    srand(2222222);

    ori_size_ = w_ * h_ * 3;
    yuyv_size_ = w_ * h_ * 2;
    sized_size_ = net_.w * net_.h * 3;
    rgbf_size_ = w_ * h_ * 3 * sizeof(float);

    cudaError_t err;
    err = cudaMalloc((void **)&dev_ori_, ori_size_);
    check_error(err);
    err = cudaMalloc((void **)&dev_undis_, ori_size_);
    check_error(err);
    err = cudaMalloc((void **)&dev_yuyv_, yuyv_size_);
    check_error(err);
    err = cudaMalloc((void **)&dev_sized_, sized_size_);
    check_error(err);
    err = cudaMalloc((void **)&dev_rgbfp_, rgbf_size_);
    check_error(err);

    err = cudaMalloc(&pCamKData, 9 * sizeof(float));
    check_error(err);
    err = cudaMalloc(&pInvNewCamKData, 9 * sizeof(float));
    check_error(err);
    err = cudaMalloc(&pDistortData, 4 * sizeof(float));
    check_error(err);
    err = cudaMalloc(&pMapxData, h_ * w_ * sizeof(float));
    check_error(err);
    err = cudaMalloc(&pMapyData, h_ * w_ * sizeof(float));
    check_error(err);

    // Prepare undistort maps on CPU then upload
    cv::Mat CamK = (cv::Mat_<float>(3, 3) << params_.fx, 0, params_.cx, 0, params_.fy, params_.cy, 0, 0, 1);
    cv::Mat Dist = (cv::Mat_<float>(1, 4) << params_.k1, params_.k2, params_.p1, params_.p2);
    cv::Mat NewCamK, InvNewCamK, mapx, mapy;
    cv::Rect roi;
    NewCamK = cv::getOptimalNewCameraMatrix(CamK, Dist, cv::Size(w_, h_), 0, cv::Size(w_, h_), &roi);
    InvNewCamK = NewCamK.inv();
    cv::initUndistortRectifyMap(CamK, Dist, cv::Mat(), NewCamK, cv::Size(w_, h_), CV_32FC1, mapx, mapy);

    err = cudaMemcpy(pCamKData, CamK.ptr<float>(), 9 * sizeof(float), cudaMemcpyHostToDevice);
    check_error(err);
    err = cudaMemcpy(pInvNewCamKData, InvNewCamK.ptr<float>(), 9 * sizeof(float), cudaMemcpyHostToDevice);
    check_error(err);
    err = cudaMemcpy(pDistortData, Dist.ptr<float>(), 4 * sizeof(float), cudaMemcpyHostToDevice);
    check_error(err);
    err = cudaMemcpy(pMapxData, mapx.data, h_ * w_ * sizeof(float), cudaMemcpyHostToDevice);
    check_error(err);
    err = cudaMemcpy(pMapyData, mapy.data, h_ * w_ * sizeof(float), cudaMemcpyHostToDevice);
    check_error(err);

    is_alive_ = true;
#ifndef MAXWELL_ROS2
    start_timer();
#endif
    return true;

#endif  // MAXWELL_VISION_RUNTIME && MAXWELL_ENABLE_CUDA
}

void Vision::stop()
{
#if !(defined(MAXWELL_VISION_RUNTIME) && MAXWELL_VISION_RUNTIME)
    // compile-only build: nothing to tear down
    if (camera_src_ != nullptr) { free(camera_src_); camera_src_ = nullptr; }
    is_alive_ = false;
    return;
#elif !(defined(MAXWELL_ENABLE_CUDA) && MAXWELL_ENABLE_CUDA)
    // runtime build but CUDA disabled
    if (camera_src_ != nullptr) { free(camera_src_); camera_src_ = nullptr; }
    is_alive_ = false;
    return;
#else

    if (is_alive_)
    {
#ifndef MAXWELL_ROS2
        delete_timer();
#endif
        free_network(net_);
        if (camera_src_ != nullptr) { free(camera_src_); camera_src_ = nullptr; }
        cudaFree(dev_ori_);
        cudaFree(dev_undis_);
        cudaFree(dev_yuyv_);
        if (dev_src_ != nullptr) { cudaFree(dev_src_); dev_src_ = nullptr; }
        if (dev_bgr_ != nullptr) { cudaFree(dev_bgr_); dev_bgr_ = nullptr; }
        cudaFree(dev_rgbfp_);
        cudaFree(dev_sized_);

        cudaFree(pCamKData);
        cudaFree(pInvNewCamKData);
        cudaFree(pDistortData);
        cudaFree(pMapxData);
        cudaFree(pMapyData);
    }
    is_alive_ = false;

#endif  // MAXWELL_VISION_RUNTIME && MAXWELL_ENABLE_CUDA
}

