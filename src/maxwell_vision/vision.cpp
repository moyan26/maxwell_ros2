#include "vision.hpp"
#include "parser/parser.hpp"
#include "darknet/parser.h"
#include <cuda_runtime.h>
#ifdef MAXWELL_WITH_SERVER
#include "server/server.hpp"
#endif
#include <algorithm>
#ifndef MAXWELL_ROS2
#include "core/worldmodel.hpp"
#endif
#include "imageproc/imageproc.hpp"
#include <fstream>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;
using namespace robot_math;
using namespace Eigen;
using namespace robot;
using namespace imgproc;

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
    odometry_offset_.x() = deg2rad(odometry_offset_.x());
    odometry_offset_.y() = deg2rad(odometry_offset_.y());

    camK = cv::Mat::eye(3, 3, CV_32F);
    newCamK = cv::Mat::eye(3, 3, CV_32F);
    invCamK = cv::Mat::eye(3, 3, CV_32F);
    float rotation_mat[9] = {0.999956, 0.00837928, 0.00423897,
                             -0.00839916, 0.999954, 0.00469485,
                             -0.00419943, -0.00473025, 0.99998};
    R = cv::Mat(3, 3, CV_32F, rotation_mat);

    D = cv::Mat::zeros(4, 1, CV_32F);
    D.at<float>(0, 0) = params_.k1;
    D.at<float>(1, 0) = params_.k2;
    D.at<float>(2, 0) = params_.p1;
    D.at<float>(3, 0) = params_.p2;

    mapx = cv::Mat(h_, w_, CV_32F);
    mapy = cv::Mat(h_, w_, CV_32F);

    camK.at<float>(0, 2) = params_.cx;
    camK.at<float>(1, 2) = params_.cy;
    camK.at<float>(0, 0) = params_.fx;
    camK.at<float>(1, 1) = params_.fy;
    camK.at<float>(2, 2) = 1;
    newCamK = camK.clone();
    invCamK = (newCamK * R.t()).inv(cv::DECOMP_LU);

    detect_filed_ = false;
    d_w_h_ = 0.3;
    localization_ = false;
    can_see_post_ = false;
}

Vision::~Vision()
{
    LOG(LOG_INFO) << setw(12) << "algorithm:" << setw(18) << "[vision]"
                  << " ended!" << endll;
}

void Vision::set_camera_info(const camera_info &para)
{
    for (auto &item : camera_infos_)
    {
        if (item.second.id == para.id)
        {
            item.second.value = para.value;
            break;
        }
    }
}

Vector2d Vision::odometry(const Vector2i &pos, const robot_math::TransformMatrix &camera_matrix)
{
    float Xw = 0.0, Yw = 0.0;
    float OC = camera_matrix.p().z();

    float roll = -static_cast<float>(camera_matrix.x_rotate());
    float theta = static_cast<float>(camera_matrix.y_rotate());

    // LOG(LOG_INFO)<<OC<<' '<<rad2deg(roll)<<' '<<rad2deg(theta)<<endll;
    roll = roll + odometry_offset_.x();
    theta = theta + odometry_offset_.y();

    Vector2i Pos(pos.x() - params_.cx, params_.cy - pos.y());
    Vector2i calCenterPos(Pos.x() / cos(roll), Pos.y() + Pos.x() * tan(roll));
    Vector2i calPos(calCenterPos.x() + params_.cx, params_.cy - calCenterPos.y());
    double gama = atan((params_.cy - calPos.y()) / params_.fy);
    double O_C_P = M_PI_2 - theta + gama;
    if (!float_equals(O_C_P, M_PI_2))
    {
        Yw = OC * tan(O_C_P);
        Xw = (calPos.x() - params_.cx) * OC * cos(gama) / (cos(O_C_P) * params_.fx);
    }
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
        if (camera_src_ != nullptr)
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
        if (OPTS->use_robot())
        {
            camera_matrix = camera_matrix_;
            head_yaw = head_yaw_;
            head_pitch = head_pitch_;
        }
        frame_mtx_.unlock();

        // double t1 = clock();
        if (use_mv_)
        {
            cudaBayer2BGR(dev_src_, dev_bgr_, camera_w_, camera_h_, camera_infos_["saturation"].value,
                          camera_infos_["red_gain"].value, camera_infos_["green_gain"].value, camera_infos_["blue_gain"].value);
        }
        else
        {
            cudaYUYV2BGR(dev_src_, dev_bgr_, camera_w_, camera_h_);
        }
        cudaResizePacked(dev_bgr_, camera_w_, camera_h_, dev_ori_, w_, h_);
        if (use_mv_)
            cudaUndistored(dev_ori_, dev_undis_, pCamKData, pDistortData, pInvNewCamKData, pMapxData, pMapyData, w_, h_, 3);
        else
            cudaMemcpy(dev_undis_, dev_ori_, ori_size_, cudaMemcpyDeviceToDevice);
#else
        // ROS2：camera_src_ 存放的是 BGR (w_ x h_ x 3)，由 set_bgr_frame() 更新
        frame_mtx_.lock();
        err = cudaMemcpy(dev_ori_, camera_src_, ori_size_, cudaMemcpyHostToDevice);
        frame_mtx_.unlock();
        check_error(err);

        if (use_mv_)
            cudaUndistored(dev_ori_, dev_undis_, pCamKData, pDistortData, pInvNewCamKData, pMapxData, pMapyData, w_, h_, 3);
        else
            cudaMemcpy(dev_undis_, dev_ori_, ori_size_, cudaMemcpyDeviceToDevice);
#endif
        cudaResizePacked(dev_undis_, w_, h_, dev_sized_, net_.w, net_.h);
        cudaBGR2RGBfp(dev_sized_, dev_rgbfp_, net_.w, net_.h); // 转为浮点型供神经网络使用
        /*
        const int *fieldBorders;
        if(detect_filed_)
        {
            detector_->process(dev_yuyv_);
            fieldBorders = detector_->getBorder();
        }
        */
        layer l = net_.layers[net_.n - 1];
        network_predict(net_, dev_rgbfp_, 0);
        int nboxes = 0;
        float nms = .45;
        detection *dets = get_network_boxes(&net_, w_, h_, 0.5, 0.5, 0, 1, &nboxes, 0);

        if (nms)
            do_nms_sort(dets, nboxes, l.classes, nms);
        {
            std::lock_guard<std::mutex> lk(det_mtx_);
            ball_dets_.clear();
            post_dets_.clear();
            for (int i = 0; i < nboxes; i++)
            {
            if (dets[i].prob[ball_id_] > dets[i].prob[post_id_])
            {
            if (dets[i].prob[ball_id_] >= ball_prob_)
            {
            int bx = (dets[i].bbox.x - dets[i].bbox.w / 2.0) * w_;
            int by = (dets[i].bbox.y - dets[i].bbox.h / 2.0) * h_;
            int bw = dets[i].bbox.w * w_, bh = dets[i].bbox.h * h_ + 1;
            float w_h = (float)bw / (float)bh;
            if (bw >= min_ball_w_ && bh >= min_ball_h_ && fabs(w_h - 1.0) < d_w_h_)
            {
            ball_dets_.push_back(object_det(ball_id_, dets[i].prob[ball_id_], bx, by, bw, bh));
            }
            }
            }
            else
            {
            if (dets[i].prob[post_id_] >= post_prob_)
            {
            int px = (dets[i].bbox.x - dets[i].bbox.w / 2.0) * w_;
            int py = (dets[i].bbox.y - dets[i].bbox.h / 2.0) * h_;
            int pw = dets[i].bbox.w * w_, ph = dets[i].bbox.h * h_;
            if (pw >= min_post_w_ && ph >= min_post_h_)
            {
            post_dets_.push_back(object_det(post_id_, dets[i].prob[post_id_], px, py, pw, ph));
            }
            }
            }
            }
            std::sort(ball_dets_.rbegin(), ball_dets_.rend());
            std::sort(post_dets_.rbegin(), post_dets_.rend());
        }
        free_detections(dets, nboxes);
        // double t2 = clock();
        // LOG(LOG_INFO)<<(t2-t1)/CLOCKS_PER_SEC<<endll;

#ifndef MAXWELL_ROS2
        if (OPTS->use_robot())
        {
            self_block p = WM->self();
            if (!ball_dets_.empty())
            {
                Vector2i ball_pix(ball_dets_[0].x + ball_dets_[0].w / 2, ball_dets_[0].y + ball_dets_[0].h);
                // ball_pix = undistored(ball_pix);
                #ifndef MAXWELL_ROS2
                    Vector2d odo_res = odometry(ball_pix, camera_matrix);
#endif
                // LOG(LOG_INFO)<<odo_res.norm()<<endll;
                Vector2d ball_pos = camera2self(odo_res, head_yaw);
                cant_see_ball_count_ = 0;
                Vector2d temp_ball = p.global + rotation_mat_2d(-p.dir) * ball_pos;
                float alpha = (ball_pix.x() - params_.cx) / (float)w_;
                float beta = (ball_pix.y() - params_.cy) / (float)h_;
                WM->set_ball_pos(temp_ball, ball_pos, ball_pix, alpha, beta, true);
            }
            else
            {
                cant_see_ball_count_++;
                if (cant_see_ball_count_ * period_ms_ > 500)
                    WM->set_ball_pos(Vector2d(0, 0), Vector2d(0, 0), Vector2i(0, 0), 0, 0, false);
            }

            if (localization_)
            {
                int post_num = 0;
                vector<goal_post> posts_;
                for (auto &post : post_dets_)
                {
                    dir = imu_data_vison.yaw;
                    post_num++;
                    if (post_num > 2)
                        break;
                    goal_post temp;
                    Vector2i post_pix(post.x + post.w / 2, post.y + post.h * 0.8);
                    #ifndef MAXWELL_ROS2
                    Vector2d odo_res = odometry(post_pix, camera_matrix);
#endif
                    temp._distance = odo_res.norm() * 100;
                    LOG(LOG_INFO) << "_distance:" << temp._distance << endl;
                    if (temp._distance > 600)
                        continue;
                    Vector2d post_pos = camera2self(odo_res, head_yaw);
                    temp._theta = azimuth_deg(post_pos);
                    posts_.push_back(temp);
                    int id_yaw = ROBOT->get_joint("jhead1")->jid_;
                    int id_pitch = ROBOT->get_joint("jhead2")->jid_;
                    if (posts_.size() == 2)
                    {
                        if (post_dets_[0].x < post_dets_[1].x)
                        {
                            posts_[0].type = goal_post::SENSORMODEL_POST_L;
                            posts_[1].type = goal_post::SENSORMODEL_POST_R;
                        }
                        else
                        {
                            posts_[0].type = goal_post::SENSORMODEL_POST_R;
                            posts_[1].type = goal_post::SENSORMODEL_POST_L;
                        }
                        break;
                    }
                    // if (posts_.size() == 1) // imu判断
                    // {
                    //     if (dir < 0 && id_yaw > 0)
                    //     {
                    //         posts_[0].type = goal_post::SENSORMODEL_POST_R;
                    //     }
                    //     else if (dir > 0 && id_yaw < 0)
                    //     {
                    //         posts_[0].type = goal_post::SENSORMODEL_POST_L;
                    //     }
                    //     LOG(LOG_INFO) << "dir:" << dir << endl;
                    //     LOG(LOG_INFO) << "POST_TYPE:" << posts_[0].type << endl;
                    // }
                }

                LOG(LOG_INFO) << "POST_SIZE:" << post_num << endl;
                if (posts_.size() > 0)
                {
                    can_see_post_count_++;
                    SL->update(player_info(p.global.x(), p.global.y(), p.dir), posts_);
                    if (can_see_post_count_ > 10)
                    {
                        can_see_post_ = true;
                        localization_ = false;
                    }
                }
            }
            else
            {
                can_see_post_ = false;
                can_see_post_count_ = 0;
            }
        }
#endif


#ifndef MAXWELL_ROS2
        if (OPTS->use_debug())
        {
            Mat bgr(h_, w_, CV_8UC3);

            if (OPTS->image_record())
            {
                err = cudaMemcpy(bgr.data, dev_undis_, ori_size_, cudaMemcpyDeviceToHost);
                check_error(err);
                send_image(bgr);
                is_busy_ = false;
                return;
            }
            err = cudaMemcpy(bgr.data, dev_undis_, ori_size_, cudaMemcpyDeviceToHost);
            check_error(err);
            if (img_sd_type_ == IMAGE_SEND_ORIGIN)
            {
                send_image(bgr);
            }
            else if (img_sd_type_ == IMAGE_SEND_RESULT)
            {
                if (detect_filed_)
                {
                    /*
                    for(int j=0; j<h_; j++)
                    {
                        for(int i=0; i<w_; i++)
                        {
                            if(detector_->isGreen(i, j))
                                bgr.at<Vec3b>(j, i) = Vec3b(0, 255, 0);
                        }
                    }*/
                    /*
                    for(int i=0;i<w_;i++)
                        bgr.at<Vec3b>(fieldBorders[i], i) = Vec3b(0, 255, 255);
                        */
                }
                if (!ball_dets_.empty())
                {
                    Vector2i ball_pix(ball_dets_[0].x + ball_dets_[0].w / 2, ball_dets_[0].y + ball_dets_[0].h * 0.95);
                    #ifndef MAXWELL_ROS2
                    Vector2d odo_res = odometry(ball_pix, camera_matrix);
#endif
                    rectangle(bgr, Point(ball_dets_[0].x, ball_dets_[0].y), Point(ball_dets_[0].x + ball_dets_[0].w, ball_dets_[0].y + ball_dets_[0].h), Scalar(255, 0, 0), 2);
                    // putText(bgr, to_string(odo_res.norm()).substr(0,4), Point(ball_dets_[0].x, ball_dets_[0].y+ball_dets_[0].h),
                    //     FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, 8);
                }
                int i = 0;
                for (auto &dd : post_dets_)
                {
                    if (i >= 2)
                        break;
                    Vector2i post_pix(dd.x + dd.w / 2, dd.y + dd.h * 0.95);
                    #ifndef MAXWELL_ROS2
                    Vector2d odo_res = odometry(post_pix, camera_matrix);
#endif
                    rectangle(bgr, Point(post_dets_[i].x, post_dets_[i].y), Point(post_dets_[i].x + post_dets_[i].w, post_dets_[i].y + post_dets_[i].h), Scalar(0, 0, 255), 2);
                    // putText(bgr, to_string(odo_res.norm()).substr(0,4), Point(post_dets_[i].x, post_dets_[i].y+post_dets_[i].h),
                    //     FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
                    i++;
                }
                send_image(bgr);
            }
        }
#endif

        is_busy_ = false;
    }
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
            imu_mtx_.lock();
            if (!imu_datas_.empty())
            {
                Imu::imu_data imu_data_ = imu_datas_.front();
                int spf = spfs_.front();
                std::vector<double> foot_degs = foot_degs_.front();
                std::vector<double> head_degs = head_degs_.front();
                AngleAxisd roll, pitch;
                pitch = AngleAxisd(deg2rad(imu_data_.pitch), Vector3d::UnitY());
                roll = AngleAxisd(deg2rad(imu_data_.roll), Vector3d::UnitX());
                Quaternion<double> quat = roll * pitch;

                head_yaw_ = -head_degs[0];
                head_pitch_ = head_degs[1];
                TransformMatrix body = ROBOT->leg_forward_kinematics(foot_degs, spf);
                body.set_R(quat.matrix());
                camera_matrix_ = body * TransformMatrix(0, 0, ROBOT->trunk_length()) * TransformMatrix(head_degs[0], 'z') * TransformMatrix(0, 0, ROBOT->neck_length()) * TransformMatrix(head_degs[1], 'y') * TransformMatrix(-0.02, 0, ROBOT->head_length());
            }
            imu_mtx_.unlock();
        }
        frame_mtx_.unlock();
        return;
    }
    if (type == Sensor::SENSOR_IMU)
    {
        shared_ptr<Imu> sptr = dynamic_pointer_cast<Imu>(pub);
        imu_mtx_.lock();
        imu_datas_.push(sptr->data());
        if (imu_datas_.size() > 4)
            imu_datas_.pop();
        int spf = WM->support_foot();
        foot_degs_.push(ROBOT->get_foot_degs(spf));
        head_degs_.push(ROBOT->get_head_degs());
        spfs_.push(spf);
        if (foot_degs_.size() > 4)
            foot_degs_.pop();
        if (head_degs_.size() > 4)
            head_degs_.pop();
        if (spfs_.size() > 4)
            spfs_.pop();
        imu_mtx_.unlock();
        return;
    }
}
#endif


bool Vision::start()
{
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

    err = cudaMemcpy(pCamKData, camK.data, 9 * sizeof(float), cudaMemcpyHostToDevice);
    check_error(err);
    err = cudaMemcpy(pInvNewCamKData, invCamK.data, 9 * sizeof(float), cudaMemcpyHostToDevice);
    check_error(err);
    err = cudaMemcpy(pDistortData, D.data, 4 * sizeof(float), cudaMemcpyHostToDevice);
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
}

void Vision::stop()
{
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
}
