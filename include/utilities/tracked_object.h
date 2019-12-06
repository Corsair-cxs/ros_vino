#pragma once

#include <ros_vino/Object.h>
#include "utilities/kalman.h"

class TrackedObject {
public:
  TrackedObject(const ros_vino::Object& object) : object_(object), kf_x_(0, 1, 2), kf_y_(0, 1, 2), kf_h_(0, 1, 2), kf_w_(0, 1, 2) {
    fade_counter_ = s_fade_counter_size;
    initKF();
  }

  void predictState() {
    kf_x_.predictState();
    kf_y_.predictState();
    kf_h_.predictState();
    kf_w_.predictState();

    object_.x = kf_x_.q_pred(0);
    object_.y = kf_y_.q_pred(0);

    object_.vel_x = kf_x_.q_pred(1);
    object_.vel_y = kf_y_.q_pred(1);

    object_.height = kf_h_.q_pred(0);
    object_.width = kf_w_.q_pred(0);

    object_.vel_height = kf_h_.q_pred(1);
    object_.vel_width = kf_w_.q_pred(1);

    fade_counter_--;
  }

  void correctState(const ros_vino::Object& new_object) {
    kf_x_.y(0) = new_object.x;
    kf_y_.y(0) = new_object.y;
    kf_h_.y(0) = new_object.height;
    kf_w_.y(0) = new_object.width;

    kf_x_.correctState();
    kf_y_.correctState();
    kf_h_.correctState();
    kf_w_.correctState();

    object_.x = kf_x_.q_est(0);
    object_.y = kf_y_.q_est(0);

    object_.vel_x = kf_x_.q_est(1);
    object_.vel_y = kf_y_.q_est(1);

    object_.height = kf_h_.q_est(0);
    object_.width = kf_w_.q_est(0);

    object_.vel_height = kf_h_.q_est(1);
    object_.vel_width = kf_w_.q_est(1);

    fade_counter_ = s_fade_counter_size;
  }

  void updateState() {
    kf_x_.predictState();
    kf_y_.predictState();
    kf_h_.predictState();
    kf_w_.predictState();

    kf_x_.correctState();
    kf_y_.correctState();
    kf_h_.correctState();
    kf_w_.correctState();

    object_.x = kf_x_.q_est(0);
    object_.y = kf_y_.q_est(0);

    object_.vel_x = kf_x_.q_est(1);
    object_.vel_y = kf_y_.q_est(1);

    object_.height = kf_h_.q_est(0);
    object_.width = kf_w_.q_est(0);

    object_.vel_height = kf_h_.q_est(1);
    object_.vel_width = kf_w_.q_est(1);

    fade_counter_--;
  }

  static void setSamplingTime(double tp) {
    s_sampling_time = tp;
  }

  static void setCounterSize(int size) {
    s_fade_counter_size = size;
  }

  static void setCovariances(double process_var, double process_rate_var, double measurement_var) {
    s_process_variance = process_var;
    s_process_rate_variance = process_rate_var;
    s_measurement_variance = measurement_var;
  }

  void setId(int num) { object_.id = num; }
  int getId(int num) { return object_.id; }

  bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }
  const ros_vino::Object& getObject() const { return object_; }
  const KalmanFilter& getKFx() const { return kf_x_; }
  const KalmanFilter& getKFy() const { return kf_y_; }
  const KalmanFilter& getKFh() const { return kf_h_; }
  const KalmanFilter& getKFw() const { return kf_w_; }

private:
  void initKF() {
    kf_x_.A(0, 1) = s_sampling_time;
    kf_y_.A(0, 1) = s_sampling_time;
    kf_h_.A(0, 1) = s_sampling_time;
    kf_w_.A(0, 1) = s_sampling_time;

    kf_x_.C(0, 0) = 1.0;
    kf_y_.C(0, 0) = 1.0;
    kf_h_.C(0, 0) = 1.0;
    kf_w_.C(0, 0) = 1.0;

    kf_x_.R(0, 0) = s_measurement_variance;
    kf_y_.R(0, 0) = s_measurement_variance;
    kf_h_.R(0, 0) = s_measurement_variance;
    kf_w_.R(0, 0) = s_measurement_variance;

    kf_x_.Q(0, 0) = s_process_variance;
    kf_y_.Q(0, 0) = s_process_variance;
    kf_h_.Q(0, 0) = s_process_variance;
    kf_w_.Q(0, 0) = s_process_variance;

    kf_x_.Q(1, 1) = s_process_rate_variance;
    kf_y_.Q(1, 1) = s_process_rate_variance;
    kf_h_.Q(1, 1) = s_process_rate_variance;
    kf_w_.Q(1, 1) = s_process_rate_variance;

    kf_x_.q_pred(0) = object_.x;
    kf_y_.q_pred(0) = object_.y;
    kf_h_.q_pred(0) = object_.height;
    kf_w_.q_pred(0) = object_.width;

    kf_x_.q_pred(1) = object_.vel_x;
    kf_y_.q_pred(1) = object_.vel_y;

    kf_x_.q_est(0) = object_.x;
    kf_y_.q_est(0) = object_.y;
    kf_h_.q_est(0) = object_.height;
    kf_w_.q_est(0) = object_.width;

    kf_x_.q_est(1) = object_.vel_x;
    kf_y_.q_est(1) = object_.vel_y;
  }

  ros_vino::Object object_;

  KalmanFilter kf_x_;
  KalmanFilter kf_y_;
  KalmanFilter kf_h_;
  KalmanFilter kf_w_;

  int fade_counter_;

  // Common variables
  static int s_fade_counter_size;
  static double s_sampling_time;
  static double s_process_variance;
  static double s_process_rate_variance;
  static double s_measurement_variance;
};

