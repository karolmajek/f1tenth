#include <vector>
#include <math.h> /* floor, abs */
#include <assert.h>
#include <algorithm> /* min, min_element, max_element */

#include "ros/ros.h"
#include "ros/console.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>

#include "ultimate_racer_in_cpp2.h"


#define PI_BY_180 3.14159265 / 180


// TODO: add `clip` template function


UltimateRacer::UltimateRacer(
  ros::NodeHandle* nodehandle,
  float min_speed,
  float max_speed,
  int init_esc
) {

  nh = *nodehandle;
  speed_record = 0;
  throttle = 1500;
  this->init_esc = init_esc;
  yaw = YAW_MID;
  this->min_speed = min_speed;
  this->max_speed = max_speed;

  estop = false;
  estart = false;
  ego = false;

  kp = 1.8;
  kd = 6.0;
  prev_error = 0.0;

  curr_speed = 0.0;

  float angle_step = 1. / STEPS_PER_DEGREE;
  for (int i=0; i<NUM_ANGLES_TO_STORE; i++)
    sin_alpha[i] = std::sin((i+1)*angle_step * PI_BY_180);

  old_time_ = ros::Time::now();
  last_stop_msg_ts = ros::Time::now().toSec();

  pub_esc = nh.advertise<std_msgs::UInt32>(
    "/esc",
    1
  );

  pub_servo = nh.advertise<std_msgs::UInt16>(
    "/servo",
    1
  );

  sub_spd = nh.subscribe(
    "/spd",
    1,
    &UltimateRacer::spd_cb,
    this
  );

  sub_scan = nh.subscribe(
    "/scan",
    1,
    &UltimateRacer::scan_cb,
    this
  );

  sub_estop = nh.subscribe(
    "/eStop",
    1,
    &UltimateRacer::estop_cb,
    this
  );
}

void UltimateRacer::spd_cb(const std_msgs::Float32 & data) {
  curr_speed = data.data;
}


void UltimateRacer::scan_cb(const sensor_msgs::LaserScan & data) {
  time_ = ros::Time::now();
  if (time_.toSec() - last_stop_msg_ts > 0.5 && (ego || estart))
    exec_estop();

  // For time measurements
  double delta_between_callbacks;
  double delta_within_callback;


  // Finding the smallest range near the center
  int mid_point = floor(data.ranges.size() / 2.);
  int margin = 20; // TODO: make this a parameter
  float min_central_value = *std::min_element(
    data.ranges.begin() + mid_point - margin,
    data.ranges.begin() + mid_point + margin
  );
  if (min_central_value < SAFE_OBSTACLE_DIST3) {
    throttle = ESC_BRAKE;
    tmp_uint16.data = throttle;
    pub_esc.publish(tmp_uint16);
    // TODO: shouldn't this result in a `return`?
  }

  // Clip the values
  std::vector<float> scan;
  float one_scan;
  for (int i=0; i<data.ranges.size(); i++) {
    one_scan = data.ranges[i];
    one_scan = one_scan < 0.06 ? 0.06 : one_scan > 10 ? 10 : one_scan;
    scan.push_back(one_scan);
  }
  for (int i=0; i<90; i++)
    scan[i] = 10.0;
  for (int i=scan.size()-90; i<scan.size(); i++)
    scan[i] = 10.0;

  bool turbo = false;

  int idx = steerMAX(scan, MARGIN_DRIVE_FAST);

  float desired_speed = -1000;
  if (idx == -1 || scan[idx] < FAST_DRIVE_DIST) {
    idx = steerMAX(scan, MARGIN_DRIVE_SLOW);
    if (idx == -1) {
      throttle = ESC_BRAKE;
      desired_speed = 0;
    } else {
      desired_speed = speed_control(scan, idx, turbo);
    }
  } else {
    turbo = true;
    desired_speed = speed_control(scan, idx, turbo);
  }

  float desired_speed_log = desired_speed;

  float idx2yaw;
  if (idx >= 0) {
    idx2yaw = 1.0 * idx / scan.size() - 0.5;
    idx2yaw = 1.2*idx2yaw;
    idx2yaw = idx2yaw < -0.5 ? -0.5 : idx2yaw > 0.5 ? 0.5 : idx2yaw;
    yaw = floor(idx2yaw * YAW_RANGE + YAW_MID);
  }

  if (ego || estart) {
    speed_pid(desired_speed);
    if (estart) {
      throttle = init_esc;
      estart = false;
      ego = true;
    }
    if (!estop) {
      tmp_uint16.data = throttle;
      pub_esc.publish(tmp_uint16);
    }
  }

  // Log everything
  delta_between_callbacks = time_.toSec() - old_time_.toSec();
  delta_within_callback = ros::Time::now().toSec() - time_.toSec();
  ROS_INFO(
    "yaw: %d throttle: %d x: %.4f delta_between_callbacks: %.4f delta_within_callback: %.4f",
    yaw, throttle, desired_speed_log, delta_between_callbacks, delta_within_callback
  );

  old_time_ = time_;
}


void UltimateRacer::estop_cb(const std_msgs::UInt16 & data) {
  last_stop_msg_ts = ros::Time::now().toSec();
  if (data.data == 0) {
    ROS_WARN("Emergency stop!");
    exec_estop();
  }
  else if (data.data == 1) {
    ROS_WARN("Reset!");
    estop = false;
  } else if (data.data == 2) {
    ROS_WARN("GO!");
    estart = true;
  }
}


void UltimateRacer::exec_estop() {
  estop = true;
  yaw = YAW_MID;
  throttle = ESC_BRAKE;
  tmp_uint16.data = throttle;
  pub_esc.publish(tmp_uint16);
  tmp_uint16.data = yaw;
  pub_servo.publish(tmp_uint16);
}


float UltimateRacer::speed_control(std::vector<float> & scan, int idx, bool nitro) {
    float speed = -1000;
    if (nitro) {
      speed = scan[idx];
    } else {
      int left_limit = std::max(idx-SLOW_SPEED_CHK_POINTS, 0);
      int right_limit = std::min(int(scan.size()-1), idx+SLOW_SPEED_CHK_POINTS);
      speed = *std::min_element(
        scan.begin() + left_limit,
        scan.begin() + right_limit
      );
      speed = 1.5*speed;
    }
    return speed;
}


void UltimateRacer::speed_pid(int desired_speed) {
  // TODO: clip this
  float dspeed;
  if (desired_speed > max_speed)
    dspeed = max_speed;
  else if (desired_speed < min_speed)
    dspeed = min_speed;
  else
    dspeed = desired_speed;

  float pid_error = dspeed - curr_speed;
  float error = kp * pid_error;
  float errordot = kd * (pid_error - prev_error);

  float speed_delta = error + errordot;
  prev_error = pid_error;

  if (throttle == ESC_BRAKE && speed_delta > 0)
    throttle = ESC_MIN;

  throttle += speed_delta;

  throttle = throttle < 1000 ? 1000 : throttle > 2000 ? 2000 : throttle;
}


float UltimateRacer::steerMAX(std::vector<float> & scan, float margin) {
  float min_scan = *std::min_element(scan.begin(), scan.end());
  if (min_scan <= SAFE_OBSTACLE_DIST2)
    return -1;

  // The following actually copies the vector `scan`
  std::vector<float> scan2(scan);
  int idx = 0;
  bool is_reachable = false;
  int scan_size = scan.size();

  // First, prepare `segs`
  std::vector<int> segs = {180, scan_size-180};
  for (int i=1; i<scan_size; i++)
    if (std::abs(scan[i]-scan[i-1]) > NON_CONT_DIST) {
      segs.push_back(i);
      segs.push_back(i-1);
    }

  for (int i=0; i<100; i++)
    scan2[i] = -1;
  for (int i=scan2.size()-100; i<scan2.size(); i++)
    scan2[i] = -1;

  while (!is_reachable) {
    float max_value = *std::max_element(scan2.begin(), scan2.end());
    if (max_value <= 0)
      break;

    // Search for argmax (https://en.cppreference.com/w/cpp/algorithm/max_element)
    idx = std::distance(
      scan2.begin(),
      std::max_element(scan2.begin(), scan2.end())
    );

    for (auto s : segs) {
      if (s != idx) {
        bool could_be_reached = check_if_reachable(scan[idx], scan[s], std::abs(s-idx), margin);
        if (!could_be_reached) {
          int left_limit = std::max(0, idx-5);
          int right_limit = std::min(idx+5, int(scan2.size()-1));
          for (int i=left_limit; i<right_limit; i++)
            scan2[i] = -1;
          break;
        }
      }
    }
    // Instead of comparing to -1 (remember, we're dealing with floats),
    //  we check whether it's non-negative
    if (scan2[idx] >= 0)
      is_reachable = true;
  }

  if (is_reachable == false)
    idx = -1;

  return idx;
}


bool UltimateRacer::check_if_reachable(float r1, float r2, int alpha, float margin) {
  // if (SAFE_OBSTACLE_DIST1 < r1 < r2 + SAFE_OBSTACLE_DIST1)
  if (r1 - SAFE_OBSTACLE_DIST2 < r2 && r1 > SAFE_OBSTACLE_DIST2 )
    return true;
  else
    return (r2*sin_alpha[alpha] > margin);
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "ultimate_racer_in_cpp2");

  ros::NodeHandle nh;
  float min_speed = 0.5;
  float max_speed = 6.0;
  int init_esc = 1580;
  UltimateRacer racer(&nh, min_speed, max_speed, init_esc);

  ros::Rate loop_rate(40);

  ros::spin();

  return 0;
}
