#ifndef SUPER_RACER
#define SUPER_RACER

#include <vector>
#include <math.h> /* floor */
#include <assert.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>


class UltimateRacer {
private:
  static constexpr int ESC_BRAKE = 1000;
  static constexpr int ESC_INIT = 1500;
  static constexpr int ESC_MIN = 1550;

  static constexpr int YAW_MID = 1585;
  static constexpr int YAW_RANGE = 1200;

  static constexpr int SLOW_SPEED_CHK_POINTS = 100;

  static constexpr float SAFE_OBSTACLE_DIST1 = 0.5;
  static constexpr float SAFE_OBSTACLE_DIST2 = 0.17;
  static constexpr float SAFE_OBSTACLE_DIST3 = 0.3;

  static constexpr float NON_CONT_DIST = 0.2;

  static constexpr float FAST_DRIVE_DIST = 6.0;
  static constexpr float MARGIN_DRIVE_FAST = 0.5;
  static constexpr float MARGIN_DRIVE_SLOW = 0.3;

  static constexpr int FULL_ANGLE = 360;
  static constexpr int STEPS_PER_DEGREE = 4;
  static constexpr int NUM_ANGLES_TO_STORE = FULL_ANGLE * STEPS_PER_DEGREE;

  float sin_alpha[NUM_ANGLES_TO_STORE];

  int throttle;
  int yaw;
  int init_esc;
  float max_speed;
  float min_speed;
  float curr_speed;
  float speed_record;

  bool estop;
  bool estart;
  bool ego;

  // PID controller
  float kp;
  float kd;
  float prev_error;

  std_msgs::UInt16 tmp_uint16;

  ros::Time time_, old_time_;
  double last_stop_msg_ts;

  // ROS-related
  ros::NodeHandle nh;

  ros::Publisher pub_esc;
  ros::Publisher pub_servo;

  ros::Subscriber sub_scan;
  ros::Subscriber sub_spd;
  ros::Subscriber sub_estop;

  float steerMAX(std::vector<float> & scan, float margin);
  bool check_if_reachable(float r1, float r2, int alpha, float margin);
  float speed_control(std::vector<float> & scan, int idx, bool nitro);
  void speed_pid(int desired_speed);
  void exec_estop();

public:
  UltimateRacer(ros::NodeHandle* nodehandle, float min_speed, float max_speed, int init_esc);
  void spd_cb(const std_msgs::Float32 & data);
  void scan_cb(const sensor_msgs::LaserScan & data);
  void estop_cb(const std_msgs::UInt16 & data);
};


#endif // SUPER_RACER
