#ifndef MOVE_AUTO_LASER_ARGMAX
#define MOVE_AUTO_LASER_ARGMAX

#include <math.h> /* floor */
#include <assert.h>

#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/LaserScan.h>


class MoveAutoLaserArgmax {
private:
  static constexpr float YAW_MID = 1570;
  static constexpr int MIN_ID = 180;
  static constexpr int MAX_ID = 900;
  static constexpr int GROUPS_COUNT = 10;
  static constexpr int THROTTLE_CHANNEL = 2;
  static constexpr int STEER_CHANNEL = 0;
  static const int SCANS_IN_GROUP = floor((MAX_ID-MIN_ID)/GROUPS_COUNT);
  float throttle;
  float yaw;

  // ROS-related
  ros::NodeHandle nh;
  ros::Subscriber scan_sub;
  ros::Publisher mavros_rc_override_pub;
  mavros_msgs::OverrideRCIn msg;

public:
  MoveAutoLaserArgmax(ros::NodeHandle* nodehandle);
  void scan_cb(const sensor_msgs::LaserScan& msg);
};


#endif // MOVE_AUTO_LASER_ARGMAX
