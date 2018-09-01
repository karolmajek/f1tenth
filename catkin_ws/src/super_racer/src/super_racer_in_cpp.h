#ifndef SUPER_RACER
#define SUPER_RACER

#include <vector>
#include <math.h> /* floor */
#include <assert.h>

#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/LaserScan.h>


class SuperRacer {
private:
  static constexpr float YAW_MID = 1570;
  static constexpr int THROTTLE_CHANNEL = 2;
  static constexpr int STEER_CHANNEL = 0;
  static constexpr int NUM_PNTS_TO_STORE = 1080;
  static constexpr int NUM_PNTS_TO_CHK = 20;
  static constexpr float SAFE_OBSTACLE_DIST1 = 0.3;
  static constexpr float SAFE_OBSTACLE_DIST2 = 0.3;
  static constexpr float NON_CONT_DIST = 0.1;
  float throttle;
  float yaw;
  std::vector<float> sin_alpha;
  ros::Time time_, old_time_;

  // ROS-related
  ros::NodeHandle nh;
  ros::Subscriber scan_sub;
  ros::Publisher mavros_rc_override_pub;
  mavros_msgs::OverrideRCIn msg;

  float steerMAX(std::vector<float> & scan);
  bool check_if_reachable(float r1, float r2, int alpha);

public:
  SuperRacer(ros::NodeHandle* nodehandle);
  void scan_cb(const sensor_msgs::LaserScan& data);
};


#endif // SUPER_RACER
