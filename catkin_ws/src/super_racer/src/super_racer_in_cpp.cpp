#include <vector>
#include <math.h> /* floor, abs */
#include <assert.h>
#include <algorithm> /* min_element, max_element */

#include "ros/ros.h"
#include "ros/console.h"

#include "super_racer_in_cpp.h"


#define PI_BY_180 3.14159265/180


SuperRacer::SuperRacer(ros::NodeHandle* nodehandle) : nh(*nodehandle) {
  throttle = 0;
  yaw = YAW_MID;

  for (int i=0; i<NUM_PNTS_TO_STORE; i++)
    sin_alpha.push_back(std::sin((i+1)*0.25 * PI_BY_180));

  old_time_ = ros::Time::now();

  msg = mavros_msgs::OverrideRCIn();

  mavros_rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>(
    "mavros/rc/override",
    5,
    true
  );

  scan_sub = nh.subscribe(
    "/scan",
    1,
    &SuperRacer::scan_cb,
    this
  );
}


void SuperRacer::scan_cb(const sensor_msgs::LaserScan& data) {
  // Measuring time of execution (and NOT CPU time) is tricky. See:
  // https://stackoverflow.com/questions/2962785/c-using-clock-to-measure-time-in-multi-threaded-programs/2962914#2962914
  // for a discussion of the solution used below
  time_ = ros::Time::now();
  double time_delta;

  std::vector<float> scan;
  // Cut out the relevant part of the scan
  float one_scan;
  int margin = 180;
  for (int i=margin; i<data.ranges.size()-margin-1; i++) {
    one_scan = data.ranges[i];
    one_scan = one_scan < 0 ? 0 : one_scan > 10 ? 10 : one_scan;
    scan.push_back(one_scan);
  }

  yaw = steerMAX(scan);

  time_delta = time_.toSec() - old_time_.toSec();
  // std::cout << time_delta << std::endl;
  ROS_INFO("yaw: %.4f throttle: %.4f time_delta: %.4f", yaw, throttle, time_delta);

  msg.channels[THROTTLE_CHANNEL] = throttle;
  msg.channels[STEER_CHANNEL] = yaw;
  mavros_rc_override_pub.publish(msg);

  old_time_ = time_;
}


float SuperRacer::steerMAX(std::vector<float> & scan) {
  // The following actually copies the vector `scan`
  std::vector<float> scan2(scan);

  int idx = 0;
  int scan_size = scan.size();
  bool is_reachable = false;

  std::vector<int> segs = {0, scan_size-1};

  for (int i=1; i<scan_size; i++)
    if (std::abs(scan[i]-scan[i-1]) > NON_CONT_DIST) {
      segs.push_back(i);
      segs.push_back(i-1);
    }

  // Assert that scan2 has only non-negative values (very important below)
  idx = std::distance(
    scan2.begin(),
    std::min_element(scan2.begin(), scan2.end())
  );
  assert (scan2[idx] >= 0);


  int safety_counter = 0;
  while (!is_reachable) {
    // Search for argmax (https://en.cppreference.com/w/cpp/algorithm/max_element)
    idx = std::distance(
      scan2.begin(),
      std::max_element(scan2.begin(), scan2.end())
    );
    if (scan2[idx] < 0) {
      ROS_WARN("max(scan2) = %.4f, car is going to stop", scan2[idx]);
      throttle = 0;
    }

    for (auto s : segs) {
      if (s != idx) {
        bool could_be_reached = check_if_reachable(scan[idx], scan[s], std::abs(s-idx));
        if (!could_be_reached) {
          scan2[idx] = -1;
          break;
        }
      }
    }
    // Instead of comparing to -1 (remember, we're dealing with floats),
    //  we check if it's non-negative
    if (scan2[idx] >= 0)
      is_reachable = true;
  }

  yaw = 1.0 * idx / scan_size - 0.5;

  // Clip the yaw
  yaw = yaw < -0.5 ? -0.5 : yaw > 0.5 ? 0.5 : yaw;

  return floor(yaw*1200) + YAW_MID;
}


bool SuperRacer::check_if_reachable(float r1, float r2, int alpha) {
  // TODO(MD): Review this method
  if (r1 < r2 + SAFE_OBSTACLE_DIST1)
    return true;
  else
    return (r2*sin_alpha[alpha] > SAFE_OBSTACLE_DIST2);
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "super_racer_in_cpp");

  ros::NodeHandle nh;
  SuperRacer mover(&nh);

  ros::Rate loop_rate(40);

  ros::spin();

  return 0;
}
