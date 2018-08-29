#include <vector>
#include <math.h> /* floor */
#include <assert.h>

#include <sstream>
#include <string>

#include "ros/ros.h"
#include "ros/console.h"

#include "move_in_cpp.h"


MoveAutoLaserArgmax::MoveAutoLaserArgmax(ros::NodeHandle* nodehandle) : nh(*nodehandle) {
  throttle = 0;
  yaw = YAW_MID;
  msg = mavros_msgs::OverrideRCIn();

  mavros_rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>(
    "mavros/rc/override",
    5,
    true
  );

  scan_sub = nh.subscribe(
    "/scan",
    1,
    &MoveAutoLaserArgmax::scan_cb,
    this
  );
}


void MoveAutoLaserArgmax::scan_cb(const sensor_msgs::LaserScan& laser_scan_msg) {
  std::stringstream log_msg;

  std::vector<float> scan;

  float total = 0;
  for (auto x : laser_scan_msg.ranges)
    total += x;

  // Cut out the relevant part of the scan
  for (int i=0; i<laser_scan_msg.ranges.size(); i++)
    if (i >= MIN_ID && i <= MAX_ID)
      scan.push_back(laser_scan_msg.ranges[i]);

  // Zero-out extreme values
  int scan_size = scan.size();
  for (int i=0; i<scan_size; i++)
    if (scan[i] >= laser_scan_msg.range_max || scan[i] <= laser_scan_msg.range_min)
      scan[i] = 0;

  // Find the group with the largest scan values
  float max_group_sum = -1.0;
  int max_group_id = -1;
  for (int group_id=0; group_id<GROUPS_COUNT; group_id++) {
    float group_sum = 0;
    for (int i=0; i<SCANS_IN_GROUP; i++)
      group_sum += scan[group_id*SCANS_IN_GROUP + i];

    if (group_sum > max_group_sum) {
      max_group_sum = group_sum;
      max_group_id = group_id;
    }
  }

  float angle = -0.5 + 1.0 * max_group_id / GROUPS_COUNT;

  // Corrections for the `angle`
  if (angle >= -0.1 && angle <= 0.1) {
    float sum1 = 0.0;
    for (int i=0; i<180; i++)
      sum1 += scan[i];

    float sum2 = 0.0;
    for (int i=1; i<=180; i++)
      sum2 += scan[scan_size-i];

    if (sum2 > 1.25 * sum1)
      angle += 0.03;
    if (sum1 > 1.25 * sum2)
      angle -= 0.03;
  }

  yaw = floor(YAW_MID + angle * 800 * 1.5);

  ROS_INFO("yaw: %.4f throttle: %.4f", yaw, throttle);

  msg.channels[THROTTLE_CHANNEL] = throttle;
  msg.channels[STEER_CHANNEL] = yaw;
  mavros_rc_override_pub.publish(msg);
}


int main(int argc, char **argv) {


  ros::init(argc, argv, "tryrover_cpp");

  ros::NodeHandle nh;
  MoveAutoLaserArgmax mover(&nh);

  ros::Rate loop_rate(40);

  ros::spin();

  return 0;
}
