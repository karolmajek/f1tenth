#include <vector>

#include "ros/ros.h"

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
  std::vector<float> scan;
  for (int i=0; i<laser_scan_msg.ranges.size(); i++)
    if (i >= MIN_ID && i <= MAX_ID)
      scan.push_back(laser_scan_msg.ranges[i]);

  for (int i=0; i<scan.size(); i++)
    if (scan[i] >= laser_scan_msg.range_max || scan[i] <= laser_scan_msg.range_min)
      scan[i] = 0;

  


  msg.channels[THROTTLE_CHANNEL] = throttle;
  msg.channels[STEER_CHANNEL] = YAW_MID;

  mavros_rc_override_pub.publish(msg);
}


int main(int argc, char **argv) {


  ros::init(argc, argv, "tryrover_cpp");

  ros::NodeHandle nh;
  MoveAutoLaserArgmax mover(&nh);

  ros::Rate loop_rate(10);

  ros::spin();

  return 0;
}
