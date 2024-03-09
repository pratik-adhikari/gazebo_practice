#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "ros/ros.h"

#include <cmath>
#include <string>
#include <tf/transform_datatypes.h>

double current_yaw = 0;
ros::Publisher cmd_vel_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw = yaw;
}

bool rotate(my_rb1_ros::Rotate::Request &req,
            my_rb1_ros::Rotate::Response &res) {
  ROS_INFO("Requested : %.1f degrees", req.degrees);

  double radians_requested = req.degrees * M_PI / 180; // degrees to radians

  // edge case 360
  if (fabs(fmod(radians_requested, 2 * M_PI)) <
      0.01) { // fmod for modulus with floating point, tolerance 0.01 rad
    res.result = "Requested a 360-degree rotation. The robot is already at the "
                 "same point.";
    ROS_INFO("%s", res.result.c_str());
    return true;
  }

  double target_yaw = fmod(current_yaw + radians_requested,
                           2 * M_PI); // Target yaw considering overflow
  double rotation_direction = radians_requested >= 0 ? 1 : -1;

  ros::Rate rate(10);
  while (ros::ok()) {
    double yaw_diff = fabs(current_yaw - target_yaw);
    if (yaw_diff < 0.05 ||
        yaw_diff >
            2 * M_PI - 0.05) { // Tolerance 0.05 rad, adjusted for overflow
      geometry_msgs::Twist stop_twist;
      stop_twist.angular.z = 0;
      cmd_vel_pub.publish(stop_twist);

      std::string direction =
          rotation_direction > 0 ? "anticlockwise (left)" : "clockwise (right)";

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1)
          << std::round(req.degrees * 10) / 10.0;
      std::string roundedDegrees = oss.str();

      res.result =
          "Completed: Rotation " + roundedDegrees + " degrees " + direction;
      ROS_INFO("%s", res.result.c_str());
      return true;
    }

    geometry_msgs::Twist twist;
    twist.angular.z = rotation_direction * 0.5;
    cmd_vel_pub.publish(twist);

    ros::spinOnce();
    rate.sleep();
  }

  res.result = "Rotation Failed";
  return false;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "rotate_robot_service");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/odom", 100, odomCallback);
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

  ros::ServiceServer service = n.advertiseService("/rotate_robot", rotate);

  ROS_INFO("Ready to Rotate the Robot...");
  ros::spin();

  return 0;
}
