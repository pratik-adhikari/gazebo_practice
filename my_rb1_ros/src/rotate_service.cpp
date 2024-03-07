#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "ros/ros.h"

#include <cmath>
#include <tf/transform_datatypes.h>

double current_yaw = 0;
ros::Publisher cmd_vel_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // convert quaternion to RPY
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw = yaw;
}

bool rotate(my_rb1_ros::Rotate::Request &req,
            my_rb1_ros::Rotate::Response &res) {
  double target_yaw = req.degrees * M_PI / 180; // degrees to radians
  double start_yaw = current_yaw;
  double rotation_direction = target_yaw >= 0 ? 1 : -1;

  ros::Rate rate(10);
  while (ros::ok()) {
    double yaw_diff = fabs(current_yaw - start_yaw);
    if (yaw_diff >= fabs(target_yaw) - 0.05) { // Tolerence 0.05 rad
      // StopRobot
      geometry_msgs::Twist stop_twist;

      stop_twist.angular.z = 0;
      cmd_vel_pub.publish(stop_twist);

      std::string direction =
          rotation_direction > 0 ? "anticlockwise (left)" : "clockwise (right)";
      res.result = "Rotation completed: " + std::to_string(fabs(req.degrees)) +
                   " degrees " + direction;
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

  // Subscribe to odom
  ros::Subscriber sub = n.subscribe("/odom", 100, odomCallback);
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

  ros::ServiceServer service = n.advertiseService("/rotate_robot", rotate);

  ROS_INFO("Ready to Rotate the Robot...");
  ros::spin();

  return 0;
}