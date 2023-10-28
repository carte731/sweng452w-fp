#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "finalProject");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Rate loop_rate(2);

  geometry_msgs::Twist speed;
  speed.linear.x = .5;
  speed.angular.z = .5;

  while (ros::ok()) {
    pub.publish(speed);
    ros::spinOnce();
    loop_rate.sleep();
    //++count.data;
  }

  return 0;
}