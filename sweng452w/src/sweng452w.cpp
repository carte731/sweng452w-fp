#include "sweng452w.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "finalProject");
  /*
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Rate loop_rate(2);

  geometry_msgs::Twist speed;
  speed.linear.x = .5;
  speed.angular.z = .5;
  */

  // Creating the NetworkController object,
  // used for inbound movement command requests
  NetworkController server;
  
  // Start a thread that will listen for inbound connects
  // from Ground Control Station (GCS)
  thread t1(&NetworkController::threadServer, &server);
  //ros::Rate loop_rate(2);
  while (ros::ok()) {
    /*
    pub.publish(speed);
    ros::spinOnce();
    loop_rate.sleep();
    //++count.data;
    */
    //loop_rate.sleep();
  }

  return(0);
}