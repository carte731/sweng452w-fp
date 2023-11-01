#include "sweng452w.hpp"


int main(int argc, char **argv) {

  sem_init(&ROScontrol, 0, 1); 
	sem_init(&cmdControl, 0, 1); 

  ros::init(argc, argv, "finalProject");
  /*
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Rate loop_rate(2);

  geometry_msgs::Twist speed;
  speed.linear.x = .5;
  speed.angular.z = .5;
  */

  // If GATE closed, then shut down ROS and destory all the objects.
  //signal(SIGINT, mySigintHandler);

  // Creating the NetworkController object,
  // used for inbound movement command requests
  NetworkController server;
  Telemetry telemetryObj;
  
  ros::NodeHandle dataNode;
  ros::Subscriber imuSubscription;
  imuSubscription = dataNode.subscribe("imu", 2, &Telemetry::processImnTelemetry, &telemetryObj);

  //ROSTelemtryInit(&telemetryObj);
  
  // Start a thread that will listen for inbound connects
  // from Ground Control Station (GCS)
  thread t1(&NetworkController::threadServer, &server);
  //ros::Rate loop_rate(2);
  int itr = 0;
  sleep(5);
  while((GATE) && (ros::ok())){
    //// ADD INDEFINE WHILE LOOP IF SYSTEM CANNOT CONNECT TO GCS-!!!
    /*
    pub.publish(speed);
    ros::spinOnce();
    loop_rate.sleep();
    //++count.data;
    */
    //loop_rate.sleep();
    if((sem_wait(&ROScontrol) == 0) && (sem_wait(&cmdControl) == 0)){
      telemetryObj.dataExecution(itr++);
			sem_post(&ROScontrol);
			sem_post(&cmdControl);
		}
  }

	t1.join();

	sem_destroy(&ROScontrol); 
	sem_destroy(&cmdControl); 

  ros::shutdown();

  return(0);
}