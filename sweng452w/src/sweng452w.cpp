#include "sweng452w.hpp"


int main(int argc, char **argv) {
  
  // Initializes the semaphores.
  // This allows the commands to interrupt the
  // Round-Robin (heartbeat and telemetry)
  sem_init(&ROScontrol, 0, 1); 
	sem_init(&cmdControl, 0, 1); 

  // Initializes the ROS control node and
  // injects my program into the ROS robot vehicle
  ros::init(argc, argv, "finalProject");

  // Holds the Input Command Server for inbound commands for movement.
  // Also holds the telemetry object used for sending commands
  NIC systemFrame;
  
  // Creates a program handle to interact with ROS
  ros::NodeHandle dataNode;

  // Used for subscribing to IMU data
  ros::Subscriber imuSubscription;
  
  // Attaches the inbound IMU data from the ROS robot to the method handler in the telemetry object
  imuSubscription = dataNode.subscribe("imu", 2, &TelemetryClient::processImnTelemetry, &systemFrame.TelemServer);
  
  // Start a thread that will listen for inbound connects
  // from Ground Control Station (GCS)
  thread t1(&CommandServer::threadServer, &systemFrame.CMDServer);

  // The main loop, will continue to operate until
  // the quit command (GATE) is issued from the GCS
  // Or there is an error with ROS.
  // itr controls wheater heartbeat or telemetry are sent out
  // to the GCS
  int itr = 0;
  sleep(5);
  while((GATE) && (ros::ok())){

    // The heartbeat and telemetry Round-Robin with interrupts loop.
    // If there is a command from the GCS, it will interrupt the main flow
    // of heartbeat and telemetry outward commands (to the GCS).
    // Heartbeat and telemetry requires two seamphores, the command input
    // onlt requires one. This gives the command inputs higher priority 
    // (due to it only needing one semaphore).  
    if((sem_wait(&ROScontrol) == 0) && (sem_wait(&cmdControl) == 0)){
      // Alternates between heartbeat (every 1000 iterations)
      // And telemetry IMU messages (if available)
      systemFrame.TelemServer.dataExecution(itr++);
			sem_post(&ROScontrol);
			sem_post(&cmdControl);
		}
  }

  // Merges the command input server thread
	t1.join();

  // Destroys the semaphores
	sem_destroy(&ROScontrol); 
	sem_destroy(&cmdControl); 

  // Shutdown ROS
  ros::shutdown();

  return(0);
}