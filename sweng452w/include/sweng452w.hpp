// INCLUDE HEADERS

// C++
#include <string>
#include <cinttypes>
#include <bits/stdc++.h>
#include <vector>
#include <queue>

// C
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <semaphore.h>

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

using namespace std;

// GLOBAL VARIABLES

// Gate for main loops and 
// quit command comes from GCS
bool GATE = true;

// Semaphores that allow for interrupts in main loop
sem_t ROScontrol, cmdControl; 

// Telemetry data vector from vehicle
// Contains vehicle IMU data
queue<string> telemVector;

// Controls the ROS robot 
class ROSMovementController {
private:
    // Node the holds the program
    ros::NodeHandle nh;
    // The publisher that sends messages to software bus
    ros::Publisher pub;
    // Used for movement message to ROS
    geometry_msgs::Twist speed;

public:

    // CONSTRUCTOR, and INITIALIZERS
    ROSMovementController(){
        // Tells ROS movement messages will be sent out of this program
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    }


    // METHODS
    // Moves the vehicle at saved speed
    void publishSpeed(){
        // The frame cycle rate for movement commands
        ros::Rate loop_rate(2);
        // Publish the message to the ROS robot
        pub.publish(this->speed);
        // Once command has been excuted, don't wait until 
        // frame cycle is over - this just ends it then
        ros::spinOnce();
        // Sleep until more commands
        loop_rate.sleep();
        
    }

    // GETTERS
    // Returns current forward and reverse speeds
    double getLinearX(){
        return(this->speed.linear.x);
    }

    double getAngularZ(){
        return(this->speed.angular.z );
    }

    // SETTERS
    // Sets direction by setting speed of robot vehicle
    void setDirection(double linerX, double angularZ){
        this->speed.linear.x = linerX;
        this->speed.angular.z = angularZ;
    }

};

// Receives commands from Ground Control Station (GCS)
// It's a server that waits and listens for inbound commands
class CommandServer {
private:
    // Socket Variables
    int portNum = 0;
    int server_fd, new_socket;
	ssize_t valread;
	struct sockaddr_in address;
	int opt = 1;
	socklen_t addrlen = sizeof(address);
	char buffer[1024] = { 0 };
	const char* confirmation = "Command Message Recevied by Server";

    // ROS Object Instance
    // Once moves have been translated, this moves the robot
    ROSMovementController ROSMove;

public:

    // CONSTRUCTOR, DESTUCTOR and INITIALIZERS
    CommandServer(int inputPort){
        // Assigning Port number for server
        this->portNum = inputPort;

        // Initializing server
        this->init();
    }

    CommandServer(){
        // Default port number
        // Varialbe pulled from environmental variable
        // in config bash script
        char* tempPort = getenv("CMD_PORT");
        this->portNum = atoi(tempPort);

        // Initializing server
        this->init();
    }

    ~CommandServer(){
        // closing the connected socket
	    close(new_socket);

        // closing the listening socket
        close(server_fd);
    }

    // Sets up input command server
    void init(){

        // Creating socket file descriptor
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Attaching socket to the port specified in config file
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        // Filling address struct
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(portNum);
        
        // Attaching socket to the port specified in config file
        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        // Start listening for input
        if (listen(server_fd, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }

    }

    // METHODS
    // Runs on a seperate thread, waits for signal from GCS
    void threadServer(){
        std::string word;
		std::stringstream stream;
        // Waits for any new connection, only closes when the quie command is entered
		while ((GATE) && (new_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen))){ 
            // This semaphore allows the input commands to interrupt the 
            // standard heartbeat and telemetry data outputs to the GCS.
            // This allows me to create a "Round Robin" with interrupts scheduler
			sem_wait(&cmdControl);
			sleep(1);

            // clears out buffer for command input
			bzero(buffer, sizeof(buffer));
			
            // Check socket binding
			if (new_socket < 0) {
				perror("accept");
				exit(EXIT_FAILURE);
			}

            // Receives command from GCS
            read(new_socket, buffer, 1024 - 1);

            // Converts const char into string
            std::string word(buffer);
            // Prints command - mostly used for testing
            cout << word << endl;

            // Checks if command input from GCS is valid
            // Mostly needed for command file input
            this->translationUnit(word);

            // If the user wants to quit, then it
            // exits the loop and program
            GATE = (word != "QUIT") ? true : false;

            // Clears the server input buffer
            bzero(buffer, sizeof(buffer));

            // Writes the confirmation message back to GCS
            write(new_socket, confirmation, strlen(confirmation));

            // Used for debugging and testing
			cout << "Command confirmation message sent" << endl;

            // Release both semaphores, so the main Round Robin loop
            // can continue to execute
            sem_post(&ROScontrol);
			sem_post(&cmdControl);
		}
    }

    // Checks if command input from GCS is valid,
    // If so issue order to ROS Controller object to move the robot.
    // Otherwise just ignore the command. 
    void translationUnit(string userInput){
        if (userInput == "UP"){
            // Standard movement forward
            ROSMove.setDirection(.5, 0);
            // Move the robot vehicle
            ROSMove.publishSpeed();
            // Sleep to clear ROS buffers
            usleep(500000);

            // Tell the ROS robot to stop moving
            ROSMove.setDirection(0, 0);
            ROSMove.publishSpeed();
        } else if(userInput == "DOWN"){
            ROSMove.setDirection(-.5, 0);
            ROSMove.publishSpeed();

            usleep(500000);
            ROSMove.setDirection(0, 0);
            ROSMove.publishSpeed();
        }
    }

    // Gets the port number for the server
    int getPort(){
        return(this->portNum);
    }

};

// Used for sending the heartbeat and telemetry 
// to the GCS. The heartbeat lets the GCS know that there is a 
// connection (when there are no movements).
// The telemetry tracks the Inertial Measurement Unit (IMU) on the
// vehicle (tracks speed, angular rate and orientation of robot vehicle)
class TelemetryClient{
private:
    // Socket Variables
    int portNum = 0;
    string ipAddr = "";
	int client_fd;
	struct sockaddr_in serv_addr;

    // Tracks if there is a connection between
    // the RC robot and the GCS
    bool isConnected = false;

    // Tracks how many heartbeats were sent
    int heartBeats = 0;

public:

    TelemetryClient(string inputIPAddr, int inputPortNum){
        this->ipAddr = inputIPAddr;
        this->portNum = inputPortNum;
        if(this->clientInit() != 0){
            this->isConnected = false;
            cout << "Error establishing telemetry bridge to GCS..." << endl;
            
        }
    }

    // CONSTRUCTOR, DESTUCTOR and INITIALIZERS
    TelemetryClient(){
        // Grabs networking variables from config file
        // Saved as char array
        char* tempIP = std::getenv("GCS_IPADDR");
        char* tempPort = std::getenv("GCS_PORT");
        
        // Converted from char to string and int
        this->ipAddr = tempIP;
        this->portNum = atoi(tempPort);
        
        // Initializes connect with GCS
        if(this->clientInit() != 0){
            this->isConnected = false;
            cout << "Error establishing telemetry bridge to GCS..." << endl;

        }
    }

    ~TelemetryClient(){
        this->closeConnection();
    }

    // A handler for inbound ROS IMU messages from RC robot vehicle
    // Saves them output to a string and it's pushed to a queue to be
    // outprocessed to the GCS telemetry window. 
    void processImnTelemetry(const sensor_msgs::Imu::ConstPtr& msg){
        string newMSg;

        // Header for GCS print-out of widget
        newMSg = "\n\n----------- INBOUND IMU DATA --------------\n";

        // Grabs the data from the message
        newMSg += "Time-Stamp/ID: " + to_string(msg->header.stamp.sec) + "\n";

        newMSg += "orientation X: " + to_string(msg->orientation.x) + "\n";
        newMSg += "orientation Y: " + to_string(msg->orientation.y) + "\n";
        newMSg += "orientation Z: " + to_string(msg->orientation.z) + "\n";
        newMSg += "orientation W: " + to_string(msg->orientation.w) + "\n";

        newMSg += "Angular Velocity X: " + to_string(msg->angular_velocity.x) + "\n";
        newMSg += "Angular Velocity Y: " + to_string(msg->angular_velocity.y) + "\n";
        newMSg += "Angular Velocity Z: " + to_string(msg->angular_velocity.z) + "\n";

        newMSg += "Linear Acceleration X: " + to_string(msg->linear_acceleration.x) + "\n";
        newMSg += "Linear Acceleration Y: " + to_string(msg->linear_acceleration.y) + "\n";
        newMSg += "Linear Acceleration Z: " + to_string(msg->linear_acceleration.z) + "\n";

        newMSg += "----------- END OF IMU DATA ---------------\n\n";
        
        cout << "IMU Added to buffer..." << endl;

        // Adds data to vector
        telemVector.push(newMSg);
    }

    // Initializes the client socket to GCS
    int clientInit(){
        // Socket creation
        if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            cout << "Socket creation error" << endl;
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(this->portNum);

        // Convert IPv4 and IPv6 addresses from text to binary form
        const char *ipAddress = this->ipAddr.data();
        if (inet_pton(AF_INET, ipAddress, &serv_addr.sin_addr) <= 0) {
            cout << "Invalid address/ Address not supported \n" << endl;
        }

        // Initial connection to GCS server
        if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            cout << "Connection Failed..." << endl;
            return(-1);
        }

        // Tells otehr methods that connection was successful
        this->isConnected = true;
        return(0);
    }

    // Sends IMU telemetry data to GCS
    int sendTelemetry(){
        // Checks if the connection is active
        if(this->isConnected == false){
            // Reattempts connection, if it fails
            // then the program exits
            if(this->clientInit() != 0){
                cout << "\nFailed to connect to GCS - Telemetry failed to send" << endl;
                GATE = false;
            }

            return(-1);
        }

        // If the IMU telemetry vector is not empty, 
        // then send the data to the GCS.
        // One message is sent per frame cycle in the
        // Round Robin with interrupts.
        if(!telemVector.empty()){
            
            // Grab and remove message and send it
            string newMSg = telemVector.front();
            telemVector.pop();  

            const char *sendMsg = newMSg.data();      

            if(send(client_fd, sendMsg, strlen(sendMsg), 0) <= -1){
                cout << "\nError transmitting telemetry data..." << endl;

                if(this->clientInit() != 0){
                    cout << "\nReconnection failed - exiting program" << endl;
                    GATE = false;
                }
                return(-1);
            }
            cout << "IMU DATA SENT" << endl;
            usleep(1000);

        }

        return(0);
    }

    // Sends heartbeat to the GCS
    // Lets the GCS know that the two systems are 
    // still connected and active.
    // If GCS doesn't receive a heartbeat in 4 minutes,
    // then the connection is cut.
    int sendHeartBeat(){
        // Double checks if system is still connected to the
        // GCS
        if(this->isConnected == false){
            if(this->clientInit() != 0){
                cout << "\nFailed to connect to GCS - Telemetry failed" << endl;
            }

            return(-1);
        }

        // Sends the heartbeat to the GCS 
        this->heartBeats++;
        const char *telemElement = "HeartBeaT";
        if(send(client_fd, telemElement, strlen(telemElement), 0) <= -1){
            cout << "\nError sending heart-beat... Attempting reconnection..." << endl;

            if(this->clientInit() != 0){
                cout << "\nReconnection failed - exiting program" << endl;
                GATE = false;
            }
            return(-1);
        }

        usleep(10000);
        return(0);
    }

    // Closed the client socket
    // used for object deconstructor
    void closeConnection(){
        // closing the connected socket
        close(client_fd);

    }

    // Alternates between heartbeats and sending available IMU
    // data. Used for the Round-Robin functionalty
    void dataExecution(int itr){
        switch(itr % 1000){
            case(0):
                //cout << "Sending Heat-Beat..." << endl;
                this->sendHeartBeat();
                break;
            default:
                //cout << "Sending Telemetry..." << endl;
                this->sendTelemetry();
                break;
        }
    }

};

// Container class used for organization
// Holds all the objects
typedef class NetworkInterfaceController {
public:
    CommandServer CMDServer;
    TelemetryClient TelemServer;

} NIC;

