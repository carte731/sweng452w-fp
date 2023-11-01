// General Includes

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

// Global variables
// Gate for main loops, 
// quit command comes from GCS
bool GATE = true;

//char semROS[] = "Sem-ROS";
//char semROS[] = "Sem-CMD";

sem_t ROScontrol, cmdControl; 

// The telemetry struct
//typedef struct telemetryPacket {
typedef class telemetryPacket {
public:
    // Time
    string execTime;

    // Orientation
    string orientationX;
    string orientationY;
    string orientationZ;
    string orientationW;

    // Angular Velocity
    string angularVelocityX;
    string angularVelocityY;
    string angularVelocityZ;

    // Linear Acceleration
    string linearAccelX;
    string linearAccelY;
    string linearAccelZ;

    vector<string> structContents;
     
    void addToVector(){
        structContents.push_back(this->execTime);
        structContents.push_back(this->orientationX);
        structContents.push_back(this->orientationY);
        structContents.push_back(this->orientationZ);
        structContents.push_back(this->orientationW);
        structContents.push_back(this->angularVelocityX);
        structContents.push_back(this->angularVelocityY);
        structContents.push_back(this->angularVelocityZ);
        structContents.push_back(this->linearAccelX);
        structContents.push_back(this->linearAccelY);
        structContents.push_back(this->linearAccelZ);
    };

} teleMsg;

// Telemetry data vector from vehicle
//vector<teleMsg> telemVector;
queue<teleMsg> telemVector;

class ROSMovementController {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    //ros::Rate loop_rate(2);
    geometry_msgs::Twist speed;

public:

    //void ROSTelemtryInit(Telemetry *teleObj);

    // CONSTRUCTOR, and INITIALIZERS
    ROSMovementController(){
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
        //this->init();
    }

    void init(){
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
        //loop_rate::Rate(2);
    }

    // METHODS
    void publishSpeed(){
        ros::Rate loop_rate(2);
        pub.publish(this->speed);
        ros::spinOnce();
        loop_rate.sleep();
        
    }

    // GETTERS

    // SETTERS
    void setDirection(double linerX, double angularZ){
        speed.linear.x = linerX;
        speed.angular.z = angularZ;
    }

};

class NetworkController {
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
    ROSMovementController ROSMove;

public:

    // CONSTRUCTOR, DESTUCTOR and INITIALIZERS
    NetworkController(int inputPort){
        // Assigning Port number for server
        this->portNum = inputPort;

        // Initializing server
        this->init();
    }

    NetworkController(){
        // Default port number
        this->portNum = 3390;

        // Initializing server
        this->init();
    }

    ~NetworkController(){
        // closing the connected socket
	    close(new_socket);

        // closing the listening socket
        close(server_fd);
    }

    void init(){
        // Creating socket file descriptor
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Forcefully attaching socket to the port 3390
        if (setsockopt(server_fd, SOL_SOCKET,
                    SO_REUSEADDR | SO_REUSEPORT, &opt,
                    sizeof(opt))) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(portNum);
        
        // Forcefully attaching socket to the port 3390
        if (bind(server_fd, (struct sockaddr*)&address,
                sizeof(address))
            < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        if (listen(server_fd, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }

    }

    // METHODS

    void threadServer(){
        std::string word;
		std::stringstream stream;
		while ((GATE) && (new_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen))){ 
			sem_wait(&cmdControl);
			sleep(1);


			bzero(buffer, sizeof(buffer));
			
			if (new_socket < 0) {
				perror("accept");
				exit(EXIT_FAILURE);
			}

			while((GATE) && (valread = read(new_socket, buffer,1024 - 1))){ 
				// subtract 1 for the null
				// terminator at the end

				//printf("%s\n", buffer);
				std::string word(buffer);
                cout << word << endl;

                this->translationUnit(word);
				GATE = (word != "QUIT") ? true : false;

				bzero(buffer, sizeof(buffer));
				write(new_socket, confirmation, strlen(confirmation));
			}

			cout << "Command confirmation message sent" << endl;

			sem_post(&cmdControl);
		}
    }

    void translationUnit(string userInput){
        //switch (userInput)
        //{
        if (userInput == "UP"){
            ROSMove.setDirection(.5, 0);
            ROSMove.publishSpeed();

            usleep(500000);
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

    // GETTERS

    // SETTERS
    
    int getPort(){
        return(this->portNum);
    }
};

class Telemetry{
private:
    // Socket Variables
    int portNum = 0;
    string ipAddr = "";
    bool isConnected = false;
	int valread, client_fd;
	struct sockaddr_in serv_addr;
	char buffer[1024] = { 0 };

    // ROS Object Instance
    //ROSMovementController ROSMove;
    int heartBeats = 0;
    

public:

    Telemetry(string inputIPAddr, int inputPortNum){
        this->ipAddr = inputIPAddr;
        this->portNum = inputPortNum;
        if(this->clinetInit() != 0){
            this->isConnected = false;
            cout << "Error establishing telemetry bridge to GCS..." << endl;
            
        }

        //ROSTelemtryInit(this);
    }

    Telemetry(){
        // My VM IP Address (Using a network bridge)
        this->ipAddr = "10.0.0.101";
        this->portNum = 9000;
        
        if(this->clinetInit() != 0){
            this->isConnected = false;
            cout << "Error establishing telemetry bridge to GCS..." << endl;

        }

        //ROSTelemtryInit(this);

    }

    ~Telemetry(){
        this->closeConnection();
    }

    void processImnTelemetry(const sensor_msgs::Imu::ConstPtr& msg){
        teleMsg newMSg;

        newMSg.execTime = "Time-Stamp/ID: " + to_string(msg->header.stamp.sec) + "\n";

        newMSg.orientationX = "orientation X: " + to_string(msg->orientation.x) + "\n";
        newMSg.orientationY = "orientation Y: " + to_string(msg->orientation.y) + "\n";
        newMSg.orientationZ = "orientation Z: " + to_string(msg->orientation.z) + "\n";
        newMSg.orientationW = "orientation W: " + to_string(msg->orientation.w) + "\n";

        newMSg.angularVelocityX = "Angular Velocity X: " + to_string(msg->angular_velocity.x) + "\n";
        newMSg.angularVelocityY = "Angular Velocity Y: " + to_string(msg->angular_velocity.y) + "\n";
        newMSg.angularVelocityZ = "Angular Velocity Z: " + to_string(msg->angular_velocity.z) + "\n";

        newMSg.linearAccelX = "Linear Acceleration X: " + to_string(msg->linear_acceleration.x) + "\n";
        newMSg.linearAccelY = "Linear Acceleration Y: " + to_string(msg->linear_acceleration.y) + "\n";
        newMSg.linearAccelZ = "Linear Acceleration Z: " + to_string(msg->linear_acceleration.z) + "\n";
        
        //cout << "Linear Acceleration: " << msg->linear_acceleration.x << endl;
        cout << "IMU Added..." << endl;

        newMSg.addToVector();

        // Used for testing
        //printTelemtry(newMSg);

        // Add Mutex and semaphore here
        //telemVector.push_back(newMSg);
        telemVector.push(newMSg);
    }

    // Used for testing
    void printTelemtry(teleMsg newMSg){

       for(auto element : newMSg.structContents){
            cout << element << endl;
       }
    }

    int clinetInit(){
        if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            cout << "Socket creation error" << endl;

            if(this->clinetInit() != 0){
                cout << "\nReconnection failed - exiting program" << endl;
                GATE = false;
            }
            return(-1);
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(this->portNum);

        // Convert IPv4 and IPv6 addresses from text to binary form
        const char *ipAddress = this->ipAddr.data();
        if (inet_pton(AF_INET, ipAddress, &serv_addr.sin_addr) <= 0) {
            cout << "Invalid address/ Address not supported \n" << endl;

            if(this->clinetInit() != 0){
                cout << "\nReconnection failed - exiting program" << endl;
                GATE = false;
            }
            return(-1);
        }

        if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            cout << "Connection Failed..." << endl;
            return(-1);
        }

        this->isConnected = true;
        return(0);
    }

    int sendTelemetry(){
        if(this->isConnected == false){
            if(this->clinetInit() != 0){
                cout << "\nFailed to connect to GCS - Telemetry failed" << endl;
            }

            if(this->clinetInit() != 0){
                cout << "\nReconnection failed - exiting program" << endl;
                GATE = false;
            }
            return(-1);
        }

        if(!telemVector.empty()){
            //teleMsg newMSg = telemVector[0];
            //telemVector.erase(telemVector.begin());

            teleMsg newMSg = telemVector.front();
            telemVector.pop();          

            // Add Mutex and semaphore here
            //cout << "Presend" << endl;
            for(auto element : newMSg.structContents){
                const char *telemElement = element.data();
                //const char *telemElement = "\nSERVER-TEST\n";
                //write(client_fd, test, strlen(test));
                if(send(client_fd, telemElement, strlen(telemElement), 0) <= -1){
                    cout << "\nError transmitting telemetry data... Attempting reconnection..." << endl;

                    if(this->clinetInit() != 0){
                        cout << "\nReconnection failed - exiting program" << endl;
                        GATE = false;
                    }
                }

                //usleep(1000);
                //if(read(client_fd, buffer, 1024 - 1) == -1){
                //    cout << "Error receving confirmation of tel " << endl;
                //}

                //printf("%s\n\n", buffer);

            }

            //cout << "Telemetry messages sent..." << endl;
        }

        return(0);
    }

    int sendHeartBeat(){
        if(this->isConnected == false){
            if(this->clinetInit() != 0){
                cout << "\nFailed to connect to GCS - Telemetry failed" << endl;
            }

            return(-1);
        }

        usleep(10000);
        this->heartBeats++;
        const char *telemElement = "HeartBeat";
        //const char *telemElement = "Heart" + this->heartBeats.data();
        if(send(client_fd, telemElement, strlen(telemElement), 0) <= -1){
            // ADD ERROR TRACKER LATER!!!
            cout << "\nError sending heart-beat... Attempting reconnection..." << endl;

            if(this->clinetInit() != 0){
                cout << "\nReconnection failed - exiting program" << endl;
                GATE = false;
            }
            return(-1);
        }
        
        return(0);
    }

    void closeConnection(){
        // closing the connected socket
        close(client_fd);

    }

    void dataExecution(int itr){
        switch(itr % 1000){
            case(0):
                //cout << "Sending Heat-Beat..." << endl;
                this->sendHeartBeat();
                break;
            case(1):
                //cout << "Sending Telemetry..." << endl;
                this->sendTelemetry();
                break;
        }
    }

};

