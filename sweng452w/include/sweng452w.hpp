// General Includes
#include <string>
#include <bits/stdc++.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <semaphore.h>

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

using namespace std;

// Global variables
bool GATE = true;

class ROSMovementController {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    //ros::Rate loop_rate(2);
    geometry_msgs::Twist speed;

public:

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
    void setSpeed(double linerX, double angularZ){
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
			//sem_wait(&cmdControl);
			sleep(1);


			bzero(buffer, sizeof(buffer));
			
			if (new_socket < 0) {
				perror("accept");
				exit(EXIT_FAILURE);
			}

			while((GATE) && (valread = read(new_socket, buffer,1024 - 1))){ 
				// subtract 1 for the null
				// terminator at the end

				printf("%s\n", buffer);
				std::string word(buffer);

                this->translationUnit(word);
				GATE = (word != "QUIT") ? true : false;

				bzero(buffer, sizeof(buffer));
				write(new_socket, confirmation, strlen(confirmation));
			}

			cout << "Command confirmation message sent" << endl;

			//sem_post(&cmdControl);
		}
    }

    void translationUnit(string userInput){
        //switch (userInput)
        //{
        if (userInput == "UP"){
            ROSMove.setSpeed(.5, 0);
            ROSMove.publishSpeed();

            usleep(500000);
            ROSMove.setSpeed(0, 0);
            ROSMove.publishSpeed();
        } else if(userInput == "DOWN"){
            ROSMove.setSpeed(-.5, 0);
            ROSMove.publishSpeed();

            usleep(500000);
            ROSMove.setSpeed(0, 0);
            ROSMove.publishSpeed();
        }
    }

    // GETTERS

    // SETTERS
    
    int getPort(){
        return(this->portNum);
    }
};

class Telemery{
private:
    // Socket Variables
    int portNum = 0;
    string ipAddr = "";
	int status, valread, client_fd;
	struct sockaddr_in serv_addr;
	char buffer[1024] = { 0 };

    // ROS Object Instance
    ROSMovementController ROSMove;

public:

    Telemery(string inputIPAddr){
        this->ipAddr = inputIPAddr;
        if(clinetInit() != 0){
            cout << "Error establishing telemetry bridge to GCS..." << endl;
        }
    }

    Telemery(){
        // My VM IP Address (Using a network bridge)
        this->ipAddr = "10.0.0.101";
        if(clinetInit() != 0){
            cout << "Error establishing telemetry bridge to GCS..." << endl; 
        }

    }

    int clinetInit(){
        if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            cout << "Socket creation error" << endl;
            return(-1);
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);

        // Convert IPv4 and IPv6 addresses from text to binary form
        if (inet_pton(AF_INET, this->ipAddr, &serv_addr.sin_addr) <= 0) {
            cout << "Invalid address/ Address not supported \n" << endl;
            return(-1);
        }

        if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            cout << "Connection Failed..." << endl;
            return(-1);
        }

        return(0);
    }


};