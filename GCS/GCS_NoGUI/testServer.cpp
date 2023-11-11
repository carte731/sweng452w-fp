// Server side C/C++ program to demonstrate Socket
// programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <unistd.h>
#define PORT 9000

#include <netinet/in.h>
#include <stdlib.h>
#include <semaphore.h>

// C++ HEADERS
#include <bits/stdc++.h>
#include <iostream>
#include <thread>
#include <string>

using namespace std;

int heartbeat = 0;
bool gate = true;

int main(int argc, char const* argv[]){
    int server_fd, new_socket;
	ssize_t valread;
	struct sockaddr_in address;
	int opt = 1;
	socklen_t addrlen = sizeof(address);
	char buffer[1024] = { 0 };
	char* hello = "Hello from server";

	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port 3395
	if (setsockopt(server_fd, SOL_SOCKET,
				SO_REUSEADDR | SO_REUSEPORT, &opt,
				sizeof(opt))) {
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);
	
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

	//{
		std::string word;
		std::stringstream stream;
		while ((gate) && (new_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen))){ 
			//sem_wait(&cmdControl);
			sleep(1);


			bzero(buffer, sizeof(buffer));
			
			if (new_socket < 0) {
				perror("accept");
				exit(EXIT_FAILURE);
			}

			while((gate) && (valread = read(new_socket, buffer,1024 - 1))){ 
				// subtract 1 for the null
				// terminator at the end

				//printf("%s\n", buffer);
				std::string word(buffer);
                if(word != "HeartBeat"){
                    std::cout << word << "\n" << endl;
                } else {
                    heartbeat++;
                }
				gate = (word != "QUIT") ? true : false;
                word.clear();
				bzero(buffer, sizeof(buffer));
				//write(new_socket, hello, strlen(hello));
			}

			printf("Hello message sent\n\n");
            printf("HEATBEATS: %d\n", heartbeat); 

			//sem_post(&cmdControl);
		}

    //std::cout << "HEATBEATS" << heartbeat << endl;

	// closing the connected socket
	close(new_socket);

	// closing the listening socket
	close(server_fd);

	// End of thread
	return 0;
}