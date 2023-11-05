// Client side C/C++ program to demonstrate Socket
// programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <unistd.h>

#include <bits/stdc++.h>
#include <fstream>
#include <string>
using namespace std;

// VARIABLES 
// For reading the file
fstream myfile;
#define PORT 3390

using namespace std;

// Opening the input.txt file
void openFile() {

  // Opening file
  string filename = "input.CEF";

  // Makes each element into an indexed value
  myfile.open(filename.c_str());

  // Error handling when opening file
  // Exits out of program if file-stream fails
  if (!myfile.is_open()) {
    cout << "File not opened correctly...\n" << endl;
    exit(-1);
  };
}

// Closing the input.txt file
void closeFile() {

  // Closing file
  myfile.close();

  // Error file closing
  if (myfile.is_open()) {
    cout << " File not closed correctly.\n";
    exit(-2);
  }
}

int main(int argc, char const* argv[])
{
	openFile();

	int status, valread, client_fd;
	struct sockaddr_in serv_addr;
	//char hello[] = {"UP", "DOWN", "UP", "DOWN"};
    //std::vector<std::string> hello = {"UP", "DOWN", "UP", "DOWN", "UP"};
	char buffer[1024] = { 0 };
	if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("\n Socket creation error \n");
		return -1;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);

	// Convert IPv4 and IPv6 addresses from text to binary
	// form
	if (inet_pton(AF_INET, "10.0.0.223", &serv_addr.sin_addr)
		<= 0) {
		printf(
			"\nInvalid address/ Address not supported \n");
		return -1;
	}

	if ((status
		= connect(client_fd, (struct sockaddr*)&serv_addr,
				sizeof(serv_addr)))
		< 0) {
		printf("\nConnection Failed \n");
		return -1;
	}
	string aLine;
	while(myfile >> aLine){
    //for(int itr = 0; itr < 5; itr++){
        //const char *test = hello[itr].data();
        const char *test = aLine.data();
		cout << aLine << endl;

        //write(client_fd, test, strlen(test));
        send(client_fd, test, strlen(test), 0);
        //printf("Hello message sent: %d\n", itr);
        printf("Command Message Sent\n");

		//usleep(1000);
        valread = read(client_fd, buffer, 1024 - 1); // subtract 1 for the null terminator at the end

        printf("%s\n\n", buffer);

    }

	// closing the connected socket
	close(client_fd);
	closeFile();
	return 0;
}
