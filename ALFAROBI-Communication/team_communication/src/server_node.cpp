// Server side C/C++ program to demonstrate Socket programming
#include <ros/ros.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <bits/stdc++.h>
#include <sstream>
#define PORT 20000


int main(int argc, char** argv)
{
  ros::init(argc, argv, "server_node");
  ros::NodeHandle nh_;
	int server_fd, new_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[1] = {0};
  	std_msgs::Bool msg;

  ros::Publisher team_comm_pub = nh_.advertise<std_msgs::Bool>("alfarobi/kicking/friend_status",1000);

	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
												&opt, sizeof(opt)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);

	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr *)&address,
								sizeof(address))<0)

	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	if (listen(server_fd, 3) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}
	if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
					(socklen_t*)&addrlen))<0)
	{
		perror("accept");
		exit(EXIT_FAILURE);
	}
  while (ros::ok()) {
    buffer[1] = {0};
  	valread = read( new_socket , buffer, 1);

    if(std::string(buffer) == "1")
    	msg.data = true;
    else
    	msg.data = false;
    team_comm_pub.publish(msg);
  }
	return 0;
}
