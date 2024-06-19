#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "std_msgs/String.h"

#define MESSAGE_FREQ 10
#define PORT 69420

void error(const char *msg) {
    perror(msg);
    exit(0);
}

class Listener {
private:
    char topic_message[1] = { 0 };
public:
    void callback(const std_msgs::String::ConstPtr& msg);
    char* getMessageValue();
};

void Listener::callback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "finished_kick"){
        ROS_INFO("BABI");
        memset(topic_message, 0, 1);
        strcpy(topic_message, "1");
    }else if(msg->data == "send"){
        ROS_INFO("BABI");
        memset(topic_message, 0, 1);
        strcpy(topic_message, "1");
    }else{
        ROS_INFO("JMBT INI ELSE");
        memset(topic_message, 0, 1);
        strcpy(topic_message, "0");
    }
}

char* Listener::getMessageValue() {
    return topic_message;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(MESSAGE_FREQ); // set the rate as defined in the macro MESSAGE_FREQ
    Listener listener;
    ros::Subscriber client_sub = nh.subscribe("/alfarobi/kicking/status", 1, &Listener::callback, &listener);
    int sockfd, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[1];

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname("192.168.0.2");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(PORT);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    while(ros::ok()) {
        buffer[1] = {0};
        bzero(buffer,1);
        strcpy(buffer, listener.getMessageValue());
        loop_rate.sleep();
        
        printf("%s\n", buffer);

        n = write(sockfd,buffer,strlen(buffer));
        if (n < 0) 
             error("ERROR writing to socket");

        ros::spinOnce();
    }
    return 0;
}