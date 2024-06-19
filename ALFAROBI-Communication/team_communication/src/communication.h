#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"

#include <iostream>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT_1 10569
#define PORT_2 11569

#define ABI_IP "192.168.0.93"
#define ALFA_IP "192.168.0.92"
#define ROBI_IP "192.168.0.94"

#define ABI_ID 1
#define ALFA_ID 2
#define ROBI_ID 3



class topicUpdater{
private:

    // ROS INITIALIZATION
    ros::NodeHandle nh;
    
    ros::Publisher my_topic;
    ros::Publisher msg_sent_pub_;
    ros::Publisher robi_dist_pub_;
    ros::Publisher robi_comp_pub_;
    ros::Publisher alfa_comp_pub_;
    ros::Publisher comm_reset_pub_;

    ros::Subscriber dist_topic;
    ros::Subscriber robi_dist_sub_;
    ros::Subscriber robi_comp_sub_;
    ros::Subscriber alfa_comp_sub_;
    ros::Subscriber msg_sent_sub_;
    ros::Subscriber comm_reset_sub_;

    // ROBOT ID
    int my_id;

public:

    // VARIABLES
    bool abi_status = false;
    bool alfa_status = false;
    bool robi_status = false;
    bool is_reset = false;

    float abi_ball_dist = 0;
    float alfa_ball_dist = 0;
    float robi_ball_dist = 0;

    bool alfa_comp = false;
    bool robi_comp = false;

    short sentMsg = 0;

    // CALLBACK METHOD
    void distTopicCallBack(const std_msgs::Float32::ConstPtr& msg){
        switch(my_id){
            case ABI_ID:
                abi_ball_dist = msg->data;
                break;
            case ALFA_ID:
                alfa_ball_dist = msg->data;
                break;
            case ROBI_ID:
                robi_ball_dist = msg->data;
                break;
        }
    }

    void msgSentCallBack(const std_msgs::Int8::ConstPtr& msg){
        sentMsg = msg->data;
    }

    void commResetCallBack(const std_msgs::Bool::ConstPtr& msg){
        is_reset = msg->data; 
    }

    void robiDistCallBack(const std_msgs::Float32::ConstPtr& msg){
        robi_ball_dist = msg->data;
    }

    void robiCompCallBack(const std_msgs::Bool::ConstPtr& msg){
        robi_comp = msg->data;
    }

    void alfaCompCallBack(const std_msgs::Bool::ConstPtr& msg){
        alfa_comp = msg->data;
    }


    // PUBLISH METHOD
    void selfPublish(){
        std_msgs::Int8 msg;
        msg.data = sentMsg;
        msg_sent_pub_.publish(msg);
        
        std::cout << "\n----------- S T A T U S -----------" << std::endl;
        std::cout << "[ ABI STATUS  ]\t: " << abi_status  << std::endl;
        std::cout << "[ ALFA STATUS ]\t: " << alfa_status << std::endl;
        std::cout << "[ ROBI STATUS ]\t: " << robi_status << std::endl;
    }

    void robiPublish(std_msgs::Float32 msg){
        robi_dist_pub_.publish(msg);
    }

    void waitSync(){
        while(!alfa_comp || !robi_comp){}
    }

    void alfaCommDone(){
        alfa_comp = true;
        std_msgs::Bool msg;
        msg.data = true;
        alfa_comp_pub_.publish(msg);
    }

    void alfaCommReset(){
        alfa_comp = false;
        std_msgs::Bool msg;
        msg.data = false;
        alfa_comp_pub_.publish(msg);
    }

    void robiCommDone(){
        robi_comp = true;
        std_msgs::Bool msg;
        msg.data = false;
        robi_comp_pub_.publish(msg);
    }

    void robiCommReset(){
        robi_comp = false;
        std_msgs::Bool msg;
        msg.data = false;
        robi_comp_pub_.publish(msg);
    }

    void updateState(){
        std_msgs::Bool msg;

        switch(my_id){
            case ABI_ID:
                msg.data = abi_status;
                break;
            case ALFA_ID:
                msg.data = alfa_status;
                break;
            case ROBI_ID:
                msg.data = robi_status;
                break;
        }

        my_topic.publish(msg);
    }

    void resetAllState(){
        std_msgs::Bool msg;
        msg.data = false;
        comm_reset_pub_.publish(msg);
    }

    // PROCESS THE BALL DISTANCES
    void processDists(){
        if (!abi_status && !alfa_status && !robi_status && my_id == ABI_ID){
            
            int closest_to_ball = 0;
            
            if(abi_ball_dist < alfa_ball_dist){
                closest_to_ball = ABI_ID;
                if(robi_ball_dist < abi_ball_dist){
                    closest_to_ball = ROBI_ID;
                }
            }

            else{
                closest_to_ball = ALFA_ID;
                if(robi_ball_dist < alfa_ball_dist){
                    closest_to_ball = ROBI_ID;
                }
            }

            switch(closest_to_ball){
                case ABI_ID:
                    abi_status = true;
                    sentMsg = ABI_ID;
                    break;
                case ALFA_ID:
                    alfa_status = true;
                    sentMsg = ALFA_ID;
                    break;
                case ROBI_ID:
                    robi_status = true;
                    sentMsg = ROBI_ID;
                    break;
            }

        }

        else if (is_reset){
            abi_status = false;
            alfa_status = false;
            robi_status = false;
            is_reset = false;
        }

    }

    // CONSTRUCTOR
    topicUpdater(int ID){
        my_id = ID;

        my_topic = nh.advertise<std_msgs::Bool>("/alfarobi/go_to_ball", 1);
        dist_topic = nh.subscribe<std_msgs::Float32>("/alfarobi/ball_dist", 10, &topicUpdater::distTopicCallBack, this);
        comm_reset_sub_ = nh.subscribe<std_msgs::Bool>("/alfarobi/comm_reset", 1, &topicUpdater::commResetCallBack, this);
        comm_reset_pub_ = nh.advertise<std_msgs::Bool>("/alfarobi/comm_reset", 1);
        
        if (my_id == ABI_ID){
            
            robi_dist_sub_ = nh.subscribe<std_msgs::Float32>("/robi_dist", 10, &topicUpdater::robiDistCallBack, this);
            robi_dist_pub_ = nh.advertise<std_msgs::Float32>("/robi_dist", 10);
            robi_comp_sub_ = nh.subscribe<std_msgs::Bool>("/robi_comp", 1, &topicUpdater::robiCompCallBack, this);
            robi_comp_pub_ = nh.advertise<std_msgs::Bool>("/robi_comp", 1);

            alfa_comp_sub_ = nh.subscribe<std_msgs::Bool>("/alfa_comp", 1, &topicUpdater::alfaCompCallBack, this);
            alfa_comp_pub_ = nh.advertise<std_msgs::Bool>("/alfa_comp", 1);

            msg_sent_sub_ = nh.subscribe<std_msgs::Int8>("/msg_sent", 1, &topicUpdater::msgSentCallBack, this);
            msg_sent_pub_ = nh.advertise<std_msgs::Int8>("/msg_sent", 1);
        }
        
    }
};




class clientSocket{
public:


    // VARIABLES
    int sock = 0, valread, client_fd, ID, PORT;
	struct sockaddr_in serv_addr;
    char buffer[64] = {0};


    // CONSTRUCTOR
    clientSocket(int ID, int PORT){
        this->ID = ID;
        this->PORT = PORT;
    }


    // CONNECTION INITIALIZATION
    int init(){

        int tryCount = 0;

        // CREATING A SOCKET
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0){
            printf("\n[INFO] GAGAL MEMBUAT SOCKET\n");
            return -1;
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);

        // CONVERT IPv4 AND IPv6 ADDRESS FROM STRING TO BINARY
        if (inet_pton(AF_INET, ABI_IP, &serv_addr.sin_addr) <= 0){
            printf("\n[INFO] ALAMAT IPv4/IPv6 TIDAK VALID\n");
            return -1;
        }

        // TRY TO CONNECT THE SOCKET SEVERAL TIMES
        while ((client_fd = connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0){
            ++tryCount;
            if(tryCount > 10){
                printf("\n[INFO] KONEKSI GAGAL\n");
                return -1;
            }
        }

        printf("\n[INFO] BERHASIL TERSAMBUNG KE MASTER (ABI)\n");

        return 1;
    }


    // TRANSIMITING A MESSAGE
    void transmitMsg(float x){

        // FLOAT TO CHAR ARRAY
        char msg[64];
        sprintf(msg, "%f", x);
        
        // SEND THE MESSAGE
        send(sock, msg, 64, 0);

    }

    void tryConnect(){
        char msg[64] = {0};
        msg[0] = 'Y';
        send(sock, msg, 64, 0);
    }

    void sendReset(bool s){
        char msg[64] = {0};
        if (s)
            msg[0] = 'R';
        else
            msg[0] = 'H';
        send(sock, msg, 64, 0);
    }


    // RECEIVING INCOMING MESSAGE
    void receiveMsg(){

        // WAITING FOR INCOMING MESSAGE
        valread = read(sock, buffer, 64);

    }

    void clearBuffer(){
        memset(buffer, 0, 64);
    }

    // FOR CLOSING THE SOCKET
    void shutdownSocket(){
        close(client_fd);
    }
    
};




class serverSocket{
public:

    // VARIABLES
    int server_fd, new_socket, valread, ID, PORT;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[64] = {0};


    // CONSTRUCTOR
    serverSocket(int ID, int PORT){
        this->ID = ID;
        this->PORT = PORT;
    }


    // CONNECTION INITIALIZATION
    int init(){

        // CREATING A SOCKET
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // FORCE CONNECT TO PORT
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))){
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(PORT);

        // FORCE ATTACH TO PORT
        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        if (listen(server_fd, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }

        if ((new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            exit(EXIT_FAILURE);
        }

        ROS_INFO("Berhasil menginisiasi socket server.");

        return 1;
    }


    // TRANSMIT MESSAGE
    void transmitMsg(int ID){

        char msg[64] = {0};

        switch(ID){
            case ABI_ID:
                msg[0] = 'A';
                break;
            case ALFA_ID:
                msg[0] = 'B';
                break;
            case ROBI_ID:
                msg[0] = 'C';
                break;
            case 69:
                msg[0] = 'R';
                break;
            default:
                msg[0] = 'O';
                break;
        }

        // SENDING THE MESSAGE
        send(new_socket, msg, 64, 0);

    }


    // RECEIVE THE INCOMING MESSAGE 
    void receiveMsg(){

        // WAIT AND READ THE INCOMING MESSAGE
        valread = read(new_socket, buffer, 64);

    }

    // CLEARING THE BUFFER
    void clearBuffer(){
        memset(buffer, 0, 64);
    }

    // FOR CLOSING THE SOCKET
    void shutdownSocket(){
        close(new_socket);
        shutdown(server_fd, SHUT_RDWR);
    }

};


#endif
