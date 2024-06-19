//ROS DEPENDENCIES
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"

//C LIBRARIES
#include <cstdio>
#include <cstdlib>
#include <cstring>

//C++ LIBRARIES
#include <iostream>

//SOCKET PROGRAMMING LIBRARIES
#include "communication.h"
#define MASTER_ID 1
#define CLIENT_ID 2
#define PORT 12124

//STATE DI SINI
/*
    ABI     =   STANDBY ->  FOLLOW AND SHOOT -> DONE SHOOT
    ALFA    =   STANDBY ->  KICKING          ->  DONE KICKING
*/


int main(int argc, char** argv){

    //SELECT ROLE AND MESSAGES
    int ID = MASTER_ID;
    char msgA[5] = "A000";
    char msgB[5] = "B000";
    std_msgs::Bool msg;

    // ROS INITIALIZATION
    ros::init(argc, argv, "communication_socket");
    ros::Time::init();
    ros::Rate loop_rate(10);
    topicUpdater tu = topicUpdater();

    //SOCKET
    serverSocket commSocket = serverSocket(ID);
    commSocket.init();

    //INISIALISASI KONEKSI
    commSocket.receiveMsg();
    if(commSocket.buffer[0] == 'A')
        ROS_INFO(" Terhubung dengan Alfa!");

    //KOMUNIKASI FSM
    while(ros::ok()){

        switch(tu.comm_status){

            case 0:
                tu.publish(0);
                commSocket.transmitMsg(msgA);
                ROS_INFO(" Alfa start pass!");
                tu.publish(1);
                break;

            case 1:
                commSocket.receiveMsg();
                if(commSocket.buffer[0] == 'B'){
                    tu.publish(2);
                    ROS_INFO(" Alfa sudah passing!");
                    msg.data = true;
                    tu.friend_status.publish(msg);
                }
                break;

            case 2:
                commSocket.transmitMsg(msgB);
                ROS_INFO(" Abi meminta repeat!");
                tu.publish(0);
                break;

            default:
                ROS_WARN(" Waiting for incoming buffer...");
                break;

            // case 0:
            //     tu.publish(0);
            //     commSocket.receiveMsg();
            //     if(commSocket.buffer[0] == 'A'){
            //         tu.publish(1);
            //         ROS_INFO(" Alfa start pass!");
            //     }
            //     break;

            // case 1:
            //     commSocket.transmitMsg(msgB);
            //     ROS_INFO(" Alfa sudah passing!");
            //     commSocket.receiveMsg();
            //     if(commSocket.buffer[0] == 'B'){
            //         tu.publish(2);
            //         ROS_INFO(" Abi meminta repeat!");
            //     }
            //     break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}