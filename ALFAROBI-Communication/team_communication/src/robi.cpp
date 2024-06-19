//C LIBRARIES
#include <cstdio>
#include <cstdlib>
#include <cstring>

//C++ LIBRARIES
#include <iostream>

//SOCKET PROGRAMMING LIBRARIES
#include "communication.h"



int main(int argc, char** argv){

    //SELECT ROLE AND MESSAGES
    int ID = ROBI_ID;

    // ROS INITIALIZATION
    ros::init(argc, argv, "team_communication_from_robi");
    ros::Time::init();
    ros::Rate loop_rate(30);
    topicUpdater tu = topicUpdater(ID);

    //SOCKET
    clientSocket commSocket = clientSocket(ID, PORT_2);
    commSocket.init();

    ROS_WARN("Requseting a connection to ID = ABI .....");
    commSocket.tryConnect();

    ROS_WARN("Connection established successfully.");

    //KOMUNIKASI FSM
    while(ros::ok()){

        commSocket.transmitMsg(tu.robi_ball_dist);
        commSocket.receiveMsg();

        switch(commSocket.buffer[0]){
            case 'A':
                tu.abi_status = true;
                tu.alfa_status = false;
                tu.robi_status = false;
                break;
            case 'B':
                tu.abi_status = false;
                tu.alfa_status = true;
                tu.robi_status = false;
                break;
            case 'C':
                tu.abi_status = false;
                tu.alfa_status = false;
                tu.robi_status = true;
                break;
        }

        commSocket.clearBuffer();

        commSocket.sendReset(tu.is_reset);
        tu.is_reset = false;

        commSocket.receiveMsg();

        if (commSocket.buffer[0] == 'R'){
            tu.abi_status = false;
            tu.alfa_status = false;
            tu.robi_status = false;
        }

        commSocket.clearBuffer();

        tu.updateState();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
