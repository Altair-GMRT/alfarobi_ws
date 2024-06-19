#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "alfarobi/controller.hpp"

namespace alfarobi{

    Controller::Controller(){
        pub_ = nh_.advertise<std_msgs::UInt8>("/alfarobi/controller_state", 1000);
    }

    Controller::~Controller(){}

    void Controller::publish_state(int state){
        std_msgs::UInt8 msg;
        msg.data = state;
        pub_.publish(msg);
    }

}