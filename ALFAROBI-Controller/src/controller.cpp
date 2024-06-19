#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "controller.hpp"

namespace alfarobi{

    Controller::Controller(){
        pub_ = nh_.advertise<std_msgs::UInt8>("/alfarobi/controller-state", 1000);
    }

    Controller::~Controller(){}

    void Controller::publish_state(unsigned int state){
        pub_.publish(state);
    }

}