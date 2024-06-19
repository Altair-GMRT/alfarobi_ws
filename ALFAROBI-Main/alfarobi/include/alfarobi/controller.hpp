#pragma once

#include <ros/ros.h>

namespace alfarobi{
    class Controller{
        private:
            ros::NodeHandle nh_;
            ros::Publisher pub_;
        public:
            Controller();
            ~Controller();

            void publish_state(int state);
    };
}