#include <ros/ros.h>
#include <ncurses.h>
#include <thread>
#include <chrono>
#include "alfarobi/controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");

    alfarobi::Controller controller;

        initscr();
        cbreak();
        noecho();

    char last = '0';

    while(true){
        char ch = getch();
        
        if(last == ch) continue;

        switch(ch){
            case 'w': // forward
                controller.publish_state(1);
                break;
            case 'a': // shift left
                controller.publish_state(2);
                break;
            case 'd': // shift right
                controller.publish_state(3);
                break;
            case 'q': // rotate ccw
                controller.publish_state(4);
                break;
            case 'e': // rotate cw
                controller.publish_state(5);
                break;
            case 'z': // revo ccw
                controller.publish_state(6);
                break;
            case 'c': // revo cw
                controller.publish_state(7);
                break;
            case 's': // kick
                controller.publish_state(8);
                break;
            default:
                break;
        }

        last = ch;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    endwin();



    return 0;
}