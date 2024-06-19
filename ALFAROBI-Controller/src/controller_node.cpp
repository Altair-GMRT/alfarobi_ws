#include <ros/ros.h>
#include <ncurses.h>
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ALFAROBI-CONTROLLER");

    alfarobi::Controller controller;

    initscr();
    noecho();

    char last = '0';

    while(true)[
        char ch = getch();
        
        if(last == ch) continue;

        switch(ch){
            case 'w':
                controller.publish_state(1);
                break;
            case 'a':
                controller.publish_state(2);
                break;
            case 'd':
                controller.publish_state(3);
                break;
            default:
                break;
        }

        last = ch;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ]
    endwin();

    return 0;
}
