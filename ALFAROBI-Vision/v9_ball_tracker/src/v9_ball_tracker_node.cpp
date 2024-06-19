#include <v9_ball_tracker/v9_ball_tracker.h>

int main(int argc, char **argv){

    ros::init(argc,argv,"v9_ball_tracker_node");

    BallTracker ball_tracker;

    ros::Rate loop_rate(30);

    while(ros::ok()){
        ros::spinOnce();
        ball_tracker.execute();
        loop_rate.sleep();
    }

    return 0;
}

