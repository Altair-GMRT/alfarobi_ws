#ifndef BALLTRACKER_H
#define BALLTRACKER_H

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>

#include <robotis_controller_msgs/JointCtrlModule.h>
#include <robotis_controller_msgs/SetJointModule.h>

class BallTracker{
public:
    BallTracker();
    ~BallTracker();
    void execute();
private:
    ros::NodeHandle nh_;

    ros::Publisher head_joint_pub_;
    sensor_msgs::JointState head_joint_;

    ros::Subscriber ball_pos_sub_;
    geometry_msgs::Point ball_pos_;
    geometry_msgs::Point last_ball_pos_;
    void ballPosCallback(const geometry_msgs::PointConstPtr &_msg);

    ros::Publisher head_scan_pub_;

    void setHeadCtrModule();
    ros::ServiceClient set_module_srv_;

    geometry_msgs::Point center_frame_;

    bool ball_found_;

    double p_gain;
    double d_gain;
    double i_gain;

    const int HOLD_THRESHOLD;

    const double H_FOV;
    const double V_FOV;

    void searchBall();
    void setHeadJoint(double _pan, double _tilt);

};

#endif
