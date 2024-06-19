#include "v9_ball_tracker/v9_ball_tracker.h"

BallTracker::BallTracker():
    nh_(ros::this_node::getName()),
    H_FOV(61.25),V_FOV(41.60),
    HOLD_THRESHOLD(10){

    head_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states_offset",1);

    ball_pos_sub_ = nh_.subscribe("/v9_ball_detector_node/ball_pos",1,&BallTracker::ballPosCallback,this);

    head_scan_pub_ = nh_.advertise<std_msgs::String>("/robotis/head_control/scan_command", 10);

    set_module_srv_ = nh_.serviceClient<robotis_controller_msgs::SetJointModule>("/robotis/set_present_joint_ctrl_modules");


//PID ??????????????????????????????????????
    nh_.param<double>("p_gain",p_gain,0.5);
    nh_.param<double>("d_gain",d_gain,0.05);

    setHeadCtrModule();

    center_frame_.x = 320;
    center_frame_.y = 240;

    ball_found_ = false;
}

BallTracker::~BallTracker(){

}

void BallTracker::ballPosCallback(const geometry_msgs::PointConstPtr &_msg){
    if(ball_pos_.x != -1 && ball_pos_.y != -1){
        ball_pos_.x = _msg->x;
        ball_pos_.y = _msg->y;
        ball_pos_.z = _msg->z;
        ball_found_ = true;
    }else{
        ball_found_ - false;
    }
}

void BallTracker::setHeadCtrModule(){
    robotis_controller_msgs::JointCtrlModule control_msg;

    control_msg.joint_name.push_back("head_pan");
    control_msg.module_name.push_back("head_control_module");
    control_msg.joint_name.push_back("head_tilt");
    control_msg.module_name.push_back("head_control_module");

    robotis_controller_msgs::SetJointModule set_module_req;
    set_module_req.request.joint_name = control_msg.joint_name;
    set_module_req.request.module_name = control_msg.module_name;

    set_module_srv_.call(set_module_req);

}

void BallTracker::setHeadJoint(double _pan, double _tilt){
    head_joint_.name.push_back("head_pan");
    head_joint_.name.push_back("head_tilt");

    head_joint_.position.push_back(_pan);
    head_joint_.position.push_back(_tilt);
    head_joint_pub_.publish(head_joint_);
}

void BallTracker::execute(){

    static int lost_count = 0;

    if(!ball_found_){

        if(lost_count < HOLD_THRESHOLD){
            lost_count++;
        }else{
            std_msgs::String scan_msg;
            scan_msg.data = "scan";
            head_scan_pub_.publish(scan_msg);
        }
        return;
    }

    lost_count = 0;

    static double last_error_x = 0,last_error_y = 0;

    double error_x = center_frame_.x - ball_pos_.x;
    double error_y = center_frame_.y - ball_pos_.y;

    double offset_x = p_gain*error_x + d_gain*(error_x - last_error_x);
    double offset_y = p_gain*error_y + d_gain*(error_y - last_error_y);

    last_error_x = error_x;
    last_error_y = error_y;

    setHeadJoint(offset_x * (H_FOV)/(2*center_frame_.x),offset_y * (V_FOV)/(2*center_frame_.y));
    ball_found_ = false;

}
