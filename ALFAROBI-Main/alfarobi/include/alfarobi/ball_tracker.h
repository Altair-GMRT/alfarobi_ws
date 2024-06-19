#ifndef BALL_TRACKING_H_
#define BALL_TRACKING_H_

#include <math.h>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "robotis_math/robotis_math.h"
#include "alfarobi.h"


#define PI 3.14159265

namespace robotis_op
{

// head tracking for looking the ball
class BallTracker
{
public:
    enum ScanState
    {
        Keeper = -1,
        Sweep = 0,
        Nod = 1,
        Square = 2,
        Line = 3,
        Ndungkluk = 4
    };

    enum TrackingStatus
    {
        NotFound = -1,
        DontTrack = 0,
        Found = 1,
    };

    BallTracker();
    ~BallTracker();

    int processTracking(int scanState);

    void predictedBall(bool use);

    void startTracking();
    void stopTracking();

    void setUsingHeadScan(bool use_scan);

    double getPanOfBall()
    {
        // left (+) ~ right (-)
        return current_ball_pan_;
    }

    double getTiltOfBall()
    {
        // top (+) ~ bottom (-)
        return current_ball_tilt_;
    }

    double getBallSize()
    {
        return current_ball_bottom_;
    }

    double ballPoint_x;
    double ballPoint_y;
    double ballPoint_z;

    int soccer_half;
    double robotState_x;
    double robotState_y;
    double robotState_theta;

    double ballPos_x;
    double ballPos_y;
    double ballPos_theta;
    float ball_dist;

protected:
    double FOV_WIDTH;
    double FOV_HEIGHT;
    const bool DEBUG_PRINT;

    void robotStateCallback(const geometry_msgs::PoseStampedConstPtr &_msg);
    void ballPositionCallback(const geometry_msgs::PointConstPtr &_msg);
    void ballProjectedCallback(const geometry_msgs::PointConstPtr &_msg);
    void ballTrackerCommandCallback(const std_msgs::String::ConstPtr &msg);
    void presentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);
    void publishHeadJoint(double pan, double tilt);
    void scanBall(int scanState);

    //ros node handle
    ros::NodeHandle nh_;

    //image publisher/subscriber
    ros::Publisher module_control_pub_;
    ros::Publisher head_joint_pub_;
    ros::Publisher head_scan_pub_;
    ros::Publisher motion_index_pub_;
    ros::Publisher predicted_ball_pub_;
    ros::Publisher phytagorean_dist_pub_;

    ros::Subscriber robot_state_sub_;
    ros::Subscriber ball_position_sub_;
    ros::Subscriber ball_tracking_command_sub_;
    ros::Subscriber present_joint_states_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber ball_projected_sub_;

    // (x, y) is the center position of the ball in image coordinates
    // z is the ball delta to goal
    geometry_msgs::Point ball_position_;
    sensor_msgs::CameraInfo camera_info_;

    double head_pan_, head_tilt_;
    
    int tracking_status_;
    bool use_head_scan_;
    bool on_tracking_;
    double current_ball_pan_, current_ball_tilt_;
    double current_ball_bottom_;

    bool penalty;

    double p_pan, p_tilt, d_pan, d_tilt, pandir, tiltdir;
};
}

#endif
