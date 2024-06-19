#ifndef SQUARE_TRACKING_H_
#define SQUARE_TRACKING_H_

#include <math.h>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
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

// head tracking for looking the Square
class SquareTracker
{
public:
    enum ScanState
    {
        Keeper = -1,
        Sweep = 0,
        Nod = 1,
        Square = 2,
        Line = 3
    };

    enum TrackingStatus
    {
        NotFound = -1,
        DontTrack = 0,
        Found = 1,
    };

    SquareTracker();
    ~SquareTracker();

    int processTracking(int scanState);

    void predictedSquare(bool use);

    void startTracking();
    void stopTracking();

    void setUsingHeadScan(bool use_scan);

    double getPanOfSquare()
    {
        // left (+) ~ right (-)
        return current_square_pan_;
    }

    double getTiltOfSquare()
    {
        // top (+) ~ bottom (-)
        return current_square_tilt_;
    }

    double getSquareSize()
    {
        return current_square_bottom_;
    }

    double squarePoint_x;
    double squarePoint_y;
    double squarePoint_z;

    int soccer_half;
    double robotState_x;
    double robotState_y;
    double robotState_theta;

    double squarePos_x;
    double squarePos_y;
    double squarePos_theta;

protected:
    double FOV_WIDTH;
    double FOV_HEIGHT;
    const bool DEBUG_PRINT;

    void robotStateCallback(const geometry_msgs::PoseStampedConstPtr &_msg);
    void squarePositionCallback(const geometry_msgs::PointConstPtr &_msg);
    void squareProjectedCallback(const geometry_msgs::PointConstPtr &_msg);
    void squareTrackerCommandCallback(const std_msgs::String::ConstPtr &msg);
    void presentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);
    void publishHeadJoint(double pan, double tilt);
    void scanSquare(int scanState);

    //ros node handle
    ros::NodeHandle nh_;

    //image publisher/subscriber
    ros::Publisher module_control_pub_;
    ros::Publisher head_joint_pub_;
    ros::Publisher head_scan_pub_;
    ros::Publisher motion_index_pub_;
    ros::Publisher predicted_square_pub_;

    ros::Subscriber robot_state_sub_;
    ros::Subscriber square_position_sub_;
    ros::Subscriber square_tracking_command_sub_;
    ros::Subscriber present_joint_states_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber square_projected_sub_;

    // (x, y) is the center position of the square in image coordinates
    // z is the square delta to goal
    geometry_msgs::Point square_position_;
    sensor_msgs::CameraInfo camera_info_;

    double head_pan_, head_tilt_;
    
    int tracking_status_;
    bool use_head_scan_;
    bool on_tracking_;
    double current_square_pan_, current_square_tilt_;
    double current_square_bottom_;

    bool penalty;

    double p_pan, p_tilt, d_pan, d_tilt, pandir, tiltdir;
};
}

#endif
