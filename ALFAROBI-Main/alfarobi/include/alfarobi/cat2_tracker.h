#ifndef CAT2_TRACKING_H_
#define CAT2_TRACKING_H_

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

// head tracking for looking the goalpost
class Cat2Tracker
{
public:
    enum ScanState
    {
        Keeper = -1,
        Sweep = 0,
        Nod = 1,
        Square = 2,
        Line = 3,
//vertikal line baru
        LineVer = 3
    };

    enum TrackingStatus
    {
        NotFound = -1,
        DontTrack = 0,
        Found = 1,
    };

    Cat2Tracker();
    ~Cat2Tracker();

    int processTracking(int scanState);

    void predictedSquare(bool use);
    void predictedLine(bool use);
//vertikal line baru
    void predictedLineVer(bool use);

    void startTrackingSquare();
    void stopTrackingSquare();

    void startTrackingLine();
    void stopTrackingLine();    

//vertikal line baru
    void startTrackingLineVer();
    void stopTrackingLineVer();

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

    double getPanOfLine()
    {
        // left (+) ~ right (-)
        return current_line_pan_;
    }

    double getTiltOfLine()
    {
        // top (+) ~ bottom (-)
        return current_line_tilt_;
    }

    double getLineSize()
    {
        return current_line_bottom_;
    }    

//vertikal line baru
    double getPanOfLineVer()
    {
        // left (+) ~ right (-)
        return current_line_ver_pan_;
    }

    double getTiltOfLineVer()
    {
        // top (+) ~ bottom (-)
        return current_line_ver_tilt_;
    }

    double getLineVerSize()
    {
        return current_line_ver_bottom_;
    }    

    double squarePoint_x;
    double squarePoint_y;
    double squarePoint_z;

    double linePoint_x;
    double linePoint_y;
    double linePoint_z;

//vertikal line baru
    double linePoint_ver_x;
    double linePoint_ver_y;
    double linePoint_ver_z;

    int soccer_half;
    double robotState_x;
    double robotState_y;
    double robotState_theta;

    double squarePos_x;
    double squarePos_y;
    double squarePos_theta;

    double linePos_x;
    double linePos_y;
    double linePos_theta;

//vertikal line baru
    double linePosVer_x;
    double linePosVer_y;
    double linePosVer_theta;    

protected:
    double FOV_WIDTH;
    double FOV_HEIGHT;
    const bool DEBUG_PRINT;

    void robotStateCallback(const geometry_msgs::PoseStampedConstPtr &_msg);

    void squarePositionCallback(const geometry_msgs::PointConstPtr &_msg);
    void squareProjectedCallback(const geometry_msgs::PointConstPtr &_msg);
    void squareTrackerCommandCallback(const std_msgs::String::ConstPtr &msg);

    void linePositionCallback(const geometry_msgs::PointConstPtr &_msg);
    void lineProjectedCallback(const geometry_msgs::PointConstPtr &_msg);
    void lineTrackerCommandCallback(const std_msgs::String::ConstPtr &msg);

//vertikal line baru
    void lineVerPositionCallback(const geometry_msgs::PointConstPtr &_msg);
    void lineVerProjectedCallback(const geometry_msgs::PointConstPtr &_msg);
    void lineVerTrackerCommandCallback(const std_msgs::String::ConstPtr &msg);

    void presentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);
    void publishHeadJoint(double pan, double tilt);

    void scanSquare(int scanState);
    void scanLine(int scanState);
    void scanLineVer(int scanState);

    //ros node handle
    ros::NodeHandle nh_;

    //image publisher/subscriber
    ros::Publisher module_control_pub_;
    ros::Publisher head_joint_pub_;
    ros::Publisher head_scan_pub_;
    ros::Publisher motion_index_pub_;

    ros::Publisher predicted_line_pub_;
    ros::Publisher predicted_square_pub_;
//vertikal line baru
    ros::Publisher predicted_line_ver_pub_;

    ros::Subscriber robot_state_sub_;

    ros::Subscriber square_position_sub_;
    ros::Subscriber square_tracking_command_sub_;
    ros::Subscriber line_position_sub_;
    ros::Subscriber line_tracking_command_sub_;
//vertikal line baru
    ros::Subscriber line_ver_position_sub_;
    ros::Subscriber line_ver_tracking_command_sub_;

    ros::Subscriber present_joint_states_sub_;
    ros::Subscriber camera_info_sub_;

    ros::Subscriber square_projected_sub_;
    ros::Subscriber line_projected_sub_;
    ros::Subscriber line_ver_projected_sub_;
    // (x, y) is the center position of the goalpost in image coordinates
    // z is the goalpost delta to goal
    geometry_msgs::Point square_position_;
    geometry_msgs::Point line_position_;
    geometry_msgs::Point line_ver_position_;

    sensor_msgs::CameraInfo camera_info_;

    double head_pan_, head_tilt_;
    
    int tracking_status_;
    bool use_head_scan_;
    bool on_tracking_;

    double current_square_pan_, current_square_tilt_;
    double current_square_bottom_;

    double current_line_pan_, current_line_tilt_;
    double current_line_bottom_;    

//vertikal line baru
    double current_line_ver_pan_, current_line_ver_tilt_;
    double current_line_ver_bottom_;
    
    bool penalty;

    double p_pan, p_tilt, d_pan, d_tilt, pandir, tiltdir;
};
}

#endif
