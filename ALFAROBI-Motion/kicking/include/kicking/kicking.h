#ifndef KICKING_H_
#define KICKING_H_

#include <iostream>
#include <fstream>
#include <typeinfo>
#include <string>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <kdl_conversions/kdl_msg.h>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "walk_zmp/kinematics.h"
#include "walk_zmp/zmp_team_darwin_kinematics.hpp"

#include "kicking_module_msgs/KickingParam.h"
#include "kicking_module_msgs/GetKickingParam.h"
#include "kicking_module_msgs/SetKickingParam.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>

#define PI 3.14159265

using namespace Eigen;

namespace robotis_op
{

    class Kicking : public robotis_framework::MotionModule, public robotis_framework::Singleton<Kicking>
    {
    public:
        Kicking();
        ~Kicking();

        void start();
        void stop();
        bool isRunning();
        void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
        void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
        void onModuleDisable();
        void onModuleEnable();

        void queueThread();
        void kickingCommandCallback(const std_msgs::String::ConstPtr &_msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void velocityAngleCallback(const geometry_msgs::PointConstPtr &_msg);

        kicking_module_msgs::KickingParam kicking_param_;
        void kickingParamCallback(const kicking_module_msgs::KickingParam::ConstPtr &msg);
        bool getKickingParamCallback(kicking_module_msgs::GetKickingParam::Request &req, kicking_module_msgs::GetKickingParam::Response &res);

        void loadConfig();
        void loadKickManip();
        void saveConfig(const std::string &path);
        void updateKickingParam();

        //Parameter Kicking
        bool initPose;
        Eigen::Vector3d L_Shift_XYZ, R_Shift_XYZ, Ankle_Shift_RPY;
        Eigen::Vector3d L_Lift_XYZ,  R_Lift_XYZ, L_Lift_RPY, R_Lift_RPY;
        Eigen::Vector3d L_Swing_XYZ, R_Swing_XYZ, L_Swing_RPY, R_Swing_RPY;
        Eigen::Vector3d L_Retract_XYZ, R_Retract_XYZ, L_Retract_RPY, R_Retract_RPY;
        //Parameter Time
        double m_state_time[6];

        //Raw Kick Parameter
        double iTorso_X, iTorso_Y, iTorso_Z; //Init Torso
        double L_Shift_X, L_Shift_Y, L_Shift_Z; //Shift Torso
        double R_Shift_X, R_Shift_Y, R_Shift_Z; //Shift Torso

        double L_Lift_X, L_Lift_Y, L_Lift_Z; //Lift Legs
        double R_Lift_X, R_Lift_Y, R_Lift_Z; //Lift Legs

        double L_Swing_X, L_Swing_Y, L_Swing_Z; //Swing Legs
        double R_Swing_X, R_Swing_Y, R_Swing_Z; //Swing Legs

        double L_Retract_X, L_Retract_Y, L_Retract_Z; //Swing Legs
        double R_Retract_X, R_Retract_Y, R_Retract_Z; //Swing Legs

        double iTorso_Pitch;
        double Shift_Roll;
        double Lift_Roll, Lift_Pitch; //Torso
        double Swing_Roll, Swing_Pitch; //Torso
        double Retract_Roll, Retract_Pitch; //Torso

    private:
        enum KickState
        {
            Kick_Start_Shift = 0,
            Kick_Lift = 1,
            Kick_Swing = 2,
            Kick_Retract = 3,
            Kick_Landing = 4,
            Kick_Finish_Shift = 5,
            Kick_Finished = 6,
        };

        enum
        {
            KickingDisable = 0,
            KickingEnable = 1,
            KickingInitPose = 2,
            KickingReady = 3,
        };
        enum
        {
            Right_Kick = 1,
            Left_Kick = 2,
        };

        robotis_op::OP3KinematicsDynamics *op3_kd_;
        int control_cycle_msec_;
        std::string config_path_;
        boost::thread queue_thread_;
        bool real_running_;
        int kicking_state_;

        ros::NodeHandle nh_;
        ros::Subscriber kicking_command_sub_;
        ros::Subscriber kicking_param_sub_;
        ros::Subscriber get_kicking_param_server;

        ros::Subscriber angle_sub_;
        ros::Subscriber angular_velocity_sub_;
        /* IMU (Inertial Measurement Unit) */
        Vector3d imu;
        Quaterniond imu_orientation;
        Eigen::Vector3d m_imuGyr;
        Eigen::Vector3d m_imuOri;
        Eigen::Vector3d m_imuAcc;
        Eigen::MatrixXd rotIMU;

        //ROS Publisher and Subscriber
        ros::Publisher kick_status_pub_;
        void kickStatus(const std::string &command);

        Eigen::MatrixXd target_position_;
        Eigen::MatrixXd current_position_;
        Eigen::MatrixXd init_position_;
        Eigen::MatrixXi joint_axis_direction_;
        std::map<std::string, int> joint_table_;

        bool firstExc;
        double firstTimeExc;
        double timeExc;

        int init_pose_count_;
        KDL::JntArray current_joint_pos;
        Eigen::MatrixXd calc_joint_tra_;

        std::string op3_joint[20], joint[18], offset_path_;
        double offset[18], op3_offset[20];

        bool m_active;
        bool m_start;
        int m_kick_mode;
        int m_kick_state;
        double m_time_start;
        double m_time_now;
        double m_time_state[6];
        double phase;

        double ankleShift_roll;
        double ankleShift_pitch;
        double kneeShift_pitch;
        double hipShift_pitch;
        double ankleShift[2];
        double kneeShift;
        double hipShift;
        double ankleImuParamRoll[4];
        double ankleImuParamPitch[4];
        double kneeImuParamPitch[4];
        double hipImuParamPitch[4];
        double gyroFactor;

        Kinematics k;

        Matrix<double, 18, 1> getJoints;
        Eigen::Vector3d actual_torso;
        Eigen::Matrix<double, 6, 1> L_Leg, R_Leg, Torso;
        Eigen::Matrix<double, 6, 1> uL_Leg, uR_Leg, uTorso;

        Eigen::Affine3d BASE;
        Eigen::Affine3d R_HIP, R_KNEE, R_ANKLE, R_FOOT, m_R_FOOT;
        Eigen::Affine3d L_HIP, L_KNEE, L_ANKLE, L_FOOT, m_L_FOOT;
        Eigen::Vector3d L_FOOT_WORLD, R_FOOT_WORLD;
        Eigen::Vector3d L_ROT_WORLD, R_ROT_WORLD;


        // Eigen::VectorXd L_FOOT_WORLD, R_FOOT_WORLD;
        Eigen::VectorXd LFootWorld, RFootWorld;
        int IndexSupport;

        void setValue();

        double procFunc(double value, double deadband, double maxValue);
        void legStabilization();
        void estimateZMP();
        Eigen::Vector3d bezierCurve2(double phase, Eigen::Vector3d start, Eigen::Vector3d end);
        Eigen::Vector3d bezierCurve3(double phase, Eigen::Vector3d start, Eigen::Vector3d inter, Eigen::Vector3d end);
        void motion_arms();
        void forwardKinematic();
    };

} // namespace robotis_op

#endif