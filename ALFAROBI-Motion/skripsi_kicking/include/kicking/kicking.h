#ifndef KICKING_H_
#define KICKING_H_

#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
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
#include "model_based_control/modelBasedControl.h"
#include "model_based_control/servoObserver.h"
#include "model_based_control/densis.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "walk_zmp/kinematics.h"
#include "walk_zmp/zmp_team_darwin_kinematics.hpp"

#include "kicking_module_msgs/KickingParam.h"
#include "kicking_module_msgs/GetKickingParam.h"
#include "kicking_module_msgs/SetKickingParam.h"
#include "data_log_msgs/DataLog.h"
#include "densis_msgs/densis.h"

#include <eigen_conversions/eigen_msg.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <tf/tf.h>

#define PI 3.14159265
#define G_CONST 9.81 //m/s^2
#define WEIGHT 50 //kg*m/s^2 (Newton)
#define MASS 5.5 // kg
#define percentRSK 0.86
#define percentLSK 1.0
#define Time_Roll 0.2

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
    // void imuCallback(const read_data::controller::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg); //(const geometry_msgs::PointConstPtr &_msg);

    kicking_module_msgs::KickingParam kicking_param_;
    void kickingParamCallback(const kicking_module_msgs::KickingParam::ConstPtr &msg);
    bool getKickingParamCallback(kicking_module_msgs::GetKickingParam::Request &req, kicking_module_msgs::GetKickingParam::Response &res);

    void loadConfig();
    void loadKickManip();
    void loadFeedbackParam();
    void saveConfig(const std::string &path);
    void updateKickingParam();

    int feedbackMode;
    //Full State Feedback
    double Gain_Angle_Pitch,     Gain_Angle_Roll;
    double Gain_Velocity_Pitch,  Gain_Velocity_Roll;
    double Gain_Integral;

    //Output Feedback
    double KrCOMX, KrCOMY;
    double KaR, KgR;
    double KaP, KgP;
    double deltaRoll, deltaPitch;
    double COMrefY, COMrefX;
    double KCOMX, KCOMY;
    double angleErrorMAX, angleErrorMIN;

    //Linear Quadratic + Integrator
    double KiCOMY, KiCOMX;

    //Right Kick Feedback Variables
    double RK_COMrefY, RK_COMrefX;
    double RK_KaR, RK_KgR;
    double RK_KaP, RK_KgP;
    double RK_KCOMX, RK_KCOMY;

    //Left Kick Feedback Variables
    double LK_COMrefY, LK_COMrefX;
    double LK_KaR, LK_KgR;
    double LK_KaP, LK_KgP;
    double LK_KCOMX, LK_KCOMY;

    //DSP Feedback Variables
    bool zeroOut;
    double DSP_COMrefY, DSP_COMrefX;
    double DSP_KaR, DSP_KgR;
    double DSP_KaP, DSP_KgP;
    double DSP_KCOMX, DSP_KCOMY;
    double DSP_pitchErrorMAX, DSP_pitchErrorMIN;
    bool highGain;
    double smallDSP_KaP, smallDSP_KgP;

    //Setpoint Position
    double Body_Height, Body_Tilt, Body_Roll;
    double L_Body_Tilt, R_Body_Tilt;
    double Leg_X, Leg_Y, Leg_Z;
    double X_Retract, Z_Retract;

    //Parameter Setpoint
    double Leg_R_X, Leg_L_X;
    double L_X_Manip, R_X_Manip;
    double R_Roll, L_Roll;

    //Parameter Kicking
    double R_Shift, L_Shift;
    double delta_R_Shift, delta_L_Shift;
    double temp_delta_R_Shift, temp_delta_L_Shift;
    double R_Lift, L_Lift;
    double R_X_Shift, L_X_Shift;
    double R_CoolDown, L_CoolDown;
    double R_Z_Lift, L_Z_Lift;

    //Parameter Time
    double Time_Shift, Time_Lift, Time_Retract, Time_Kick, Time_CoolDown, Time_BringDown;

private:
    enum
    {
        KickingDisable = 0,
        KickingEnable = 1,
        KickingInitPose = 2,
        KickingReady = 3
    };
    enum
    {
        Right_Kick = 1,
        Left_Kick = 2,
        Right_Side_Kick = 3,
        Left_Side_Kick = 4
    };
    enum
    {
        Trq2Pos = 1,
        Pos2Pos = 2
    };
    enum
    {
        densisDSP = 0,
        densisRAnkPitch = 1,
        densisLAnkPitch = 2,
        densisRAnkRoll = 3,
        densisLAnkRoll = 4
    };
    enum
    {
        DSP = 0,
        LSSP = 1,
        RSSP = 2
    };
    enum
    {
        FRONTFALLING = 0,
        ANTENG = 1,
        BACKFALLING = 2
    };

    robotis_op::OP3KinematicsDynamics* op3_kd_;
    int control_cycle_msec_;
    std::string config_path_;
    std::string physicalParam_path_;
    std::string feedbackParam_path_;
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
    std::vector<double> movAvg;
    double gyro_y_, gyro_y;
    double samplingRate;
    double EMAV2(const std::vector<double> movAvg, double n );
    Eigen::Vector3d m_imuOri;
    Eigen::Vector3d m_imuAcc;
    Eigen::Vector3d m_imuAccFil;
    Eigen::MatrixXd rotIMU;
    Eigen::Vector3d accelCOM(Eigen::Vector3d COM,Eigen::Vector3d posFootFromBase);
    Eigen::Vector3d m_accelCOM;
    Eigen::Vector3d accelCOMFil(Eigen::Vector3d COM,Eigen::Vector3d posFootFromBase);
    Eigen::Vector3d m_accelCOMFil;
    Eigen::Vector3d movingAverage(Eigen::Vector3d rawData);
    std::vector<double> bufferAccX,bufferAccY, bufferAccZ;
    Eigen::Vector3d cumsum;
    Eigen::Vector3d filteredData;
    Eigen::Vector3d rawData;
    int MeasurementsToAverage;

    Eigen::Vector3d omega_dot;
    Eigen::MatrixXd omega_cross;
    Eigen::Vector3d pre_omega;

    //ROS Publisher and Subscriber
    ros::Publisher kick_status_pub_;
    ros::Publisher data_log_pub_;
    ros::Publisher densis_pub_;
    void kickStatus(const std::string &command);
    void dataLogPublish();
    data_log_msgs::DataLog dataLogMsgs;
    void densisPublish();
    densis_msgs::densis densisMsgs;
    struct Point {
    float x;
    float y;
    };
    Point current_joint_arr[20];
    Point point;

    Eigen::MatrixXd target_position_;
    Eigen::MatrixXd current_position_;
    Eigen::MatrixXd init_position_;
    Eigen::MatrixXi joint_axis_direction_;
    Eigen::MatrixXd current_velocity_;
    std::map<std::string, int> joint_table_;

    bool firstExc;
    double firstTimeExc;
    double timeExc;

    int init_pose_count_;
    KDL::JntArray current_joint_pos;
    KDL::JntArray current_joint_vel;
    Eigen::MatrixXd calc_joint_tra_;

    std::string op3_joint[20], joint[18], offset_path_, kickManip_path_;
    double offset[18], op3_offset[20];
    Eigen::Vector3d posLFootFromBase;
    Eigen::Vector3d posRFootFromBase;
    Eigen::Vector3d posDSPFromBase;
    Eigen::Vector3d passiveLSSP;
    Eigen::Vector3d passiveRSSP;
    Eigen::Vector3d passiveDSP;

    Eigen::Vector2d angle;
    Eigen::Vector2d velocity_angle;
    Eigen::Vector2d integral_angle;

    bool m_active;
    bool m_start;
    int m_position;
    double m_time_start;
    double m_delta_time;
    // double delta_time;
    // double tr;

    Kinematics k;

    Matrix<double, 2, 1> rollAnkle;
    Matrix<double, 2, 1> pitchAnkle;
    Matrix<double, 18, 1> getJoints;
    Matrix<double, 18, 1> feedback;

    struct feedbackParam{
        double ZMPrefX;
        double ZMPrefY;
        double KrZMPx;
        double KrZMPy;
        double KAngleRoll;
        double KGyroRoll;
        double KAnglePitch;
        double KGyroPitch;
        double pitchErrorMAX;
        double pitchErrorMIN;
        double rollErrorMAX;
        double rollhErrorMIN;
    }fbDSP, fbLSSP, fbRSSP;

    struct stateVariable{
        double pitch;
        double gyroPitch;
        double roll;
        double gyroRoll;
    }stDSP, stLSSP, stRSSP;

    void resetServoValues();

    Eigen::Vector3d actual_torso;
    double R_X, R_Y, R_Z, R_R, R_P, R_A;
    double L_X, L_Y, L_Z, L_R, L_P, L_A;
    Eigen::Matrix<double, 6, 1> L_Leg, R_Leg, Torso;
    // Matrix<double, 18, 1> current_joint_pos;
    double init_LLegY, init_RLegY;

    Eigen::Affine3d BASE;
    Eigen::Affine3d R_HIP, R_KNEE, R_ANKLE, R_FOOT, m_R_FOOT;
    Eigen::Affine3d L_HIP, L_KNEE, L_ANKLE, L_FOOT, m_L_FOOT;
    Eigen::Vector3d L_FOOT_WORLD, R_FOOT_WORLD;
    Eigen::Vector3d L_ROT_WORLD, R_ROT_WORLD;

    // Eigen::VectorXd L_FOOT_WORLD, R_FOOT_WORLD;
    Eigen::VectorXd LFootWorld, RFootWorld;
    int IndexSupport;

    double R_X_Kick, R_Y_Kick, R_Z_Kick;
    double L_X_Kick, L_Y_Kick, L_Z_Kick;
    double ballPosX, ballPosZ;

    void setValue();
    double sigmoid(double C, double G, double H, double x);
    void shifting(double duration, double totalPrevious);
    void shiftingHold();
    void rolling(double duration, double totalPrevious);
    void lifting(double duration, double totalPrevious);
    void liftingHold();
    void retract(double duration, double totalPrevious);
    void kicking(double duration, double totalPrevious);
    void coolingDown(double duration, double totalPrevious);
    void bringDown(double duration, double totalPrevious);
    void DSPHold();
    void printValue();
    void motion_arms();
    void forwardKinematic();
    void wholeBodyCOM();
    void wholeBodyCOM_LSSP();
    void wholeBodyCOM_RSSP();
    void wholeBodyCOM_DSP();
    void calculateZMP();
    void densisInput();
    
    void feedbackDecision();
    double integratorPitch;
    double integratorRoll;
    double inputTorsiPitch;
    double inputTorsiRoll;
    void feedbackDSPAnklePitchPos2Pos();
    void   resetFeedback();
    double torquePitch, torqueRoll;
    double feedbackPitch, feedbackRoll;
    bool printKickVar, feedbackActive, servoFeedbackActive, initPose, FKfromgoalPos;

    //Calculate COM
    Eigen::MatrixXd COM, COM_LSSP, COM_RSSP, COM_DSP;
    Eigen::MatrixXd op3_rot;
    Eigen::Vector3d op3_rpy;
    Eigen::Vector3d COMLSSPrpy, COMRSSPrpy, COMDSPrpy;
    double COM_LSSP_Length, COM_RSSP_Length, COM_DSP_Length;
    //Calculate Inertia Tensor
    void globalInertia();
    Eigen::MatrixXd totalInertia;
    void globalInertiaLSSP();
    Eigen::MatrixXd totalInertiaLSSP;
    void globalInertiaRSSP();
    Eigen::MatrixXd totalInertiaRSSP;
    void globalInertiaDSP();
    Eigen::MatrixXd totalInertiaDSP;

    Eigen::Vector3d ZMP;
    Eigen::Vector3d ZMPLSSP;
    Eigen::Vector3d ZMPRSSP;
    Eigen::Vector3d ZMPDSP;
    Eigen::Vector3d ZMPLSSP_accFil;
    Eigen::Vector3d ZMPRSSP_accFil;
    Eigen::Vector3d ZMPDSP_accFil;
    Eigen::Vector3d pelvisToCOM;

    //Densis variables
    int ankleMode;

    //Kicking Mode
    bool liftFoot;
    bool holdPose;

    //Coba2
    boost::chrono::steady_clock::time_point c1;

    // Model Based Control
    robotis_op::servoObserver so;
    robotis_op::modelBasedControl mbc;
    robotis_op::densis densis;
    double balance_angle[12];
};

}

#endif
