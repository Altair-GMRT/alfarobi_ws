#ifndef QUINTICWALK_H
#define QUINTICWALK_H

#include <stdio.h>
#include <cmath>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <boost/assign/std/vector.hpp>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "model_based_control/modelBasedControl.h"
#include "model_based_control/servoObserver.h"
#include "model_based_control/densis.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "Footstep.hpp"
#include "AnalyticIKSolver.hpp"
#include "quintic_walk_msgs/WalkingParam.h"
#include "quintic_walk_msgs/SetWalkingParam.h"
#include "quintic_walk_msgs/GetWalkingParam.h"
#include "utils/TrajectoryUtils.h"
#include "utils/VectorLabel.hpp"
#include "utils/AxisAngle.h"
#include "utils/Angle.h"
#include "utils/Euler.h"
#include "analyze_msgs/Fuzzy.h"
#include "analyze_msgs/movAvg.h"
#include "densis_msgs/densis.h"
#include "fuzzy.hpp"

#include "kinematics.h"

#define PI 3.14159265
#define G_CONSTANT 9.80665
#define CUPLIK_EMA 15

using namespace Eigen;

namespace robotis_op
{

class QuinticWalk : public robotis_framework::MotionModule, public robotis_framework::Singleton<QuinticWalk>
{
public:
    QuinticWalk();
    virtual ~QuinticWalk();

    void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
    void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
    void stop();
    bool isRunning();
    void onModuleEnable();
    void onModuleDisable();

    Footstep getFootstep();
    void walkingSpeedCallback(const std_msgs::String::ConstPtr &msg);


    /**
     * Get the weight balance between left and right leg.
     * 1.0 means all weight on left leg
     * 0.0 means all weight on right leg
     * during double support phase, the value is between those two values
     */
    double getWeightBalance();

    /**
     * Return true is the walk
     * oscillations are enabled
     */
    bool isEnabled() const;

    /**
     * Return if true if left is current support foot
     */
    bool isLeftSupport();

    /**
     * Return true if both feet are currently on the ground
     */
    bool isDoubleSupport();

    /**
     * Return current walk phase
     * between 0 and 1
     */
    double getPhase() const;

    /**
     * Return current time between
     * 0 and half period for
     * trajectories evaluation
     */
    double getTrajsTime() const;

    /**
     * Get current walk footstep orders
     */
    const Eigen::Vector3d& getOrders() const;

    /**
     * Rebuilt the trajectories and
     * reset saved state as disable.
     * Used to directly apply
     * newly parameters.
     */
    void forceRebuildTrajectories();

    /**
     * Set used walk footstep orders,
     * enable or disable the walk oscillations
     * and optionnaly set the starting
     * supporting foot.
     */
    void setOrders(const Eigen::Vector3d& orders, bool isEnabled, bool beginWithLeftSupport = true);

    /**
     * Return the trajectories for
     * current half cycle
     */
    const Trajectories& getTrajectories() const;

    /**
     * Update the internal walk state
     * (pĥase, trajectories) from given
     * elapsed time since last update() call
     */
    void update(double dt);

    /**
     * Compute current cartesian
     * target from trajectories and assign
     * it to given model through inverse
     * kinematics.
     * Return false is the target is
     * unreachable.
     */

    void computeCartesianPosition(Eigen::Vector3d& trunkPos, Eigen::Vector3d& trunkAxis,
                                               Eigen::Vector3d& footPos, Eigen::Vector3d& footAxis, bool& isLeftsupportFoot);

    void computeCartesianPositionAtTime(Eigen::Vector3d& trunkPos, Eigen::Vector3d& trunkAxis, Eigen::Vector3d& footPos,
                                    Eigen::Vector3d& footAxis, bool& isLeftsupportFoot, double time);


private:
    enum WalkingStatus
    {
        WalkingDisable = 0,
        WalkingEnable = 1,
        WalkingInitPose = 2,
        WalkingReady = 3
    };

    // FSM
    enum RobotStatus
    {
        RobotStop = 0,
        RobotWalk = 1,
        RobotCaptureStep = 2
    };
    size_t robot_state, robot_previous_state;
    double capturePointValue;
    // Capture Point
    double calcCapturePoint();
    // Input Walking
    Eigen::Vector3d walkingParam;
    bool CP_active;
    double CP_GAIN;
    double CP_start;

    int walking_state_;
    bool firstExc;
    int init_pose_count_;
    Eigen::MatrixXd calc_joint_tra_;

    /**
     * Reset and rebuild the
     * spline trajectories for
     * current half cycle
     */
    void buildTrajectories();

    /**
     * Reset the trunk position and
     * orientation state vectors at last
     * half cycle as stopped pose
     */
    void resetTrunkLastState();

    void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void walkingReset();
    void motion_arms();
    
    void calculateWalking();
    void walkingCommandCallback(const std_msgs::String::ConstPtr &msg);
    void walkingParameterCallback(const quintic_walk_msgs::WalkingParam::ConstPtr &msg);
    bool getWalkigParameterCallback(quintic_walk_msgs::GetWalkingParam::Request &req,
                                    quintic_walk_msgs::GetWalkingParam::Response &res);
    void publishStatusMsg(unsigned int type, std::string msg);
    void walkingStatus(const std::string &command);

    // Activate Capture Point From Behavioral of Strategy
    void CPCallback(const std_msgs::Bool::ConstPtr &msg);

    void queueThread();
    void loadParameter();
    void loadOffset();
    void loadFuzzy();
    void saveParameter();

    void forwardKinematic();
    double EMAV2(const std::vector<double> movAvg, double n );
    void wholeBodyCOM();
    void wholeBodyCOM_DSP();
    void wholeBodyCOM_LSSP();
    void wholeBodyCOM_RSSP();
    void densisInput();

    double stepFeedback;

    int control_cycle_msec_;
    std::string param_path_;
    boost::thread queue_thread_;
    boost::mutex publish_mutex_;

    ros::Publisher status_msg_pub_;
    ros::Publisher walking_status_pub_;
    ros::Publisher fuzzy_pub;
    ros::Publisher gyr_pub;
    ros::Publisher densis_pub_;
    ros::Subscriber cp_sub_;

    Eigen::MatrixXd target_position_;
    Eigen::MatrixXd goal_position_;
    Eigen::MatrixXd init_position_;
    Eigen::MatrixXi joint_axis_direction_;

    std::map<std::string, int> joint_table_, current_position, joint_offset;
    Eigen::MatrixXd current_position_;
    bool debug_print_;

    robotis_op::OP3KinematicsDynamics* op3_kd_;
    bool ctrl_running_;
    bool real_running_;
    double time_;

    quintic_walk_msgs::WalkingParam walking_param_;

    // tambahan Bagas:
    densis_msgs::densis densisMsgs;
    bool feedbackActive;
    void feedbackDSPAnklePitchPos2Pos();
    void densisPublish();

    // Forward Kinematic
    Kinematics k;
    Eigen::Matrix<double, 18, 1> current_joint_pos;
    Eigen::Matrix<double, 18, 1> getJoints;
    Eigen::Affine3d BASE;
    Eigen::Affine3d R_HIP, R_KNEE, R_ANKLE, R_FOOT, m_R_FOOT;
    Eigen::Affine3d L_HIP, L_KNEE, L_ANKLE, L_FOOT, m_L_FOOT;
    Eigen::Vector3d L_FOOT_WORLD, R_FOOT_WORLD;
    Eigen::Vector3d L_ROT_WORLD, R_ROT_WORLD;
    Eigen::VectorXd LFootWorld, RFootWorld;
    double R_X, R_Y, R_Z, R_R, R_P, R_A;
    double L_X, L_Y, L_Z, L_R, L_P, L_A;
    double iR_X, iR_Y, iR_Z, iR_R, iR_P, iR_A;
    double iL_X, iL_Y, iL_Z, iL_R, iL_P, iL_A;
    double wR_X, wR_Y, wR_Z, wR_R, wR_P, wR_A;
    double wL_X, wL_Y, wL_Z, wL_R, wL_P, wL_A;

    std::string joint[12], offset_;
    std::string kickManip_path_, physicalParam_path_;
    double offset[12];

    double IndexSupport;

    double Ankle_Pitch;
    double orientation_y, gyro_y, gyro_y_, gyro_y_EMAV2, trunkPitch, trunkXOffset;
    double orientation_x, gyro_x;
    double m_Kp,m_Kg;

    std::vector<double> movAvg;

    //Feedback with Fuzzy
    struct fuzzyData
    {
        int DataAmount;
        std::vector<double> Upper1;
        std::vector<double> Bottom1;
        std::vector<double> Bottom2;
        std::vector<double> Upper2;
    } Angle, Gyro, Velocity, Kp, Kd;
    Fuzzy ANGLE, GYRO, VELOCITY, KP, KD;
    std::string fuzzy_path_;
    double outputFeedback();

    //COM Norm
    double COM_DSP_Length;
    Eigen::MatrixXd rotIMU;
    Eigen::MatrixXd COM, COM_DSP, COM_LSSP, COM_RSSP;
    Eigen::Vector3d posLFootFromBase;
    Eigen::Vector3d posRFootFromBase;
    Eigen::Vector3d posDSPFromBase;
    Eigen::Vector3d passiveDSP;
    Eigen::Vector3d COMDSPrpy, COMLSSPrpy, COMRSSPrpy;

    //Kicking Mode
    bool liftFoot;
    bool holdPose;

    /**
     * Current footstep support
     * and flying last and next pose
     */
    Footstep _footstep;

    AnalyticIKSolver ik_solver;

    /**
     * Movement phase between 0 and 1
     */
    double _phase;

    /**
     * Currently used parameters
     */
    VectorLabel _params;

    /**
     * Currently used footstep
     * orders flush at next suppot
     * foot swap
     */
    Eigen::Vector3d _orders;

    /**
     * Enable or disable
     * the oscillations and
     * value at last half cycle.
     */
    bool _isEnabled;
    bool _wasEnabled;

    /**
     * True if the current used
     * trajectories has oscillations
     */
    bool _isTrajsOscillating;

    /**
     * Trunk pose and orientation
     * position, velocity and acceleration
     * at half cycle start
     */
    Eigen::Vector3d _trunkPosAtLast;
    Eigen::Vector3d _trunkVelAtLast;
    Eigen::Vector3d _trunkAccAtLast;
    Eigen::Vector3d _trunkAxisPosAtLast;
    Eigen::Vector3d _trunkAxisVelAtLast;
    Eigen::Vector3d _trunkAxisAccAtLast;

    //std::vector<std::string> _joint_ordering {"head_yaw","head_pitch","left_shoulder_pitch","left_shoulder_roll","left_elbow","right_shoulder_pitch","right_shoulder_roll","right_elbow","left_hip_yaw","left_hip_roll","left_hip_pitch","left_knee","left_ankle_pitch","left_ankle_roll","right_hip_yaw","right_hip_roll","right_hip_pitch","right_knee","right_ankle_pitch","right_ankle_roll"};
    std::vector<std::string> _joint_ordering {"HeadYaw","HeadPitch","LShoulderPitch","LShoulderRoll","LElbow","RShoulderPitch","RShoulderRoll","RElbow","LHipYaw","LHipRoll","LHipPitch","LKnee","LAnklePitch","LAnkleRoll","RHipYaw","RHipRoll","RHipPitch","RKnee","RAnklePitch","RAnkleRoll"};
    std::vector<std::string> _left_leg_joint_names {"LHipYaw", "LHipRoll", "LHipPitch", "LKnee", "LAnklePitch", "LAnkleRoll"};
    std::vector<std::string> _right_leg_joint_names {"RHipYaw", "RHipRoll", "RHipPitch", "RKnee", "RAnklePitch", "RAnkleRoll"};

    bool _debugActive;
    bool _walkActive;
    bool _stopRequest;
    double _engineFrequency;
    std::chrono::time_point<std::chrono::steady_clock> _last_update_time;
    bool _just_started;

    Eigen::Vector3d _stepOdom;
    tf::Transform _supportFootOdom;

    std::string _ik_type;
    std::string _robot_type;

    Eigen::Vector3d _trunkPos;
    Eigen::Vector3d _trunkAxis;
    Eigen::Vector3d _footPos;
    Eigen::Vector3d _footAxis;
    bool _isLeftSupport;
    bool _wasLeftSupport;

    bool _compensate_gravity;

    std::vector<double> joint_goals;
    // Eigen::Matrix<double, 18, 1> feedback;
    Eigen::MatrixXd feedback;


    /**
     * Generated half walk
     * cycle trajectory
     */
    Trajectories _trajs;

    Eigen::Vector3d m_imuGyr;
    Eigen::Vector3d m_imuOri;
    Eigen::Vector3d m_imuAcc;

    // Model Based Control
    robotis_op::servoObserver so;
    robotis_op::modelBasedControl mbc;
    robotis_op::densis densis;
    double skripsiEMA(double rawData);
    void fuzzyQuintic();
    std::vector<std::vector<double>> gainFuzzy;
    std::vector<double> bufferSkripsiEMA;

    // Integrator
    double integralPitch;

    //simpleCapturePoint
    void simpleCapturePoint();
    double lastXMove = .0;
    double orientation_y_capture;
    int IndexSupport_capture = 0;
    int Index_tmp = 0;
    int supportCount = 0;
    bool firstActive = false;
    bool firstCond = true;
    bool countCapture;
    double timeNow;
    double firstTime;
    double timeStep = 0;
    double deltaStep = 0;

    // 
};

}

#endif // QUINTICWALK_H
