#ifndef CAPTURE_STEP_H
#define CAPTURE_STEP_H

#include <stdio.h>
#include <math.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <eigen_conversions/eigen_msg.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "robotis_math/robotis_trajectory_calculator.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "capture_step_module_msgs/WalkingParam.h"
#include "capture_step_module_msgs/GetWalkingParam.h"
#include "capture_step_module_msgs/SetWalkingParam.h"
#include "Action.h"


#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )

#define VX_MAX 1 // Max Sagital Speed
#define VY_MAX 1 // Max Lateral Speed
#define VPHI_MAX 1 // Max Rotational Speed

namespace robotis_op {

class CaptureStepModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<CaptureStepModule>
{
public:
  enum WalkingPhase{
    LEFTSUPPORT,
    L2R,
    RIGHTSUPPORT,
    R2L};
  WalkingPhase phase;

  struct desiredStep{
    double Stepx;
    double Stepy;
    double Stepphi;
  }; desiredStep dStep;

  struct COMState{
    double Cx;
    double Cxdot;
    double Cy;
    double Cydot;
  }; COMState com;

  struct exchangeState{
    double Sx;
    double Sxdot;
    double Sy;
    double Sydot;
    double halfT;
  }; exchangeState desSL, desSR;

  LegPose Leg, RightLeg, LeftLeg;

  // Support leg information
  enum
  {
    LEFT_LEG = -1,
    RIGHT_LEG = 1,
    BASE = 0,
    COM = 25,
    leftFootStep = 26,
    rightFootStep = 27,
    suppFootStep = 28,
    PELVIS = 29,
    LEFT_FOOT = 30,
    RIGHT_FOOT = 31
  };

  capture_step_module_msgs::WalkingParam walking_param_;
  std::map<std::string, robotis_framework::DynamixelState *> current_;
  CaptureStepModule();
  virtual ~CaptureStepModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void captureStep();
  void walk_step();
  void startWalking();
  void stop();
  bool isRunning();
  void onModuleDisable();
  void onModuleEnable();

private:
  enum
  {
    WalkingDisable = 0,
    WalkingEnable = 1,
    WalkingInitPose = 2,
    WalkingReady = 3
  };
  void queueThread();

  /* ROS Topic Callback Functions */
  void walkingCommandCallback(const std_msgs::String::ConstPtr &msg);
  void walkingParameterCallback(const capture_step_module_msgs::WalkingParam::ConstPtr &msg);
  bool getWalkigParameterCallback(capture_step_module_msgs::GetWalkingParam::Request &req,
                                  capture_step_module_msgs::GetWalkingParam::Response &res);
  void gazeboImuCallback(const sensor_msgs::ImuConstPtr &msg);
  void gazeboMagCallback(const geometry_msgs::Vector3StampedPtr &msg);
  void currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

  void publishStatusMsg(unsigned int type, std::string msg);
  void goToHaltPose(int sign);

  void setV(Eigen::Vector3d m_Vtarget);
  exchangeState getReferenceTrajectory(Eigen::Vector3d, double leg);
  Eigen::Vector2d balanceControl(double leg, COMState curC, CaptureStepModule::exchangeState desS);
  void controlInterface(double m_VxTarget, double m_VyTarget, double m_VphiTarget);

  void calculateJointAngles(int leg, double& eta, LegPose& Leg);
  void calculateWalkPattern(int leg, double tau, double& eta, LegPose& Leg);
  void updateState();

  void loadWalkingParam(const std::string &path);
  void saveWalkingParam(std::string &path);

  robotis_op::OP3KinematicsDynamics* op3_kd_;
  int control_cycle_msec_;
  std::string param_path_;
  boost::thread queue_thread_;
  boost::mutex publish_mutex_;

  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;

  Eigen::MatrixXd calc_joint_tra_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd current_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd init_position_;
  Eigen::MatrixXi joint_axis_direction_;
  std::map<std::string, int> joint_table_;

//  RobotModel robot_model_;
//  RobotState robot_state_;

  int walking_state_;
  int cycleNumber;
  bool debug_print_;
  int init_pose_count_;
  ros::Time last_t;

  // variable for state estimation TODO calculate this shit
  double  alpha = 0.3; //lateral  distance  between  the  support frame and the CoM at the step apex
  double  delta= 0.3; //defines the lateral support exchange location
  double  omega= 0.3; // the support exchange location for the maximal lateral walking velocity
  double  sigma= 0.3; //maximum sagittal walking velocity
  double const C= 0.3; //sqrt g / com height
  double xi;

  Eigen::Vector3d m_VTarget;
  Eigen::Vector3d m_V;
  double m_VxTarget, m_VyTarget, m_VphiTarget; // V desired
  double m_Vx,m_Vy,m_Vphi; // V in the algorithm
  double m_Tau, m_TauRate;

  bool ctrl_running_;
  bool real_running_;
  double time_;

  double it[12];

  double lambda;
  double hipYaw ;
  double hipRoll;
  double hipPitch;
  double knee;
  double ankPitch;
  double ankRoll;

  double gx;
  double gy;
  double gz;
  double ax;
  double ay;
  double az;
  double mx;
  double my;
  double mz;

};
}

#endif // CAPTURE_STEP_H
