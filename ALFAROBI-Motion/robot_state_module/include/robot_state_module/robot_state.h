#ifndef ROBOT_STATE_MODULE_H
#define ROBOT_STATE_MODULE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <robotis_controller/robotis_controller.h>
#include <robotis_device/sensor.h>
#include <robotis_framework_common/sensor_module.h>
#include <robotis_math/robotis_math.h>

namespace robotis_op
{

class RobotState : public robotis_framework::SensorModule, public robotis_framework::Singleton<RobotState>
{
public:
  RobotState();
  ~RobotState();

  /* ROS Topic Callback Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, robotis_framework::Sensor *> sensors);

private:
  int control_cycle_msec_;
  boost::thread queue_thread_;
  const bool DEBUG_PRINT;

  ros::Publisher base2d_;
  ros::Publisher base_pose_;
  ros::Publisher camera_pub_;

  void queueThread();
  void initializeModel();
  void updateModel();
  void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

  Eigen::Affine3d world;
  Eigen::Affine3d base;
  Eigen::Affine3d hipYawR, hipRight, kneeRight, ankleRight, footRight;
  Eigen::Affine3d hipYawL, hipLeft, kneeLeft, ankleLeft, footLeft;
  Eigen::Affine3d head_pan, head_tilt, camera;
  Eigen::Affine3d armPitchR, armRollR, armElbR;
  Eigen::Affine3d armPitchL, armRollL, armElbL;
  Eigen::Matrix3d mat_reset;

  Eigen::Vector3d prev_trans_;
  double prev_yaw_;


  sensor_msgs::Imu imu_;

  double BaseHipYawx, BaseHipYawy, BaseHipYawz;    //OP, measured
  double HipYawRollx, HipYawRollz;
  double thighLength;
  double tibiaLength;
  double footHeight;

  double BaseHeadPanx, BaseHeadPany, BaseHeadPanz;
  double HeadPanTiltx, HeadPanTiltz;
  double HeadTiltCamx, HeadTiltCamy, HeadTiltCamz;

  double BaseArmPitchx, BaseArmPitchy, BaseArmPitchz;
  double ArmPitchRolly, ArmPitchRollz;
  double ArmRollElbx, ArmRollElbz;

  std::map<std::string, int> joint_table_;
  std::map<std::string, double> joint_pos_;



};


}

#endif // ROBOT_STATE_H
