#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/time.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/thread.hpp>
#include <boost/any.hpp>

#include <kdl_conversions/kdl_msg.h>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <odometry/kinematics.h>

#include <robotis_controller/robotis_controller.h>
#include <robotis_device/sensor.h>
#include <robotis_framework_common/sensor_module.h>
#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <eigen3/Eigen/Eigen>

#define G_CONST 9.81
#define PI 3.14159265

using namespace Eigen;

namespace robotis_op
{

  class Odometry : public robotis_framework::SensorModule, public robotis_framework::Singleton<Odometry>
  {
  public:
    Odometry();
    ~Odometry();

    void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
    void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                 std::map<std::string, robotis_framework::Sensor *> sensors);

    void start();
    void stop();

    std::map<std::string, robotis_framework::DynamixelState *> current_;

    bool m_active;

  private:
    enum
    {
      Double_Support = 0,
      Right_Support = 1,
      Left_Support = 2,
    };

    int control_cycle_msec_;
    boost::thread queue_thread_;
    const bool DEBUG_PRINT;

    int checkSupport();
    int current_support, prev_support;

    bool dsp_flag;
    double x_dist, y_dist;
    double length_temp;
    double resetOffset;

    Kinematics k;

    /* ROS Topic Callback Functions */
    void queueThread();

    void ImuCallback(const sensor_msgs::ImuConstPtr &msg);
    void odometryCommandCallback(const std_msgs::String::ConstPtr &msg);
    void goalPosCommandCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void buttonHandlerCallback(const std_msgs::String::ConstPtr &msg);

    Eigen::Vector3d m_imuGyr;
    Eigen::Vector3d m_imuOri;

    geometry_msgs::Point odometry_;
    geometry_msgs::PoseStamped odometry_out_;
    geometry_msgs::Point angle_;
    geometry_msgs::Point angular_velocity_;
    geometry_msgs::Point offset_angle_;

    ros::Publisher odometry_pub_;
    ros::Publisher robot_height_pub_;
    ros::Publisher angle_pub_;
    ros::Publisher angular_velocity_pub_;
    ros::Publisher vis_pub;

    int odometry_pub_counter;
    int counter_id;

    Matrix<double, 18, 1> getJoints;
    KDL::JntArray current_joint_pos;
    KDL::JntArray current_joint_pos_;
    Eigen::MatrixXd current_position_;
    Eigen::MatrixXi joint_axis_direction_;
    std::map<std::string, int> joint_table_;
    std::map<std::string, int> goal_joint_table_;
    std::string joint[12], offset_;
    double offset[12];

    void forwardKinematic();
    Eigen::Affine3d ODOM;
    Eigen::Affine3d BASE;
    Eigen::Affine3d R_HIP, R_KNEE, R_ANKLE, R_FOOT, m_R_FOOT;
    Eigen::Affine3d L_HIP, L_KNEE, L_ANKLE, L_FOOT, m_L_FOOT;
    Eigen::Vector3d ODOM_VECTOR;
    Eigen::Vector3d L_FOOT_WORLD, R_FOOT_WORLD;
    Eigen::Vector3d L_ROT_WORLD, R_ROT_WORLD;

    void calculation();

    double hip_to_base, hip_to_base_pitch, hip_to_base_roll;
    double camera_to_base;

    double offset_imu_x, offset_imu_y, offset_imu_z;

    geometry_msgs::Point nonpoly_odom;
  };
} // namespace robotis_op

#endif // ODOMETRY_H
