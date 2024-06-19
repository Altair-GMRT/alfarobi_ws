#include <robot_state_module/robot_state.h>

using namespace Eigen;
namespace robotis_op
{

RobotState::RobotState()
  : control_cycle_msec_(8),
    DEBUG_PRINT(false)
{
    module_name_ = "robot_state_module";

    BaseHipYawx = 0.0175; BaseHipYawy = 0.0384; BaseHipYawz = 0.09728;
    HipYawRollx = 0.003; HipYawRollz=  0.0335;
    thighLength = .1168;
    tibiaLength = .1168;
    footHeight = .0355;

    BaseHeadPanx = 0.0175; BaseHeadPany = 0.000125; BaseHeadPanz = 0.05982;
    HeadPanTiltx = 0.002691; HeadPanTiltz = 0.043;
    HeadTiltCamx = 0.009; HeadTiltCamy = 0.000717; HeadTiltCamz = 0.06021;

    BaseArmPitchx = 0.0175; BaseArmPitchy = 0.066125; BaseArmPitchz = 0.03652;
    ArmPitchRolly = 0.0245; ArmPitchRollz = 0.016;
    ArmRollElbx = 0.021; ArmRollElbz = 0.1072;

    joint_table_["r_hip_yaw"] = 0;
    joint_table_["r_hip_roll"] = 1;
    joint_table_["r_hip_pitch"] = 2;
    joint_table_["r_knee"] = 3;
    joint_table_["r_ank_pitch"] = 4;
    joint_table_["r_ank_roll"] = 5;

    joint_table_["l_hip_yaw"] = 6;
    joint_table_["l_hip_roll"] = 7;
    joint_table_["l_hip_pitch"] = 8;
    joint_table_["l_knee"] = 9;
    joint_table_["l_ank_pitch"] = 10;
    joint_table_["l_ank_roll"] = 11;

    joint_table_["head_pan"] = 12;
    joint_table_["head_tilt"] = 13;

    joint_pos_["r_hip_yaw"] = 0;
    joint_pos_["r_hip_roll"] = 0;
    joint_pos_["r_hip_pitch"] = 0;
    joint_pos_["r_knee"] = 0;
    joint_pos_["r_ank_pitch"] = 0;
    joint_pos_["r_ank_roll"] = 0;

    joint_pos_["l_hip_yaw"] = 0;
    joint_pos_["l_hip_roll"] = 0;
    joint_pos_["l_hip_pitch"] = 0;
    joint_pos_["l_knee"] = 0;
    joint_pos_["l_ank_pitch"] = 0;
    joint_pos_["l_ank_roll"] = 0;

    joint_pos_["head_pan"] = 0;
    joint_pos_["head_tilt"] = 0;

}

RobotState::~RobotState()
{
  queue_thread_.join();
}

void RobotState::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&RobotState::queueThread, this));

  initializeModel();

}

void RobotState::initializeModel()
{
    mat_reset = Eigen::Matrix3d::Identity();

    world = Affine3d(Translation3d(Vector3d(0,0,0)));
    base = Affine3d(Translation3d(Vector3d(0, 0, 0)));
    hipYawL = Affine3d(Translation3d(Vector3d(BaseHipYawx, BaseHipYawy, -BaseHipYawz)));
    hipYawR = Affine3d(Translation3d(Vector3d(BaseHipYawx, -BaseHipYawy, -BaseHipYawz)));
    hipLeft = Affine3d(Translation3d(Vector3d(-HipYawRollx, 0, -HipYawRollz)));
    hipRight = Affine3d(Translation3d(Vector3d(-HipYawRollx, 0, -HipYawRollz)));
    kneeRight = Affine3d(Translation3d(Vector3d(0, 0, -thighLength)));
    kneeLeft = Affine3d(Translation3d(Vector3d(0, 0, -thighLength)));
    ankleRight = Affine3d(Translation3d(Vector3d(0, 0, -tibiaLength)));
    ankleLeft = Affine3d(Translation3d(Vector3d(0, 0, -tibiaLength)));
    footRight = Affine3d(Translation3d(Vector3d(0, 0, -footHeight)));
    footLeft = Affine3d(Translation3d(Vector3d(0, 0, -footHeight)));

    head_pan = Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(BaseHeadPanx, BaseHipYawy, BaseHipYawz)));
    head_tilt = Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(HeadPanTiltx,0, HeadPanTiltz)));
    camera = Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(HeadTiltCamx,HeadTiltCamy, HeadTiltCamz)));

    armPitchL = Affine3d(Translation3d(Vector3d(BaseArmPitchx, BaseArmPitchy, BaseArmPitchz)));
    armRollL = Affine3d(Translation3d(Vector3d(0, ArmPitchRolly, -ArmPitchRolly)));
    armElbL = Affine3d(Translation3d(Vector3d(ArmRollElbx, 0, -ArmRollElbz)));

    armPitchR = Affine3d(Translation3d(Vector3d(BaseArmPitchx, -BaseArmPitchy, BaseArmPitchz)));
    armRollR = Affine3d(Translation3d(Vector3d(0, -ArmPitchRolly, -ArmPitchRolly)));
    armElbR = Affine3d(Translation3d(Vector3d(ArmRollElbx, 0, -ArmRollElbz)));

    prev_trans_ = Vector3d(0,0,0);
    prev_yaw_ = 0;
}

void RobotState::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_.orientation = msg->orientation;
  imu_.angular_velocity = msg->angular_velocity;
  imu_.linear_acceleration = msg->linear_acceleration;
}

void RobotState::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  ros::Subscriber gazebo_imu = ros_node.subscribe("/arduino_controller/imu", 10, &RobotState::ImuCallback, this);
  base2d_ = ros_node.advertise<geometry_msgs::Pose2D>("/robot_state/base_odometry", 10);
  base_pose_ = ros_node.advertise<geometry_msgs::Pose>("/robot_state/base_pose", 10);
  camera_pub_ = ros_node.advertise<geometry_msgs::PoseStamped>("/robot_state/camera_pose", 10);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);

}

void RobotState::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                         std::map<std::string, robotis_framework::Sensor *> sensors)
{

    // present angle
    for (std::map<std::string, int>::iterator table_iter = joint_table_.begin();
         table_iter != joint_table_.end(); table_iter++)
    {
      std::string _joint_name = table_iter->first;

      robotis_framework::Dynamixel *dxl = NULL;
      std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(_joint_name);
      if (dxl_it != dxls.end())
        dxl = dxl_it->second;
      else
        continue;

      joint_pos_[_joint_name] = dxl->dxl_state_->goal_position_;
    }

    base.linear() = mat_reset;
    hipYawL.linear() =  hipYawR.linear() = mat_reset;
    hipLeft.linear() = kneeLeft.linear() = ankleLeft.linear() = footLeft.linear() = mat_reset;
    hipRight.linear() = kneeRight.linear() = ankleRight.linear() = footRight.linear() = mat_reset;
    head_pan.linear() = head_tilt.linear() = mat_reset;

    armPitchL.linear() = armRollL.linear() =  armElbL.linear() = mat_reset;
    armPitchR.linear() = armRollR.linear() =  armElbR.linear() = mat_reset;

    geometry_msgs::Quaternion base_rot = imu_.orientation;

    base.rotate(Eigen::Quaterniond(base_rot.w, base_rot.x, base_rot.y, base_rot.z));
    hipYawL.rotate(AngleAxisd(joint_pos_["l_hip_yaw"], Eigen::Vector3d(0,0,1)));
    hipLeft.rotate(AngleAxisd(joint_pos_["l_hip_roll"], Vector3d(1,0,0)) *
                   AngleAxisd(joint_pos_["l_hip_pitch"], Vector3d(0,-1,0)));
    kneeLeft.rotate(AngleAxisd(joint_pos_["l_knee"], Vector3d(0,-1,0)));
    ankleLeft.rotate(AngleAxisd(joint_pos_["l_ank_pitch"], Vector3d(0,1,0)) *
                     AngleAxisd(joint_pos_["l_ank_roll"], Vector3d(-1,0,0)));

    hipYawR.rotate(AngleAxisd(joint_pos_["r_hip_yaw"], Eigen::Vector3d(0,0,1)));
    hipRight.rotate(AngleAxisd(joint_pos_["r_hip_roll"], Vector3d(1,0,0)) *
                   AngleAxisd(joint_pos_["r_hip_pitch"], Vector3d(0,1,0)));
    kneeRight.rotate(AngleAxisd(joint_pos_["r_knee"], Vector3d(0,1,0)));
    ankleRight.rotate(AngleAxisd(joint_pos_["r_ank_pitch"], Vector3d(0,-1,0)) *
                     AngleAxisd(joint_pos_["r_ank_roll"], Vector3d(-1,0,0)));

    head_pan.rotate(Eigen::AngleAxisd(joint_pos_["head_pan"], Eigen::Vector3d(0, 0, -1)));
    head_tilt.rotate(Eigen::AngleAxisd(joint_pos_["head_tilt"], Eigen::Vector3d(0, -1, 0)));

    Affine3d right_foot;
    Affine3d left_foot;
    Affine3d base_temp;
    Affine3d cam;

    left_foot = base*hipYawL*hipLeft*kneeLeft*ankleLeft*footLeft;
    right_foot = base*hipYawR*hipRight*kneeRight*ankleRight*footRight;

    int foot_stance = 0;
    Affine3d base_inv_, base_pose_3d;
    Vector3d dif_pose_ = Vector3d(0,0,0);
    Vector3d legToBase = Vector3d(0,0,0);
    Vector3d temp = Vector3d(0,0,0);
    double dif_yaw_ = 0;
    geometry_msgs::Pose2D base_2d;
    geometry_msgs::Pose base_3d;
    geometry_msgs::PoseStamped cam_pose;


    foot_stance = (left_foot.translation().z() < right_foot.translation().z()) ? 0 : 1;

    if(foot_stance == 0)
        base_inv_ = left_foot.inverse();
    else
        base_inv_ = right_foot.inverse();

    legToBase = base_inv_*base.translation();

    base_temp.linear() = base.linear();
//    temp = base_inv_.translation();
    base_temp.translation() = Vector3d(0,0,legToBase.z());
    cam = base_temp*head_pan*head_tilt*camera;

    base_pose_3d.translation() = legToBase;
    base_pose_3d.linear() = base.linear();

    //6D Base Pose re-check the base relative to foot and world
    tf::poseEigenToMsg(base_pose_3d, base_3d);
    base_pose_.publish(base_3d);

    //6D Camera Pose
    tf::poseEigenToMsg(cam, cam_pose.pose);

    cam_pose.header.stamp = ros::Time::now();
    cam_pose.header.frame_id = "Camera";

    camera_pub_.publish(cam_pose);

    Vector3d heading = robotis_framework::convertQuaternionToRPY(Eigen::Quaterniond(base_rot.w, base_rot.x, base_rot.y, base_rot.z));

    //3D Pose of Base relative to foot

    //Odometry Information
    dif_pose_ = legToBase - prev_trans_;
    dif_yaw_ = heading.z() - prev_yaw_;

    base_2d.x = dif_pose_.x(); base_2d.y = dif_pose_.y(); base_2d.theta = dif_yaw_;

    base2d_.publish(base_2d);

    ROS_DEBUG_COND(DEBUG_PRINT, "Updating Robot State");

    prev_trans_ = legToBase; prev_yaw_ = heading.z();

}



}
