#include "alfarobi/goalpost_tracker.h"

namespace robotis_op
{

GoalpostTracker::GoalpostTracker()
    : nh_(ros::this_node::getName()),
      p_pan(0.2),
      d_pan(0.001),
      p_tilt(0.2),
      d_tilt(0.001),
      tiltdir(1),
      pandir(1),
      FOV_WIDTH(61.25),
      FOV_HEIGHT(41.60),
      use_head_scan_(true),
      on_tracking_(false),
      current_goalpost_pan_(0),
      current_goalpost_tilt_(0),
      current_goalpost_bottom_(0),
      tracking_status_(NotFound),
      DEBUG_PRINT(false)
{
  head_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states_offset", 0);
  head_scan_pub_ = nh_.advertise<std_msgs::String>("/robotis/head_control/scan_command", 1);
  predicted_goalpost_pub_ = nh_.advertise<std_msgs::Bool>("/alfarobi/prediction_status", 2);

  robot_state_sub_ = nh_.subscribe("/v9_localization_node/robot_state",1, &GoalpostTracker::robotStateCallback, this); 
  goalpost_position_sub_ = nh_.subscribe("/v10_goalpost_detector_node/goalpost_pos", 1, &GoalpostTracker::goalpostPositionCallback, this);
  goalpost_projected_sub_ = nh_.subscribe("/v10_projection_node/projected_goalpost",1, &GoalpostTracker::goalpostProjectedCallback, this);
  goalpost_tracking_command_sub_ = nh_.subscribe("/goalpsot_tracker/command", 1, &GoalpostTracker::goalpostTrackerCommandCallback, this);
  present_joint_states_sub_ = nh_.subscribe("/robotis/present_joint_states", 5, &GoalpostTracker::presentJointStatesCallback, this);
  camera_info_sub_ = nh_.subscribe("/goalpost_detector_node/camera_info", 10, &GoalpostTracker::cameraInfoCallback, this);

  nh_.param<double>("FOV_WIDTH", FOV_WIDTH, 0.0);
  nh_.param<double>("FOV_HEIGHT", FOV_HEIGHT, 0.0);

  nh_.param<double>("pan_p", p_pan, 0.0);
  nh_.param<double>("pan_d", d_pan, 0.0);
  nh_.param<double>("pan_dir", pandir, 1.0);

  nh_.param<double>("tilt_p", p_tilt, 0.0);
  nh_.param<double>("tilt_d", d_tilt, 0.0);
  nh_.param<double>("tilt_dir", tiltdir, 1.0);

  nh_.param<bool>("penalty", penalty, false);
}

GoalpostTracker::~GoalpostTracker()
{

}

void GoalpostTracker::robotStateCallback(const geometry_msgs::PoseStampedConstPtr &_msg)
{
    int x, y, theta;
    x     = _msg->pose.position.x;
    y     = _msg->pose.position.y;
    theta = _msg->pose.orientation.z;

    double current_pos = (theta > 0) ? theta : 360 + theta;

    if(current_pos >= 360) current_pos -= 360;
    else if (current_pos < 0) current_pos += 360;

    robotState_theta = current_pos <= 180 ? current_pos : current_pos - 360;
    if(penalty) robotState_theta=0;

    if(x != 999 || y != 999)
    {
      robotState_x     = x - 520;
      robotState_y     = y - 370;
    }
    else
    {
      robotState_x     = 999;
      robotState_y     = 999;
    }
}

void GoalpostTracker::goalpostPositionCallback(const geometry_msgs::PointConstPtr &_msg)
{
    goalpostPoint_x = _msg->x;
    goalpostPoint_y = _msg->y;
    goalpostPoint_z = _msg->z;

    goalpost_position_.x = (_msg->x-320)/320;
    goalpost_position_.y = (_msg->y-240)/240;
    goalpost_position_.z = _msg->z;
}

void GoalpostTracker::goalpostProjectedCallback(const geometry_msgs::PointConstPtr &_msg)
{
    int theta;
    goalpostPos_x     = _msg->x;
    goalpostPos_y     = _msg->y;
    theta = _msg->z;

    double current_pos = (theta) > 0 ? (theta ) : 360 + (theta);
 
    if(current_pos >= 360) current_pos -= 360;
    else if (current_pos < 0) current_pos += 360;
      
    goalpostPos_theta = current_pos <= 180 ? current_pos : current_pos - 360;
    if(penalty) goalpostPos_theta=0;
}

void GoalpostTracker::goalpostTrackerCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "start")
  {
    startTracking();
  }
  else if (msg->data == "stop")
  {
    stopTracking();
  }
  else if (msg->data == "toggle_start")
  {
    if (on_tracking_ == false)
      startTracking();
    else
      stopTracking();
  }
}

void GoalpostTracker::presentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double pan, tilt;
  int get_count = 0;

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    if (msg->name[ix] == "head_pan")
    {
      pan = msg->position[ix];
      get_count += 1;
    }
    else if (msg->name[ix] == "head_tilt")
    {
      tilt = msg->position[ix];
      get_count += 1;
    }

    if (get_count == 2)
      break;
  }
  head_tilt_ = tilt;
  head_pan_ = pan;
}

void GoalpostTracker::cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
  camera_info_ = msg;
}

void GoalpostTracker::startTracking()
{
  on_tracking_ = true;
  ROS_INFO_COND(DEBUG_PRINT, "Start Goalpost tracking");
}

void GoalpostTracker::stopTracking()
{
  on_tracking_ = false;
  ROS_INFO_COND(DEBUG_PRINT, "Stop Goalpost tracking");

  double x_error = -atan(goalpost_position_.x * tan(FOV_WIDTH * M_PI / 180));
  double y_error = -atan(goalpost_position_.y * tan(FOV_HEIGHT * M_PI / 180));
  publishHeadJoint(x_error, y_error);
}

void GoalpostTracker::setUsingHeadScan(bool use_scan)
{
  use_head_scan_ = use_scan;
}

int GoalpostTracker::processTracking(int scanState)
{
  int tracking_status = Found;

  if (on_tracking_ == false)
    return DontTrack;

  if (goalpostPoint_x == -1 || goalpostPoint_y == -1)
  {
    scanGoalpost(scanState);
    tracking_status = NotFound;
  }

  // if goalpost is found
  // convert goalpost position to desired angle(rad) of head
  // goalpost_position : top-left is (-1, -1), bottom-right is (+1, +1)
  // offset_rad : top-left(+, +), bottom-right(-, -)
  double x_error = 0.0, y_error = 0.0, goalpost_size = 0.0;

  switch (tracking_status)
  {
    case NotFound:
      tracking_status_ = tracking_status;
      return tracking_status;

    case Found:
      if(scanState == Keeper)
      {
        x_error = 0;
        y_error = 0;
      }
      else
      {
        x_error = atan(goalpost_position_.x * tan(FOV_WIDTH * M_PI / 180));
        y_error = atan(goalpost_position_.y * tan(FOV_HEIGHT * M_PI / 180));
      }
      break;

    default:
      break;
  }

  double x_error_diff = x_error - current_goalpost_pan_;
  double y_error_diff = y_error - current_goalpost_tilt_;
  double x_error_target = x_error * p_pan + x_error_diff * d_pan;
  double y_error_target = y_error * p_tilt + y_error_diff * d_tilt;

  publishHeadJoint(pandir * x_error_target, tiltdir * y_error_target);

  current_goalpost_pan_ = x_error;
  current_goalpost_tilt_ = y_error;

  tracking_status_ = tracking_status;
  return tracking_status;
}

void GoalpostTracker::publishHeadJoint(double pan, double tilt)
{
  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(pan);
  head_angle_msg.position.push_back(tilt);

  head_joint_pub_.publish(head_angle_msg);
}

void GoalpostTracker::scanGoalpost(int scanState)
{
  if (use_head_scan_ == false)
    return;

  std_msgs::String scan_msg;
  switch(scanState)
  {
    case Keeper:
     scan_msg.data = "scanKeeper";
     break;
    case Sweep:
     scan_msg.data = "scanSweep";
     break;
    case Nod:
     scan_msg.data = "scanNod";
     break;
    case Square:
     scan_msg.data = "scanSquare";
     break;
    case Line:
     scan_msg.data = "scanLine";
     break;
  }

  head_scan_pub_.publish(scan_msg);
}

void GoalpostTracker::predictedGoalpost(bool use)
{
    std_msgs::Bool predicted_msg;
    predicted_msg.data = use;

    predicted_goalpost_pub_.publish(predicted_msg);
}

}

