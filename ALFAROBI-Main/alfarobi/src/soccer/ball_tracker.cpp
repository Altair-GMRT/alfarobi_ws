#include "alfarobi/ball_tracker.h"

namespace robotis_op
{

BallTracker::BallTracker()
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
      current_ball_pan_(0),
      current_ball_tilt_(0),
      current_ball_bottom_(0),
      tracking_status_(NotFound),
      DEBUG_PRINT(false)
{
  head_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states_offset", 0);
  head_scan_pub_ = nh_.advertise<std_msgs::String>("/robotis/head_control/scan_command", 1);
  predicted_ball_pub_ = nh_.advertise<std_msgs::Bool>("/alfarobi/prediction_status", 2);
  phytagorean_dist_pub_ = nh_.advertise<std_msgs::Float32>("/alfarobi/ball_dist", 10);

  robot_state_sub_ = nh_.subscribe("/v9_localization_node/robot_state",1, &BallTracker::robotStateCallback, this); 
  ball_position_sub_ = nh_.subscribe("/v9_ball_detector_node/ball_pos", 1, &BallTracker::ballPositionCallback, this);
  ball_projected_sub_ = nh_.subscribe("/v9_localization_node/projected_ball",1, &BallTracker::ballProjectedCallback, this);
  ball_tracking_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &BallTracker::ballTrackerCommandCallback, this);
  present_joint_states_sub_ = nh_.subscribe("/robotis/present_joint_states", 5, &BallTracker::presentJointStatesCallback, this);
  camera_info_sub_ = nh_.subscribe("/ball_detector_node/camera_info", 10, &BallTracker::cameraInfoCallback, this);

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

BallTracker::~BallTracker()
{

}

void BallTracker::robotStateCallback(const geometry_msgs::PoseStampedConstPtr &_msg)
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

void BallTracker::ballPositionCallback(const geometry_msgs::PointConstPtr &_msg)
{
    ballPoint_x = _msg->x;
    ballPoint_y = _msg->y;
    ballPoint_z = _msg->z;

    ball_position_.x = (_msg->x-320)/320;
    ball_position_.y = (_msg->y-240)/240;
    ball_position_.z = _msg->z;
}

void BallTracker::ballProjectedCallback(const geometry_msgs::PointConstPtr &_msg)
{
    int theta;
    ballPos_x     = _msg->x;
    ballPos_y     = _msg->y;
    theta         = _msg->z;

    ball_dist     = (float)sqrt(pow(ballPos_x, 2) + pow(ballPos_y, 2));
    std_msgs::Float32 msg;
    msg.data = ball_dist;

    phytagorean_dist_pub_.publish(msg);

    double current_pos = (theta) > 0 ? (theta ) : 360 + (theta);
 
    if(current_pos >= 360) current_pos -= 360;
    else if (current_pos < 0) current_pos += 360;
      
    ballPos_theta = current_pos <= 180 ? current_pos : current_pos - 360;
    if(penalty) ballPos_theta=0;
}

void BallTracker::ballTrackerCommandCallback(const std_msgs::String::ConstPtr &msg)
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

void BallTracker::presentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
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

void BallTracker::cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
  camera_info_ = msg;
}

void BallTracker::startTracking()
{
  on_tracking_ = true;
  ROS_INFO_COND(DEBUG_PRINT, "Start Ball tracking");
}

void BallTracker::stopTracking()
{
  on_tracking_ = false;
  ROS_INFO_COND(DEBUG_PRINT, "Stop Ball tracking");

  double x_error = -atan(ball_position_.x * tan(FOV_WIDTH * M_PI / 180));
  double y_error = -atan(ball_position_.y * tan(FOV_HEIGHT * M_PI / 180));
  publishHeadJoint(x_error, y_error);
}

void BallTracker::setUsingHeadScan(bool use_scan)
{
  use_head_scan_ = use_scan;
}

int BallTracker::processTracking(int scanState)
{
  int tracking_status = Found;

  if (on_tracking_ == false)
    return DontTrack;

  if (ballPoint_x == -1 || ballPoint_y == -1)
  {
    scanBall(scanState);
    tracking_status = NotFound;
  }

  // if ball is found
  // convert ball position to desired angle(rad) of head
  // ball_position : top-left is (-1, -1), bottom-right is (+1, +1)
  // offset_rad : top-left(+, +), bottom-right(-, -)
  double x_error = 0.0, y_error = 0.0, ball_size = 0.0;

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
        x_error = atan(ball_position_.x * tan(FOV_WIDTH * M_PI / 180));
        y_error = atan(ball_position_.y * tan(FOV_HEIGHT * M_PI / 180));
      }
      break;

    default:
      break;
  }

  double x_error_diff = x_error - current_ball_pan_;
  double y_error_diff = y_error - current_ball_tilt_;
  double x_error_target = x_error * p_pan + x_error_diff * d_pan;
  double y_error_target = y_error * p_tilt + y_error_diff * d_tilt;

  publishHeadJoint(pandir * x_error_target, tiltdir * y_error_target);

  current_ball_pan_ = x_error;
  current_ball_tilt_ = y_error;

  tracking_status_ = tracking_status;
  return tracking_status;
}

void BallTracker::publishHeadJoint(double pan, double tilt)
{
  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(pan);
  head_angle_msg.position.push_back(tilt);

  head_joint_pub_.publish(head_angle_msg);
}

void BallTracker::scanBall(int scanState)
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
     scan_msg.data = "scanZoro";
     break;
    case Ndungkluk:
     scan_msg.data = "scanNdungkluk";
     break;
  }

  head_scan_pub_.publish(scan_msg);
}

void BallTracker::predictedBall(bool use)
{
    std_msgs::Bool predicted_msg;
    predicted_msg.data = use;

    predicted_ball_pub_.publish(predicted_msg);
}

}

