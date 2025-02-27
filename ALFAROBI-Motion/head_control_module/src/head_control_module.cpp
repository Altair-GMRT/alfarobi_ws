/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Kayman Jung */

#include <stdio.h>
#include "op3_head_control_module/head_control_module.h"

namespace robotis_op
{

HeadControlModule::HeadControlModule()
  : control_cycle_msec_(0),
    stop_process_(false),
    is_moving_(false),
    is_direct_control_(true),
    tra_count_(0),
    tra_size_(0),
    moving_time_(0.05),
    scan_state_(NoScan),
    DEBUG(false)
{
  enable_ = false;
  module_name_ = "head_control_module";
  control_mode_ = robotis_framework::PositionControl;

  result_["head_pan"] = new robotis_framework::DynamixelState();
  result_["head_tilt"] = new robotis_framework::DynamixelState();

  using_joint_name_["head_pan"] = 0;
  using_joint_name_["head_tilt"] = 1;

  max_angle_[using_joint_name_["head_pan"]] = 80 * DEGREE2RADIAN;
  min_angle_[using_joint_name_["head_pan"]] = -80 * DEGREE2RADIAN;
  max_angle_[using_joint_name_["head_tilt"]] = 90 * DEGREE2RADIAN;
  min_angle_[using_joint_name_["head_tilt"]] = 0 * DEGREE2RADIAN;

  target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  current_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  joint_axis_direction_ = Eigen::MatrixXi::Zero(1, result_.size());

  last_msg_time_ = ros::Time::now();
}

HeadControlModule::~HeadControlModule()
{
  queue_thread_.join();
}

void HeadControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&HeadControlModule::queueThread, this));

  control_cycle_msec_ = control_cycle_msec;

  ros::NodeHandle ros_node;

  joint_axis_direction_ << 1, 1;

  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);
}

void HeadControlModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  ros::Subscriber set_head_joint_sub = ros_node.subscribe("/robotis/head_control/set_joint_states", 1,
                                                          &HeadControlModule::setHeadJointCallback, this);
  ros::Subscriber set_head_joint_offset_sub = ros_node.subscribe("/robotis/head_control/set_joint_states_offset", 1,
                                                                 &HeadControlModule::setHeadJointOffsetCallback, this);
  ros::Subscriber set_head_scan_sub = ros_node.subscribe("/robotis/head_control/scan_command", 1,
                                                         &HeadControlModule::setHeadScanCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);

  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void HeadControlModule::setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  setHeadJoint(msg, false);
}

void HeadControlModule::setHeadJointOffsetCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  setHeadJoint(msg, true);
}

void HeadControlModule::setHeadJoint(const sensor_msgs::JointState::ConstPtr &msg, bool is_offset)
{
  if (enable_ == false)
  {
    ROS_INFO_THROTTLE(1, "Head module is not enable.");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Not Enable");
    return;
  }

  // moving time
  moving_time_ = is_offset ? 0.1 : 1.0;               // default : 1 sec

  // set target joint angle
  target_position_ = goal_position_;        // default

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    std::string joint_name = msg->name[ix];
    std::map<std::string, int>::iterator joint_it = using_joint_name_.find(joint_name);

    if (joint_it != using_joint_name_.end())
    {
      double target_position = 0.0;
      int joint_index = joint_it->second;

      // set target position
      if (is_offset == true)
        target_position = goal_position_.coeff(0, joint_index) + msg->position[ix];
      else
        target_position = msg->position[ix];

      // check angle limit
      checkAngleLimit(joint_index, target_position);

      // apply target position
      target_position_.coeffRef(0, joint_index) = target_position;

      // set time
      double angle_unit = 35 * M_PI / 180;
      double calc_moving_time = fabs(goal_position_.coeff(0, joint_index) - target_position_.coeff(0, joint_index))
          / angle_unit;
      if (calc_moving_time > moving_time_)
        moving_time_ = calc_moving_time;

      if (DEBUG)
        std::cout << "joint : " << joint_name << ", Index : " << joint_index << ", Angle : " << msg->position[ix]
                     << ", Time : " << moving_time_ << std::endl;
    }
  }

  // set init joint vel, accel
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  if (is_moving_ == true)
  {
    goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result_.size());
    goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1, result_.size());
  }

  // set mode
  is_direct_control_ = true;
  scan_state_ = NoScan;

  // generate trajectory
  tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
  delete tra_gene_thread_;
}

void HeadControlModule::setHeadScanCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
  {
    // ROS_ERROR_THROTTLE(1, "Head control module is not enabled, scan command is canceled.");
    return;
  }
  // else
  //   ROS_INFO_THROTTLE(1, "Scan command is accepted. [%d]", scan_state_);

  if ((msg->data == "scanKeeper" || msg->data == "scanSweep" || msg->data == "scanNod" || msg->data == "scanSquare" || msg->data == "scanZoro" || msg->data == "scanNdungkluk") && scan_state_ == NoScan)
  {
      if(msg->data == "scanKeeper"){ 
        scan_state_ = LeftUp;
        timeHead = 0.7;
      }else if(msg->data == "scanSweep"){
        scan_state_ = LeftUp;
        timeHead = 0.7;
      }else if(msg->data == "scanNod"){
      	scan_state_ = ToUp;
        timeHead = 0.7;
      }else if(msg->data == "scanSquare"){
        scan_state_ = LeftUp;
        timeHead = 0.420;
      }else if(msg->data == "scanZoro"){
        scan_state_ = LeftCenter;
        timeHead = 0.7;
      }else if(msg->data == "scanNdungkluk"){
        scan_state_ = ForNdungkluk;
        timeHead = 0.7;
      }

    is_direct_control_ = false;

    generateScanTra(scan_state_);
  }
  else if (msg->data == "stop")
  {
    scan_state_ = NoScan;
  }

  if(msg->data == "scanKeeper"){
    next_state_ = Sweep;
    timeHead = 0.7;
  }else if(msg->data == "scanSweep"){
          
    next_state_ = Sweep;
    timeHead = 0.7;
  }else if(msg->data == "scanNod"){
      	  
    next_state_ = Nod;
    timeHead = 0.7;
  }else if(msg->data == "scanSquare"){
          
    next_state_ = Square;
    timeHead = 0.4646;
  }else if(msg->data == "scanZoro"){
    next_state_ = Zoro;
    timeHead = 0.7;
  }else if(msg->data == "scanNdungkluk"){
    next_state_ = Ndungkluk;
    timeHead = 0.7;
  }
}

bool HeadControlModule::checkAngleLimit(const int joint_index, double &goal_position)
{
  std::map<int, double>::iterator angle_it = min_angle_.find(joint_index);
  if (angle_it == min_angle_.end())
    return false;
  double min_angle = angle_it->second;

  angle_it = max_angle_.find(joint_index);
  if (angle_it == max_angle_.end())
    return false;
  double max_angle = angle_it->second;

  if (goal_position < min_angle)
    goal_position = min_angle;
  if (goal_position > max_angle)
    goal_position = max_angle;

  return true;
}

void HeadControlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  tra_lock_.lock();

  // get joint data
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];

    robotis_framework::Dynamixel *_dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      _dxl = dxl_it->second;
    else
      continue;

    current_position_.coeffRef(0, index) = _dxl->dxl_state_->present_position_;
    goal_position_.coeffRef(0, index) = _dxl->dxl_state_->goal_position_;
  }

  // check to stop
  if (stop_process_ == true)
  {
    stopMoving();
  }
  else
  {
    // process
    if (tra_size_ != 0)
    {
      // start of steps
      if (tra_count_ == 0)
      {
        startMoving();
      }

      // end of steps
      if (tra_count_ >= tra_size_)
      {
        finishMoving();
      }
      else
      {
        // update goal position
        goal_position_ = calc_joint_tra_.block(tra_count_, 0, 1, result_.size());
        tra_count_ += 1;
      }
    }
  }
  tra_lock_.unlock();

  // set joint data
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];
    double goal_position = goal_position_.coeff(0, index) * joint_axis_direction_(index);
    checkAngleLimit(index, goal_position);

    result_[joint_name]->goal_position_ = goal_position;
  }
}

void HeadControlModule::stop()
{
  tra_lock_.lock();

  if (is_moving_ == true)
    stop_process_ = true;

  tra_lock_.unlock();

  return;
}

bool HeadControlModule::isRunning()
{
  return is_moving_;
}

void HeadControlModule::onModuleEnable()
{
  scan_state_ = NoScan;
  ROS_INFO("Head Control Module is Enabled");
}

void HeadControlModule::onModuleDisable()
{

}

void HeadControlModule::startMoving()
{
  is_moving_ = true;

  // start procedure
}

void HeadControlModule::finishMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_direct_control_ = true;
  is_moving_ = false;

  // log
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Head movement is finished.");

  if (DEBUG)
    std::cout << "Trajectory End" << std::endl;
  
  // std::cout<<"NEXT STATE_ : "<<next_state_<<std::endl;
  // type 1 head scan
  switch (scan_state_)
  {
    case LeftUp:
      switch(next_state_)
      {
        case Nod:
          scan_state_ = ToUp;
          break;

        case Sweep:
          scan_state_ = RightUp;
          break;

        case Square:
          scan_state_ = RightUp;
          break;

        // case Zoro:
        //   scan_state_ = RightUp;
        //   break;
      }
      break;

    case RightUp:
      switch(next_state_)
      {
        case Nod:
          scan_state_ = ToUp;
          break;

        case Sweep:
          scan_state_ = RightCenter;
          break;

        case Square:
          scan_state_ = RightCenter;
          break;

        // case Zoro:
        //   scan_state_ = LeftBottom;
        //   break;
      }

      last_state = 0;
      break;

    case RightCenter:
      switch(next_state_)
      {
        case Nod:
          scan_state_ = ToUp;
          break;

        case Sweep:
          scan_state_ = LeftCenter;
          break;

        case Square:
          scan_state_ = LeftCenter;
          break;
        
        case Zoro:
          scan_state_ = LeftBottom;
      }
      break;

    case LeftCenter:
      switch(next_state_)
      {
        case Nod:
          scan_state_ = ToUp;
          break;

        case Sweep:
          // if(last_state == 0)
          //   scan_state_ = LeftBottom;
          //else
            scan_state_ = LeftUp;

          break;

        case Square:
          scan_state_ = LeftUp;
          break;
        
        case Zoro:
          scan_state_ = RightCenter;
      }
      break;

    case LeftBottom:
      switch(next_state_)
      {
        case Nod:
          scan_state_ = ToDown;
          break;

        case Sweep:
          scan_state_ = RightBottom;
          break;

        case Square:
          scan_state_ = LeftCenter;
          break;

        case Zoro:
        scan_state_ = RightBottom;
      }
      break;

    case RightBottom:
      switch(next_state_)
      {
        case Nod:
          scan_state_ = ToDown;
          break;

        case Sweep:
          scan_state_ = RightCenter;
          break;

        case Square:
          scan_state_ = RightCenter;
          break;

        case Zoro:
          scan_state_ = LeftCenter;
          break;

      }
      last_state = 1;
      break;

    case ToUp:
      switch(next_state_)
      {
        case Nod:
          scan_state_ = ToDown;
          break;

        case Sweep:
          scan_state_ = RightCenter;
          break;

        case Square:
          scan_state_ = RightCenter;
          break;
      }
      break;

    case ToDown:
      switch(next_state_)
      {
        case Nod:
          scan_state_ = ToUp;
          break;

        case Sweep:
          scan_state_ = RightBottom;
          break;

        case Square:
          scan_state_ = RightCenter;
          break;
      }
      break;

    case ForNdungkluk:
      switch(next_state_)
      {
        case Ndungkluk:
          scan_state_ = ForNdungkluk;
          break;
      }
      break;
  }

  // ROS_INFO_THROTTLE(1, "Current State [%d]", scan_state_);

  generateScanTra(scan_state_);
}

void HeadControlModule::stopMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_moving_ = false;
  is_direct_control_ = true;
  stop_process_ = false;
  scan_state_ = NoScan;

  // log
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Stop Module.");
}

void HeadControlModule::generateScanTra(const int head_direction)
{
  switch (head_direction)
  {
    case LeftUp:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0.8;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.5;
      break;
    }

    case RightUp:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = max_angle_[using_joint_name_["head_pan"]] * 0.8;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.5;
      break;
    }

    case RightCenter:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = max_angle_[using_joint_name_["head_pan"]] * 0.8;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = max_angle_[using_joint_name_["head_tilt"]] * 0.3;
      break;
    }

    case LeftCenter:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0.8;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = max_angle_[using_joint_name_["head_tilt"]] * 0.3;
      break;
    }

    case LeftBottom:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0.8;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = max_angle_[using_joint_name_["head_tilt"]] * 0.8;
      break;
    }

    case RightBottom:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = max_angle_[using_joint_name_["head_pan"]] * 0.8;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = max_angle_[using_joint_name_["head_tilt"]] * 0.8;
      break;
    }

    case ToUp:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.5;
      break;
    }

    case ToDown:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = max_angle_[using_joint_name_["head_tilt"]] * 0.8;
      break;
    }

    case ForNdungkluk:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = max_angle_[using_joint_name_["head_tilt"]] * 0.68;
      break;
    }

    default:
      return;
  }

  //set moving time
  //moving_time_ = 1.0;               // default : 1 sec

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];

    // set time
    int calc_moving_time = fabs(goal_position_.coeff(0, index) - target_position_.coeff(0, index)) / timeHead;
    if (calc_moving_time > moving_time_)
    {
      moving_time_ = calc_moving_time;
    }
    else
    {
      if(calc_moving_time > 0.05)
      {
        moving_time_ = calc_moving_time;
      }
    }
    
  }

  // set init joint vel, accel
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  if (is_moving_ == true)
  {
    goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result_.size());
    goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1, result_.size());
  }

  // generate trajectory
  tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
  delete tra_gene_thread_;
}

/*
 simple minimum jerk trajectory

 pos_start : position at initial state
 vel_start : velocity at initial state
 accel_start : acceleration at initial state

 pos_end : position at final state
 vel_end : velocity at final state
 accel_end : acceleration at final state

 smp_time : sampling time

 mov_time : movement time
 */
Eigen::MatrixXd HeadControlModule::calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start,
                                                         double pos_end, double vel_end, double accel_end,
                                                         double smp_time, double mov_time)
{
  Eigen::MatrixXd poly_matrix(3, 3);
  Eigen::MatrixXd poly_vector(3, 1);

  poly_matrix << robotis_framework::powDI(mov_time, 3), robotis_framework::powDI(mov_time, 4), robotis_framework::powDI(
        mov_time, 5), 3 * robotis_framework::powDI(mov_time, 2), 4 * robotis_framework::powDI(mov_time, 3), 5
      * robotis_framework::powDI(mov_time, 4), 6 * mov_time, 12 * robotis_framework::powDI(mov_time, 2), 20
      * robotis_framework::powDI(mov_time, 3);

  poly_vector << pos_end - pos_start - vel_start * mov_time - accel_start * pow(mov_time, 2) / 2, vel_end - vel_start
      - accel_start * mov_time, accel_end - accel_start;

  Eigen::MatrixXd poly_coeff = poly_matrix.inverse() * poly_vector;

  int all_time_steps = round(mov_time / smp_time + 1);

  Eigen::MatrixXd time = Eigen::MatrixXd::Zero(all_time_steps, 1);
  Eigen::MatrixXd minimum_jer_tra = Eigen::MatrixXd::Zero(all_time_steps, 3);

  for (int step = 0; step < all_time_steps; step++)
    time.coeffRef(step, 0) = step * smp_time;

  for (int step = 0; step < all_time_steps; step++)
  {
    // position
    minimum_jer_tra.coeffRef(step, 0) = pos_start + vel_start * time.coeff(step, 0)
        + 0.5 * accel_start * robotis_framework::powDI(time.coeff(step, 0), 2)
        + poly_coeff.coeff(0, 0) * robotis_framework::powDI(time.coeff(step, 0), 3)
        + poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 4)
        + poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 5);
    // velocity
    minimum_jer_tra.coeffRef(step, 1) = vel_start + accel_start * time.coeff(step, 0)
        + 3 * poly_coeff.coeff(0, 0) * robotis_framework::powDI(time.coeff(step, 0), 2)
        + 4 * poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 3)
        + 5 * poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 4);
    // accel
    minimum_jer_tra.coeffRef(step, 2) = accel_start + 6 * poly_coeff.coeff(0, 0) * time.coeff(step, 0)
        + 12 * poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 2)
        + 20 * poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 3);
  }

  return minimum_jer_tra;
}

void HeadControlModule::jointTraGeneThread()
{
  tra_lock_.lock();

  double smp_time = control_cycle_msec_ * 0.001;		// ms -> s
  int all_time_steps = int(moving_time_ / smp_time) + 1;

  // for debug
  try
  {
    calc_joint_tra_.resize(all_time_steps, result_.size());
    calc_joint_vel_tra_.resize(all_time_steps, result_.size());
    calc_joint_accel_tra_.resize(all_time_steps, result_.size());
  }
  catch(std::exception &e)
  {
    std::cout << "All step tile : " << all_time_steps << std::endl;
    std::cout << e.what() << std::endl;
    throw;
  }

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];

    double ini_value = goal_position_.coeff(0, index);
    double ini_vel = goal_velocity_.coeff(0, index);
    double ini_accel = goal_acceleration_.coeff(0, index);

    double tar_value = target_position_.coeff(0, index);

    Eigen::MatrixXd tra = calcMinimumJerkTraPVA(ini_value, ini_vel, ini_accel, tar_value, 0.0, 0.0, smp_time,
                                                moving_time_);

    calc_joint_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 0, all_time_steps, 1);
    calc_joint_vel_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 1, all_time_steps, 1);
    calc_joint_accel_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 2, all_time_steps, 1);
  }

  tra_size_ = calc_joint_tra_.rows();
  tra_count_ = 0;

  if (DEBUG)
    ROS_INFO("[ready] make trajectory : %d, %d", tra_size_, tra_count_);

  tra_lock_.unlock();
}

void HeadControlModule::publishStatusMsg(unsigned int type, std::string msg)
{
  ros::Time now = ros::Time::now();

  if(msg.compare(last_msg_) == 0)
  {
    ros::Duration dur = now - last_msg_time_;
    if(dur.sec < 1)
      return;
  }

  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = now;
  status_msg.type = type;
  status_msg.module_name = "Head Control";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);

  last_msg_ = msg;
  last_msg_time_ = now;
}
}
