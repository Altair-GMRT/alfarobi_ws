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

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/op3_gui_demo/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace robotis_op
{

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent),
    qnode_op3_(argc, argv),
    is_updating_(false),
    is_walking_(false)
{
  // code to DEBUG
  debug_ = false;

  if (argc >= 2)
  {
    std::string arg_code(argv[1]);
    if (arg_code == "debug")
      debug_ = true;
    else
      debug_ = false;
  }

  ui_.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));  // qApp is a global variable for the application

  readSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  //  ui_.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode_op3_, SIGNAL(rosShutdown()), this, SLOT(close()));

  qRegisterMetaType<std::vector<int> >("std::vector<int>");
  QObject::connect(&qnode_op3_, SIGNAL(updateCurrentJointControlMode(std::vector<int>)), this,
                   SLOT(updateCurrentJointMode(std::vector<int>)));
  QObject::connect(&qnode_op3_, SIGNAL(updateHeadAngles(double,double)), this, SLOT(updateHeadAngles(double,double)));

  QObject::connect(ui_.head_pan_slider, SIGNAL(valueChanged(int)), this, SLOT(setHeadAngle()));
  QObject::connect(ui_.head_tilt_slider, SIGNAL(valueChanged(int)), this, SLOT(setHeadAngle()));

  qRegisterMetaType<op3_walking_module_msgs::WalkingParam>("op_walking_params");
  QObject::connect(&qnode_op3_, SIGNAL(updateWalkingParameters(op3_walking_module_msgs::WalkingParam)), this,
                   SLOT(updateWalkingParams(op3_walking_module_msgs::WalkingParam)));
  
  qRegisterMetaType<zmp_walking_module_msgs::ZMPWalkingParam>("zmp_walking_params");
  QObject::connect(&qnode_op3_, SIGNAL(updateZMPWalkingParameters(zmp_walking_module_msgs::ZMPWalkingParam)), this,
                   SLOT(updateZMPWalkingParams(zmp_walking_module_msgs::ZMPWalkingParam)));
  
  qRegisterMetaType<kicking_module_msgs::KickingParam>("kicking_params");
  QObject::connect(&qnode_op3_, SIGNAL(updateKickingParameters(kicking_module_msgs::KickingParam)), this,
                  SLOT(updateKickingParams(kicking_module_msgs::KickingParam)));
  
  qRegisterMetaType<feedback_msgs::FeedbackParam>("feedback_params");
  QObject::connect(&qnode_op3_, SIGNAL(updateFeedbackParameters(feedback_msgs::FeedbackParam)), this,
                  SLOT(updateFeedbackParams(feedback_msgs::FeedbackParam)));                 
  
  qRegisterMetaType<capture_step_module_msgs::WalkingParam>("zmp_walking_params");
  QObject::connect(&qnode_op3_, SIGNAL(updateCaptureStepWalkingParameters(capture_step_module_msgs::WalkingParam)), this,
                   SLOT(updateCaptureStepParams(capture_step_module_msgs::WalkingParam)));
  
  qRegisterMetaType<capture_step_module_msgs::WalkingParam>("quintic_walk_params");
  QObject::connect(&qnode_op3_, SIGNAL(updateQuinticWalkParameters(quintic_walk_msgs::WalkingParam)), this,
                   SLOT(updateQuinticWalkParams(quintic_walk_msgs::WalkingParam)));

  /*********************
   ** Logging
   **********************/
  ui_.view_logging->setModel(qnode_op3_.loggingModel());
  QObject::connect(&qnode_op3_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));


  /*********************
   ** Auto Start
   **********************/
  qnode_op3_.init();
  initModeUnit();
  setUserShortcut();
  updateModuleUI();
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_init_pose_clicked(bool check)
{
  qnode_op3_.moveInitPose();
}

// Walking
void MainWindow::on_button_init_gyro_clicked(bool check)
{
  qnode_op3_.initGyro();
}

void MainWindow::on_button_walking_start_clicked(bool check)
{
  is_walking_ = true;
  qnode_op3_.setWalkingCommand("start");
}

void MainWindow::on_button_walking_stop_clicked(bool check)
{
  is_walking_ = false;
  qnode_op3_.setWalkingCommand("stop");
}

// Kicking
void MainWindow::on_left_kick_clicked(bool check)
{
  qnode_op3_.setKickingCommand("left_kick");
}

void MainWindow::on_right_kick_clicked(bool check)
{
  qnode_op3_.setKickingCommand("right_kick");
}

void MainWindow::on_stop_kick_clicked(bool check)
{
  qnode_op3_.setKickingCommand("stop");
}

void MainWindow::on_kick_apply_button_clicked(bool check)
{
  applyKickingParams();
}

void MainWindow::on_kick_save_button_clicked(bool check)
{
  qnode_op3_.setKickingCommand("save");
}

void MainWindow::on_kick_refresh_button_clicked(bool check)
{
  qnode_op3_.refreshKickingParam();
}

// Feedback
void MainWindow::on_Left_SSP_clicked(bool check)
{
  qnode_op3_.setFeedbackCommand("left_ssp");
}

void MainWindow::on_Right_SSP_clicked(bool check)
{
  qnode_op3_.setFeedbackCommand("right_ssp");
}

void MainWindow::on_DSP_clicked(bool check)
{
  qnode_op3_.setFeedbackCommand("dsp");
}

void MainWindow::on_fb_apply_button_clicked(bool check)
{
  applyFeedbackParams();
}

void MainWindow::on_fb_save_button_clicked(bool check)
{
  qnode_op3_.setFeedbackCommand("save");
}

void MainWindow::on_fb_refresh_button_clicked(bool check)
{
  qnode_op3_.refreshFeedbackParam();
}

void MainWindow::on_zmp_start_button_clicked(bool check)
{
  is_walking_ = true;
  qnode_op3_.setZMPWalkingCommand("start");
}

void MainWindow::on_zmp_stop_button_clicked(bool check)
{
  is_walking_ = false;
  qnode_op3_.setZMPWalkingCommand("stop");
}

void MainWindow::on_zmp_reset_button_clicked(bool check)
{
  is_walking_ = true;
  qnode_op3_.setZMPWalkingCommand("reset");

}

void MainWindow::on_zmp_apply_button_clicked(bool check)
{
  applyZMPWalkingParams();
}

void MainWindow::on_zmp_save_button_clicked(bool check)
{
  qnode_op3_.setZMPWalkingCommand("save");
}

void MainWindow::on_zmp_refresh_button_clicked(bool check)
{
  qnode_op3_.refreshZMPParam();
}


void MainWindow::on_button_param_refresh_clicked(bool check)
{
  qnode_op3_.refreshWalkingParam();
}

void MainWindow::on_button_param_save_clicked(bool check)
{
  qnode_op3_.setWalkingCommand("save");
}

void MainWindow::on_button_param_apply_clicked(bool check)
{
  applyWalkingParams();
}

void MainWindow::on_checkBox_balance_on_clicked(bool check)
{
}
void MainWindow::on_checkBox_balance_off_clicked(bool check)
{
}

// Capture StepsetZMPWalkingCommand

void MainWindow::on_button_capture_step_apply_clicked(bool check)
{
    applyCaptureStepParams();
}

void MainWindow::on_button_capture_step_refresh_clicked(bool check)
{
    qnode_op3_.refreshCaptureStepParam();
}

void MainWindow::on_button_capture_step_save_clicked(bool check)
{
    qnode_op3_.setCaptureStepCommand("save");

}

void MainWindow::on_button_capture_step_start_clicked(bool check)
{
    qnode_op3_.setCaptureStepCommand("start");
}

void MainWindow::on_button_capture_step_stop_clicked(bool check)
{
    qnode_op3_.setCaptureStepCommand("stop");
}

void MainWindow::on_head_center_button_clicked(bool check)
{
  setHeadAngle(0, 0);
}

/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

// user shortcut
void MainWindow::setUserShortcut()
{
  // Setup a signal mapper to avoid creating custom slots for each tab
  QSignalMapper *_sig_map = new QSignalMapper(this);

  // Setup the shortcut for the first tab : Mode
  QShortcut *_short_tab1 = new QShortcut(QKeySequence("F1"), this);
  connect(_short_tab1, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab1, 0);

  // Setup the shortcut for the second tab : Manipulation
  QShortcut *_short_tab2 = new QShortcut(QKeySequence("F2"), this);
  connect(_short_tab2, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab2, 1);

  // Setup the shortcut for the third tab : Walking
  QShortcut *_short_tab3 = new QShortcut(QKeySequence("F3"), this);
  connect(_short_tab3, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab3, 2);

  // Setup the shortcut for the fouth tab : Head control
  QShortcut *_short_tab4 = new QShortcut(QKeySequence("F4"), this);
  connect(_short_tab4, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab4, 3);

  // Setup the shortcut for the fouth tab : Motion
  QShortcut *_short_tab5 = new QShortcut(QKeySequence("F5"), this);
  connect(_short_tab5, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab5, 4);

  // Wire the signal mapper to the tab widget index change slot
  connect(_sig_map, SIGNAL(mapped(int)), ui_.tabWidget_control, SLOT(setCurrentIndex(int)));

  QShortcut *walking_shortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
  connect(walking_shortcut, SIGNAL(activated()), this, SLOT(walkingCommandShortcut()));
}

// mode control
// it's not used now
void MainWindow::setMode(bool check)
{
  robotis_controller_msgs::JointCtrlModule _control_msg;

  QList<QComboBox *> _combo_children = ui_.widget_mode->findChildren<QComboBox *>();
  for (int ix = 0; ix < _combo_children.length(); ix++)
  {
    std::stringstream _stream;
    std::string _joint;
    int _id;

    int _control_index = _combo_children.at(ix)->currentIndex();
    // if(_control_index == QNodeThor3::Control_None) continue;

    std::string _control_mode = _combo_children.at(ix)->currentText().toStdString();

    if (qnode_op3_.getIDJointNameFromIndex(ix, _id, _joint) == true)
    {
      _stream << "[" << (_id < 10 ? "0" : "") << _id << "] " << _joint << " : " << _control_mode;

      _control_msg.joint_name.push_back(_joint);
      _control_msg.module_name.push_back(_control_mode);
    }
    else
    {
      _stream << "id " << ix << " : " << _control_mode;
    }

  }

  // no control
  if (_control_msg.joint_name.size() == 0)
    return;


  std::cout << _control_msg << std::endl;

  qnode_op3_.setJointControlMode(_control_msg);
}

void MainWindow::updateCurrentJointMode(std::vector<int> mode)
{
  QList<QComboBox *> _combo_children = ui_.widget_mode->findChildren<QComboBox *>();
  for (int ix = 0; ix < _combo_children.length(); ix++)
  {
    int _control_index = mode.at(ix);
    _combo_children.at(ix)->setCurrentIndex(_control_index);

    if (debug_)
    {
      std::stringstream _stream;
      std::string _joint;
      int _id;

      std::string _control_mode = _combo_children.at(ix)->currentText().toStdString();

      if (qnode_op3_.getIDJointNameFromIndex(ix, _id, _joint) == true)
      {
        _stream << "[" << (_id < 10 ? "0" : "") << _id << "] " << _joint << " : " << _control_mode;
      }
      else
      {
        _stream << "id " << ix << " : " << _control_mode;
      }

    }
  }

  // set module UI
  updateModuleUI();
}

void MainWindow::updateModuleUI()
{
  if (debug_)
    return;

  for (int index = 0; index < qnode_op3_.getModeSize(); index++)
  {
    std::string _mode = qnode_op3_.getModeName(index);
    if (_mode == "")
      continue;

    std::map<std::string, QList<QWidget *> >::iterator _module_iter = module_ui_table_.find(_mode);
    if (_module_iter == module_ui_table_.end())
      continue;

    bool _is_enable = qnode_op3_.isUsingModule(_mode);

    QList<QWidget *> _list = _module_iter->second;
    for (int ix = 0; ix < _list.size(); ix++)
    {
      _list.at(ix)->setEnabled(_is_enable);
    }
  }

  // refresh walking parameter
  if (qnode_op3_.isUsingModule("walking_module"))
    qnode_op3_.refreshWalkingParam();

  if(qnode_op3_.isUsingModule("walk_zmp"))
    qnode_op3_.refreshZMPParam();
  if(qnode_op3_.isUsingModule("capture_step_module"))
    qnode_op3_.refreshCaptureStepParam();
  if(qnode_op3_.isUsingModule("quintic_walk"))
    qnode_op3_.refreshQuinticWalkParam();
}

// head control
void MainWindow::updateHeadAngles(double pan, double tilt)
{
  if (ui_.head_pan_slider->underMouse() == true)
    return;
  if (ui_.head_pan_spinbox->underMouse() == true)
    return;
  if (ui_.head_tilt_slider->underMouse() == true)
    return;
  if (ui_.head_tilt_spinbox->underMouse() == true)
    return;

  is_updating_ = true;

  ui_.head_pan_slider->setValue(pan * 180.0 / M_PI);
  // ui.head_pan_spinbox->setValue( pan * 180.0 / M_PI );
  ui_.head_tilt_slider->setValue(tilt * 180.0 / M_PI);
  // ui.head_tilt_spinbox->setValue( tilt * 180.0 / M_PI );

  is_updating_ = false;
}

void MainWindow::setHeadAngle()
{
  if (is_updating_ == true)
    return;
  qnode_op3_.setHeadJoint(ui_.head_pan_slider->value() * M_PI / 180, ui_.head_tilt_slider->value() * M_PI / 180);
}

void MainWindow::setHeadAngle(double pan, double tilt)
{
  qnode_op3_.setHeadJoint(pan * M_PI / 180, tilt * M_PI / 180);
}

// walking
void MainWindow::updateWalkingParams(op3_walking_module_msgs::WalkingParam params)
{
  // init pose
  ui_.dSpinBox_init_offset_x->setValue(params.init_x_offset);
  ui_.dSpinBox_init_offset_y->setValue(params.init_y_offset);
  ui_.dSpinBox_init_offset_z->setValue(params.init_z_offset);
  ui_.dSpinBox_init_offset_roll->setValue(params.init_roll_offset * RADIAN2DEGREE);
  ui_.dSpinBox_init_offset_pitch->setValue(params.init_pitch_offset * RADIAN2DEGREE);
  ui_.dSpinBox_init_offset_yaw->setValue(params.init_yaw_offset * RADIAN2DEGREE);
  ui_.dSpinBox_hip_pitch_offset->setValue(params.hip_pitch_offset * RADIAN2DEGREE);
  // time
  ui_.dSpinBox_period_time->setValue(params.period_time * 1000);       // s -> ms
  ui_.dSpinBox_dsp_ratio->setValue(params.dsp_ratio);
  ui_.dSpinBox_step_fb_ratio->setValue(params.step_fb_ratio);
  ;
  // walking
  ui_.dSpinBox_x_move_amplitude->setValue(params.x_move_amplitude);
  ui_.dSpinBox_y_move_amplitude->setValue(params.y_move_amplitude);
  ui_.dSpinBox_z_move_amplitude->setValue(params.z_move_amplitude);
  ui_.dSpinBox_y_move_amplitude->setValue(params.angle_move_amplitude);
  ui_.checkBox_move_aim_on->setChecked(params.move_aim_on);
  ui_.checkBox_move_aim_off->setChecked(!params.move_aim_on);
  // balance
  ui_.checkBox_balance_on->setChecked(params.balance_enable);
  ui_.checkBox_balance_off->setChecked(!params.balance_enable);
  ui_.dSpinBox_hip_roll_gain->setValue(params.balance_hip_roll_gain);
  ui_.dSpinBox_knee_gain->setValue(params.balance_knee_gain);
  ui_.dSpinBox_ankle_roll_gain->setValue(params.balance_ankle_roll_gain);
  ui_.dSpinBox_ankle_pitch_gain->setValue(params.balance_ankle_pitch_gain);
  ui_.dSpinBox_y_swap_amplitude->setValue(params.y_swap_amplitude);
  ui_.dSpinBox_z_swap_amplitude->setValue(params.z_swap_amplitude);
  ui_.dSpinBox_pelvis_offset->setValue(params.pelvis_offset * RADIAN2DEGREE);
  ui_.dSpinBox_arm_swing_gain->setValue(params.arm_swing_gain);

  ui_.dspin_D_ANKLE_P->setValue(params.D_ANKLE_P);

  ui_.dspin_D_HIP_P->setValue(params.D_HIP_P);

  ui_.dspin_D_ANKLE_R->setValue(params.D_ANKLE_R);
  ui_.dspin_D_HIP_R->setValue(params.D_HIP_R);

  ui_.dspin_KP_P->setValue(params.KP_P);
  ui_.dspin_KD_P->setValue(params.KD_P);
  ui_.dspin_KI_P->setValue(params.KI_P);

  ui_.dspin_KP_R->setValue(params.KP_R);
  ui_.dspin_KD_R->setValue(params.KD_R);
  ui_.dspin_KI_R->setValue(params.KI_R);

  ui_.dspin_r_x_offset->setValue(params.r_x_offset);
  ui_.dspin_r_y_offset->setValue(params.r_y_offset);
  ui_.dspin_r_z_offset->setValue(params.r_z_offset);
  ui_.dspin_r_yaw_offset->setValue(params.r_yaw_offset);
  ui_.dspin_r_pitch_offset->setValue(params.r_pitch_offset);
  ui_.dspin_r_roll_offset->setValue(params.r_roll_offset);

  ui_.dspin_l_x_offset->setValue(params.l_x_offset);
  ui_.dspin_l_y_offset->setValue(params.l_y_offset);
  ui_.dspin_l_z_offset->setValue(params.l_z_offset);
  ui_.dspin_l_yaw_offset->setValue(params.l_yaw_offset);
  ui_.dspin_l_pitch_offset->setValue(params.l_pitch_offset);
  ui_.dspin_l_roll_offset->setValue(params.l_roll_offset);

  ui_.dspin_shoulder_gain->setValue(params.shoulder_gain);
}

void MainWindow::applyWalkingParams()
{
  op3_walking_module_msgs::WalkingParam walking_param;

  // init pose
  walking_param.init_x_offset = ui_.dSpinBox_init_offset_x->value();
  walking_param.init_y_offset = ui_.dSpinBox_init_offset_y->value();
  walking_param.init_z_offset = ui_.dSpinBox_init_offset_z->value();
  walking_param.init_roll_offset = ui_.dSpinBox_init_offset_roll->value() * DEGREE2RADIAN;
  walking_param.init_pitch_offset = ui_.dSpinBox_init_offset_pitch->value() * DEGREE2RADIAN;
  walking_param.init_yaw_offset = ui_.dSpinBox_init_offset_yaw->value() * DEGREE2RADIAN;
  walking_param.hip_pitch_offset = ui_.dSpinBox_hip_pitch_offset->value() * DEGREE2RADIAN;
  // time
  walking_param.period_time = ui_.dSpinBox_period_time->value() * 0.001;     // ms -> s
  walking_param.dsp_ratio = ui_.dSpinBox_dsp_ratio->value();
  walking_param.step_fb_ratio = ui_.dSpinBox_step_fb_ratio->value();
  ;
  // walking
  walking_param.x_move_amplitude = ui_.dSpinBox_x_move_amplitude->value();
  walking_param.y_move_amplitude = ui_.dSpinBox_y_move_amplitude->value();
  walking_param.z_move_amplitude = ui_.dSpinBox_z_move_amplitude->value();
  walking_param.angle_move_amplitude = ui_.dSpinBox_a_move_amplitude->value() * DEGREE2RADIAN;
  walking_param.move_aim_on = ui_.checkBox_move_aim_on->isChecked();
  // balance
  walking_param.balance_enable = ui_.checkBox_balance_on->isChecked();
  walking_param.balance_hip_roll_gain = ui_.dSpinBox_hip_roll_gain->value();
  walking_param.balance_knee_gain = ui_.dSpinBox_knee_gain->value();
  walking_param.balance_ankle_roll_gain = ui_.dSpinBox_ankle_roll_gain->value();
  walking_param.balance_ankle_pitch_gain = ui_.dSpinBox_ankle_pitch_gain->value();
  walking_param.y_swap_amplitude = ui_.dSpinBox_y_swap_amplitude->value();
  walking_param.z_swap_amplitude = ui_.dSpinBox_z_swap_amplitude->value();
  walking_param.pelvis_offset = ui_.dSpinBox_pelvis_offset->value() * DEGREE2RADIAN;
  walking_param.arm_swing_gain = ui_.dSpinBox_arm_swing_gain->value();

  walking_param.D_ANKLE_P = ui_.dspin_D_ANKLE_P->value();
  walking_param.D_KNEE = ui_.dspin_D_KNEE->value();
  walking_param.D_HIP_P = ui_.dspin_D_HIP_P->value();

  walking_param.D_ANKLE_R = ui_.dspin_D_ANKLE_R->value();
  walking_param.D_HIP_R = ui_.dspin_D_HIP_R->value();

  walking_param.KP_P = ui_.dspin_KP_P->value();
  walking_param.KD_P = ui_.dspin_KD_P->value();
  walking_param.KI_P = ui_.dspin_KI_P->value();

  walking_param.KP_R = ui_.dspin_KP_R->value();
  walking_param.KD_R = ui_.dspin_KD_R->value();
  walking_param.KI_R = ui_.dspin_KI_R->value();

  walking_param.r_x_offset = ui_.dspin_r_x_offset->value();
  walking_param.r_y_offset = ui_.dspin_r_y_offset->value();
  walking_param.r_z_offset = ui_.dspin_r_z_offset->value();
  walking_param.r_yaw_offset = ui_.dspin_r_yaw_offset->value();
  walking_param.r_pitch_offset = ui_.dspin_r_pitch_offset->value();
  walking_param.r_roll_offset = ui_.dspin_r_roll_offset->value();

  walking_param.l_x_offset = ui_.dspin_l_x_offset->value();
  walking_param.l_y_offset = ui_.dspin_l_y_offset->value();
  walking_param.l_z_offset = ui_.dspin_l_z_offset->value();
  walking_param.l_yaw_offset = ui_.dspin_l_yaw_offset->value();
  walking_param.l_pitch_offset = ui_.dspin_l_pitch_offset->value();
  walking_param.l_roll_offset = ui_.dspin_l_roll_offset->value();

  walking_param.shoulder_gain = ui_.dspin_shoulder_gain->value();

  qnode_op3_.applyWalkingParam(walking_param);
}

void MainWindow::updateZMPWalkingParams(zmp_walking_module_msgs::ZMPWalkingParam params)
{
  ui_.spinbox_vx->setValue(params.zmp_vx);
  ui_.spinbox_vy->setValue(params.zmp_vy);
  ui_.spinbox_vphi->setValue(params.zmp_vphi);

  ui_.spinbox_vx_2->setValue(params.m_zmp_vx);
  ui_.spinbox_vy_2->setValue(params.m_zmp_vy);
  ui_.spinbox_vphi_2->setValue(params.m_zmp_vphi);

  ui_.dspin_supp_front->setValue(params.supp_front);
  ui_.dspin_supp_front2->setValue(params.supp_front2);
  ui_.dspin_supp_turn->setValue(params.supp_turn);
  ui_.dspin_supp_sidex->setValue(params.supp_side_x);
  ui_.dspin_supp_sidey->setValue(params.supp_side_y);
  ui_.dspin_supp_x->setValue(params.supp_x);
  ui_.dspin_supp_y->setValue(params.supp_y);

  ui_.dspin_supp_modinity->setValue(params.suppmod_y_init);
  ui_.dspin_turn_comp_thrs->setValue(params.turncomp_thrs);
  ui_.dspin_turn_comp->setValue(params.turn_comp);
  ui_.dspin_accel_comp->setValue(params.accel_comp);
  ui_.dspin_front_comp->setValue(params.front_comp);
  ui_.dspin_hiproll_comp->setValue(params.hiproll_comp);

  ui_.dspin_tstep->setValue(params.tstep);
  ui_.dspin_tzmp->setValue(params.tzmp);
  ui_.dspin_velfast_forward->setValue(params.velfast_forward);
  ui_.dspin_velfast_turn->setValue(params.velfast_turn);
  ui_.dspin_ankle_supp_fact->setValue(params.angkle_supp_factor);
  ui_.dspin_footy->setValue(params.foot_y);
  ui_.dspin_footx->setValue(params.foot_x);
  ui_.dspin_stan_offs->setValue(params.stand_offset);
  ui_.dspin_boyd_height->setValue(params.body_height);
  ui_.dspin_bodytilt->setValue(params.bodytilt);
  ui_.dspin_step_height->setValue(params.step_height);
  ui_.dspin_belly_roll->setValue(params.belly_roll);
  ui_.dspin_belly_hip->setValue(params.belly_hip);

  ui_.dspin_armpitch->setValue(params.arm_pitch);
  ui_.dspin_armroll->setValue(params.arm_roll);
  ui_.dspin_armelbow->setValue(params.arm_elbow);

  ui_.dspin_Kp->setValue(params.Kp);
  ui_.dspin_Kd->setValue(params.Kd);
  ui_.dspin_TH1->setValue(params.TH1);
  ui_.dspin_TH2->setValue(params.TH2);
  ui_.dspin_Acc->setValue(params.Acc);

  ui_.dspin_KpP->setValue(params.KpP);
  ui_.dspin_KpR->setValue(params.KpR);
  ui_.dspin_KdP->setValue(params.KdP);
  ui_.dspin_KdR->setValue(params.KdR);
  ui_.dspin_Ki->setValue(params.Ki);
  ui_.zmp_feedback->setChecked(params.feedback_);
}

void MainWindow::applyZMPWalkingParams()
{
  zmp_walking_module_msgs::ZMPWalkingParam walking_param;

  walking_param.zmp_vx = ui_.spinbox_vx->value();
  walking_param.zmp_vy = ui_.spinbox_vy->value();
  walking_param.zmp_vphi = ui_.spinbox_vphi->value();

  walking_param.m_zmp_vx = ui_.spinbox_vx_2->value();
  walking_param.m_zmp_vy = ui_.spinbox_vy_2->value();
  walking_param.m_zmp_vphi = ui_.spinbox_vphi_2->value();

  walking_param.supp_front = ui_.dspin_supp_front->value();
  walking_param.supp_front2 = ui_.dspin_supp_front2->value();
  walking_param.supp_turn = ui_.dspin_supp_turn->value();
  walking_param.supp_side_x = ui_.dspin_supp_sidex->value();
  walking_param.supp_side_y = ui_.dspin_supp_sidey->value();
  walking_param.supp_x = ui_.dspin_supp_x->value();
  walking_param.supp_y = ui_.dspin_supp_y->value();

  walking_param.suppmod_y_init = ui_.dspin_supp_modinity->value();
  walking_param.turncomp_thrs = ui_.dspin_turn_comp_thrs->value();
  walking_param.turn_comp = ui_.dspin_turn_comp->value();
  walking_param.accel_comp = ui_.dspin_accel_comp->value();
  walking_param.front_comp = ui_.dspin_front_comp->value();
  walking_param.hiproll_comp = ui_.dspin_hiproll_comp->value();

  walking_param.tstep = ui_.dspin_tstep->value();
  walking_param.tzmp = sqrt(ui_.dspin_boyd_height->value()/9.81);
  walking_param.velfast_forward = ui_.dspin_velfast_forward->value();
  walking_param.velfast_turn = ui_.dspin_velfast_turn->value();
  walking_param.angkle_supp_factor = ui_.dspin_ankle_supp_fact->value();
  walking_param.foot_y = ui_.dspin_footy->value();
  walking_param.foot_x = ui_.dspin_footx->value();
  walking_param.stand_offset = ui_.dspin_stan_offs->value();
  walking_param.body_height = ui_.dspin_boyd_height->value();
  walking_param.bodytilt  = ui_.dspin_bodytilt->value();
  walking_param.step_height = ui_.dspin_step_height->value();
  walking_param.belly_roll = ui_.dspin_belly_roll->value();
  walking_param.belly_hip = ui_.dspin_belly_hip->value();

  walking_param.arm_pitch = ui_.dspin_armpitch->value();
  walking_param.arm_roll = ui_.dspin_armroll->value();
  walking_param.arm_elbow = ui_.dspin_armelbow->value();

  walking_param.Kp = ui_.dspin_Kp->value();
  walking_param.Kd = ui_.dspin_Kd->value();
  walking_param.TH1 = ui_.dspin_TH1->value();
  walking_param.TH2 = ui_.dspin_TH2->value();
  walking_param.Acc = ui_.dspin_Acc->value();

  walking_param.KpP = ui_.dspin_KpP->value();
  walking_param.KpR = ui_.dspin_KpR->value();
  walking_param.KdP = ui_.dspin_KdP->value();
  walking_param.KdR = ui_.dspin_KdR->value();
  walking_param.Ki  = ui_.dspin_Ki->value();

  walking_param.feedback_ = ui_.zmp_feedback->isChecked();

//  walking_param.L_Shift = ui_.dspin_L_Shift->value();
//  walking_param.L_Lift = ui_.dspin_L_Lift->value();
//  walking_param.L_P_Kick = ui_.dspin_L_P_Kick->value();
//  walking_param.L_P_Cool = ui_.dspin_L_P_Cool->value();

//  walking_param.R_Shift = ui_.dspin_R_Shift->value();
//  walking_param.R_Lift = ui_.dspin_R_Lift->value();
//  walking_param.R_P_Kick = ui_.dspin_R_P_Kick->value();
//  walking_param.R_P_Cool = ui_.dspin_R_P_Cool->value();

  qnode_op3_.applyZMPWalkingParam(walking_param);
}

//Kicking
void MainWindow::updateKickingParams(kicking_module_msgs::KickingParam params)
{

  ui_.R_Shift_X->setValue(params.R_Shift_X);
  ui_.R_Shift_Y->setValue(params.R_Shift_Y);
  ui_.R_Shift_Z->setValue(params.R_Shift_Z);

  ui_.R_Lift_X->setValue(params.R_Lift_X);
  ui_.R_Lift_Y->setValue(params.R_Lift_Y);
  ui_.R_Lift_Z->setValue(params.R_Lift_Z);

  ui_.R_Swing_X->setValue(params.R_Swing_X);
  ui_.R_Swing_Y->setValue(params.R_Swing_Y);
  ui_.R_Swing_Z->setValue(params.R_Swing_Z);

  ui_.R_Retract_X->setValue(params.R_Retract_X);
  ui_.R_Retract_Y->setValue(params.R_Retract_Y);
  ui_.R_Retract_Z->setValue(params.R_Retract_Z);

  ui_.L_Shift_X->setValue(params.L_Shift_X);
  ui_.L_Shift_Y->setValue(params.L_Shift_Y);
  ui_.L_Shift_Z->setValue(params.L_Shift_Z);

  ui_.L_Lift_X->setValue(params.L_Lift_X);
  ui_.L_Lift_Y->setValue(params.L_Lift_Y);
  ui_.L_Lift_Z->setValue(params.L_Lift_Z);

  ui_.L_Swing_X->setValue(params.L_Swing_X);
  ui_.L_Swing_Y->setValue(params.L_Swing_Y);
  ui_.L_Swing_Z->setValue(params.L_Swing_Z);

  ui_.L_Retract_X->setValue(params.L_Retract_X);
  ui_.L_Retract_Y->setValue(params.L_Retract_Y);
  ui_.L_Retract_Z->setValue(params.L_Retract_Z);

  ui_.Time_Shift->setValue(params.SHIFT_TIME);
  ui_.Time_Lift->setValue(params.LIFT_TIME);
  ui_.Time_Swing->setValue(params.SWING_TIME);
  ui_.Time_Retract->setValue(params.RETRACT_TIME);
  ui_.Time_Landing->setValue(params.LANDING_TIME);
  ui_.Time_Finished->setValue(params.FINISHED_TIME);

  ui_.Torso_X->setValue(params.Torso_X);
  ui_.Torso_Y->setValue(params.Torso_Y);
  ui_.Torso_Z->setValue(params.Torso_Z);  

  ui_.Torso_Pitch->setValue(params.Torso_Pitch);
  ui_.Lift_Pitch->setValue(params.Lift_Pitch);
  ui_.Swing_Pitch->setValue(params.Swing_Pitch);
  ui_.Retract_Pitch->setValue(params.Retract_Pitch);

  ui_.Shift_Roll->setValue(params.Shift_Roll);
  ui_.Lift_Roll->setValue(params.Lift_Roll);
  ui_.Swing_Roll->setValue(params.Swing_Roll);
  ui_.Retract_Roll->setValue(params.Retract_Roll);



}

void MainWindow::applyKickingParams()
{
  kicking_module_msgs::KickingParam kicking_param;

  kicking_param.SHIFT_TIME = ui_.Time_Shift->value();
  kicking_param.LIFT_TIME = ui_.Time_Lift->value();
  kicking_param.SWING_TIME = ui_.Time_Swing->value();
  kicking_param.RETRACT_TIME = ui_.Time_Retract->value();
  kicking_param.LANDING_TIME = ui_.Time_Landing->value();
  kicking_param.FINISHED_TIME = ui_.Time_Finished->value();
  
  kicking_param.Torso_X = ui_.Torso_X->value();
  kicking_param.Torso_Y = ui_.Torso_Y->value();
  kicking_param.Torso_Z = ui_.Torso_Z->value();

  kicking_param.Torso_Pitch = ui_.Torso_Pitch->value();
  kicking_param.Lift_Pitch = ui_.Lift_Pitch->value();
  kicking_param.Swing_Pitch = ui_.Swing_Pitch->value();
  kicking_param.Retract_Pitch = ui_.Retract_Pitch->value();

  kicking_param.Shift_Roll = ui_.Shift_Roll->value();
  kicking_param.Lift_Roll = ui_.Lift_Roll->value();
  kicking_param.Swing_Roll = ui_.Swing_Roll->value();
  kicking_param.Retract_Roll = ui_.Retract_Roll->value();

  kicking_param.R_Shift_X = ui_.R_Shift_X->value();
  kicking_param.R_Shift_Y = ui_.R_Shift_Y->value();
  kicking_param.R_Shift_Z = ui_.R_Shift_Z->value();

  kicking_param.R_Lift_X = ui_.R_Lift_X->value();
  kicking_param.R_Lift_Y = ui_.R_Lift_Y->value();
  kicking_param.R_Lift_Z = ui_.R_Lift_Z->value();

  kicking_param.R_Swing_X = ui_.R_Swing_X->value();
  kicking_param.R_Swing_Y = ui_.R_Swing_Y->value();
  kicking_param.R_Swing_Z = ui_.R_Swing_Z->value();

  kicking_param.R_Retract_X = ui_.R_Retract_X->value();
  kicking_param.R_Retract_Y = ui_.R_Retract_Y->value();
  kicking_param.R_Retract_Z = ui_.R_Retract_Z->value();

  kicking_param.L_Shift_Y = ui_.L_Shift_Y->value();
  kicking_param.L_Shift_Z = ui_.L_Shift_Z->value();
  kicking_param.L_Shift_X = ui_.L_Shift_X->value();

  kicking_param.L_Lift_X = ui_.L_Lift_X->value();
  kicking_param.L_Lift_Y = ui_.L_Lift_Y->value();
  kicking_param.L_Lift_Z = ui_.L_Lift_Z->value();

  kicking_param.L_Swing_X = ui_.L_Swing_X->value();
  kicking_param.L_Swing_Y = ui_.L_Swing_Y->value();
  kicking_param.L_Swing_Z = ui_.L_Swing_Z->value();

  kicking_param.L_Retract_X = ui_.L_Retract_X->value();
  kicking_param.L_Retract_Y = ui_.L_Retract_Y->value();
  kicking_param.L_Retract_Z = ui_.L_Retract_Z->value();


  qnode_op3_.applyKickingParam(kicking_param);
}

//Feedback
void MainWindow::updateFeedbackParams(feedback_msgs::FeedbackParam params)
{
  ui_.FB_Body_Height->setValue(params.Body_Height);
  ui_.FB_Body_Tilt->setValue(params.Body_Tilt);
  ui_.FB_Leg_X->setValue(params.Leg_X);
  ui_.FB_Leg_Y->setValue(params.Leg_Y);
  ui_.FB_Leg_Z->setValue(params.Leg_Z);

  ui_.FB_Leg_R_Z->setValue(params.Leg_R_Z);
  ui_.FB_Leg_L_Z->setValue(params.Leg_L_Z);

  ui_.FB_R_Roll->setValue(params.R_Roll);
  ui_.FB_L_Roll->setValue(params.L_Roll);

  ui_.FB_Setpoint_Pitch->setValue(params.Setpoint_Pitch);
  ui_.FB_Setpoint_Roll->setValue(params.Setpoint_Roll);

  ui_.FB_Gain_Angle_Pitch->setValue(params.Gain_Angle_Pitch);
  ui_.FB_Gain_Angle_Roll->setValue(params.Gain_Angle_Roll);

  ui_.FB_Gain_Velocity_Pitch->setValue(params.Gain_Velocity_Pitch);
  ui_.FB_Gain_Velocity_Roll->setValue(params.Gain_Velocity_Roll);

  ui_.FB_Gain_Integral_Pitch->setValue(params.Gain_Integral_Pitch);
  ui_.FB_Gain_Integral_Roll->setValue(params.Gain_Integral_Roll); 
}

void MainWindow::applyFeedbackParams()
{
  feedback_msgs::FeedbackParam feedback_param;

  feedback_param.Body_Height = ui_.FB_Body_Height->value();
  feedback_param.Body_Tilt = ui_.FB_Body_Tilt->value();
  feedback_param.Leg_X = ui_.FB_Leg_X->value();
  feedback_param.Leg_Y = ui_.FB_Leg_Y->value();
  feedback_param.Leg_Z = ui_.FB_Leg_Z->value();

  feedback_param.Leg_R_Z = ui_.FB_Leg_R_Z->value();
  feedback_param.Leg_L_Z = ui_.FB_Leg_L_Z->value();

  feedback_param.R_Roll = ui_.FB_R_Roll->value();
  feedback_param.L_Roll = ui_.FB_L_Roll->value();

  feedback_param.Setpoint_Pitch = ui_.FB_Setpoint_Pitch->value();
  feedback_param.Setpoint_Roll = ui_.FB_Setpoint_Roll->value();

  feedback_param.Gain_Angle_Pitch = ui_.FB_Gain_Angle_Pitch->value();
  feedback_param.Gain_Angle_Roll = ui_.FB_Gain_Angle_Roll->value();

  feedback_param.Gain_Velocity_Pitch = ui_.FB_Gain_Velocity_Pitch->value();
  feedback_param.Gain_Velocity_Roll = ui_.FB_Gain_Velocity_Roll->value();

  feedback_param.Gain_Integral_Pitch = ui_.FB_Gain_Integral_Pitch->value();
  feedback_param.Gain_Integral_Roll = ui_.FB_Gain_Integral_Roll->value();

  qnode_op3_.applyFeedbackParam(feedback_param);
}

// Capture Step

void MainWindow::updateCaptureStepParams(capture_step_module_msgs::WalkingParam params)
{
    ui_.d_spinBox_c1->setValue(params.halt_pos_leg_ext);
    ui_.d_spinBox_c2->setValue(params.halt_pos_leg_roll_ext);
    ui_.d_spinBox_c3->setValue(params.halt_pos_leg_pitch_ext);
    ui_.d_spinBox_c4->setValue(params.halt_pos_foot_roll_ext);
    ui_.d_spinBox_c5->setValue(params.halt_pos_foot_pitch_ext);
    ui_.d_spinBox_c6->setValue(params.const_ground_push);
    ui_.d_spinBox_c7->setValue(params.propt_ground_push);
    ui_.d_spinBox_c8->setValue(params.const_step_height);
    ui_.d_spinBox_c9->setValue(params.propt_step_height);

    ui_.d_spinBox_t0->setValue(params.swing_start);
    ui_.d_spinBox_t1->setValue(params.swing_stop);
    ui_.d_spinBox_c10->setValue(params.sagittal_swg_fwd);
    ui_.d_spinBox_c11->setValue(params.sagittal_swg_bwd);
    ui_.d_spinBox_c12->setValue(params.lateral_swg);
    ui_.d_spinBox_c13->setValue(params.lateral_swg_offset);
    ui_.d_spinBox_c14->setValue(params.trng_lateral_swg_offset);
    ui_.d_spinBox_c15->setValue(params.rotational_swg);
    ui_.d_spinBox_c16->setValue(params.rotational_swg_offset);

    ui_.d_spinBox_c17->setValue(params.lateral_hip_swg);

    ui_.d_spinBox_c18->setValue(params.fwd_lean);
    ui_.d_spinBox_c19->setValue(params.bwd_lean);
    ui_.d_spinBox_c20->setValue(params.fwd_trng_lean);
    ui_.d_spinBox_c21->setValue(params.gait_vel_limit);
    ui_.d_spinBox_c22->setValue(params.sagittal_acc);
    ui_.d_spinBox_c23->setValue(params.lateral_acc);
    ui_.d_spinBox_c24->setValue(params.rotational_acc);
    ui_.d_spinBox_c25->setValue(params.const_step_freq);
    ui_.d_spinBox_c26->setValue(params.sagittal_prop_step_freq);
    ui_.d_spinBox_c27->setValue(params.lateral_prop_step_freq);

    ui_.d_spinBox_x_move->setValue(params.x_move_amplitude);
    ui_.d_spinBox_y_move->setValue(params.y_move_amplitude);
    ui_.d_spinBox_angle_move->setValue(params.angle_move_amplitude);
}

void MainWindow::applyCaptureStepParams()
{
    capture_step_module_msgs::WalkingParam walking_param;

    walking_param.halt_pos_leg_ext = ui_.d_spinBox_c1->value();
    walking_param.halt_pos_leg_roll_ext = ui_.d_spinBox_c2->value();
    walking_param.halt_pos_leg_pitch_ext = ui_.d_spinBox_c3->value();
    walking_param.halt_pos_foot_roll_ext = ui_.d_spinBox_c4->value();
    walking_param.halt_pos_foot_pitch_ext = ui_.d_spinBox_c5->value();
    walking_param.const_ground_push = ui_.d_spinBox_c6->value();
    walking_param.propt_ground_push = ui_.d_spinBox_c7->value();
    walking_param.const_step_height = ui_.d_spinBox_c8->value();
    walking_param.propt_step_height = ui_.d_spinBox_c9->value();

    walking_param.swing_start = ui_.d_spinBox_t0->value();
    walking_param.swing_stop = ui_.d_spinBox_t1->value();
    walking_param.sagittal_swg_fwd = ui_.d_spinBox_c10->value();
    walking_param.sagittal_swg_bwd = ui_.d_spinBox_c11->value();
    walking_param.lateral_swg = ui_.d_spinBox_c12->value();
    walking_param.lateral_swg_offset = ui_.d_spinBox_c13->value();
    walking_param.trng_lateral_swg_offset = ui_.d_spinBox_c14->value();
    walking_param.rotational_swg = ui_.d_spinBox_c15->value();
    walking_param.rotational_swg_offset = ui_.d_spinBox_c16->value();

    walking_param.lateral_hip_swg = ui_.d_spinBox_c17->value();

    walking_param.fwd_lean = ui_.d_spinBox_c18->value();
    walking_param.bwd_lean = ui_.d_spinBox_c19->value();
    walking_param.fwd_trng_lean = ui_.d_spinBox_c20->value();
    walking_param.gait_vel_limit = ui_.d_spinBox_c21->value();
    walking_param.sagittal_acc = ui_.d_spinBox_c22->value();
    walking_param.lateral_acc = ui_.d_spinBox_c23->value();
    walking_param.rotational_acc = ui_.d_spinBox_c24->value();
    walking_param.const_step_freq = ui_.d_spinBox_c25->value();
    walking_param.sagittal_prop_step_freq = ui_.d_spinBox_c26->value();
    walking_param.lateral_prop_step_freq = ui_.d_spinBox_c27->value();

    walking_param.x_move_amplitude = ui_.d_spinBox_x_move->value();
    walking_param.y_move_amplitude = ui_.d_spinBox_y_move->value();
    walking_param.angle_move_amplitude = ui_.d_spinBox_angle_move->value();
    walking_param.halt_position = ui_.checkBox_halt_position->isChecked();
    walking_param.leg_lifting = ui_.checkBox_leg_lifting->isChecked();
    walking_param.leg_swing = ui_.checkBox_leg_swing->isChecked();
    walking_param.lateral_swing = ui_.checkBox_lateral_hip_swing->isChecked();
    walking_param.leaning = ui_.checkBox_leaning->isChecked();

    qnode_op3_.applyCaptureStepParam(walking_param);

}

void MainWindow::walkingCommandShortcut()
{
  if (is_walking_ == true)
  {
    is_walking_ = false;
    qnode_op3_.setWalkingCommand("stop");
  }
  else
  {
    is_walking_ = true;
    qnode_op3_.setWalkingCommand("start");
  }
}

/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."), tr("<h2>OP3 Demo 0.10</h2><p>Copyright ROBOTIS</p>"));
}

/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

void MainWindow::initModeUnit()
{
  int number_joint = qnode_op3_.getJointSize();

  // preset button
  QHBoxLayout *preset_layout = new QHBoxLayout;
  QSignalMapper *signalMapper = new QSignalMapper(this);

  // yaml preset
  for (std::map<int, std::string>::iterator module_it = qnode_op3_.module_table_.begin();
       module_it != qnode_op3_.module_table_.end(); ++module_it)
  {
    std::string preset_name = module_it->second;
    QPushButton *preset_button = new QPushButton(tr(preset_name.c_str()));
    if (debug_)
      std::cout << "name : " << preset_name << std::endl;

    preset_layout->addWidget(preset_button);

    signalMapper->setMapping(preset_button, preset_button->text());
    QObject::connect(preset_button, SIGNAL(clicked()), signalMapper, SLOT(map()));
  }

  QObject::connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(setMode(QString)));

  ui_.widget_mode_preset->setLayout(preset_layout);

  // joints
  QGridLayout *grid_layout = new QGridLayout;
  for (int ix = 0; ix < number_joint; ix++)
  {
    std::stringstream label_stream;
    std::string joint_name;
    int joint_id;

    if (qnode_op3_.getIDJointNameFromIndex(ix, joint_id, joint_name) == false)
      continue;

    label_stream << "[" << (joint_id < 10 ? "0" : "") << joint_id << "] " << joint_name;
    QLabel *id_label = new QLabel(tr(label_stream.str().c_str()));

    QStringList module_list;
    for (int index = 0; index < qnode_op3_.getModeSize(); index++)
    {
      std::string module_name = qnode_op3_.getModeName(index);
      if (module_name != "")
        module_list << module_name.c_str();
    }

    QComboBox *module_combo = new QComboBox();
    module_combo->setObjectName(tr(joint_name.c_str()));
    module_combo->addItems(module_list);
    module_combo->setEnabled(false);      // not changable
    int num_row = ix / 2 + 1;
    int num_col = (ix % 2) * 3;
    grid_layout->addWidget(id_label, num_row, num_col, 1, 1);
    grid_layout->addWidget(module_combo, num_row, num_col + 1, 1, 2);
  }

  // get/set buttons
  QPushButton *get_mode_button = new QPushButton(tr("Get Mode"));
  grid_layout->addWidget(get_mode_button, (number_joint / 2) + 2, 0, 1, 3);
  QObject::connect(get_mode_button, SIGNAL(clicked(bool)), &qnode_op3_, SLOT(getJointControlMode()));

  ui_.widget_mode->setLayout(grid_layout);

  // make module widget table
  for (int index = 0; index < qnode_op3_.getModeSize(); index++)
  {
    std::string module_name = qnode_op3_.getModeName(index);
    if (module_name == "")
      continue;
    std::string module_reg = "*_" + module_name;

    QRegExp reg_exp(QRegExp(tr(module_reg.c_str())));
    reg_exp.setPatternSyntax(QRegExp::Wildcard);

    QList<QWidget *> widget_list = ui_.centralwidget->findChildren<QWidget *>(reg_exp);
    module_ui_table_[module_name] = widget_list;

    if (debug_)
      std::cout << "Module widget : " << module_name << " [" << widget_list.size() << "]" << std::endl;
  }

  // make motion tab
  if (qnode_op3_.getModeIndex("action_module") != -1)
    initMotionUnit();
}

void MainWindow::initMotionUnit()
{
  // preset button
  QGridLayout *motion_layout = new QGridLayout;
  QSignalMapper *signal_mapper = new QSignalMapper(this);

  // yaml preset
  int index = 0;
  for (std::map<int, std::string>::iterator motion_it = qnode_op3_.motion_table_.begin();
       motion_it != qnode_op3_.motion_table_.end(); ++motion_it)
  {
    int motion_index = motion_it->first;
    std::string motion_name = motion_it->second;
    QString q_motion_name = QString::fromStdString(motion_name);
    QPushButton *motion_button = new QPushButton(q_motion_name);

    int button_size = (motion_index < 0) ? 2 : 1;
    int num_row = index / 4;
    int num_col = index % 4;
    motion_layout->addWidget(motion_button, num_row, num_col, 1, button_size);

    //hotkey
    std::map<int, int>::iterator shortcut_it = qnode_op3_.motion_shortcut_table_.find(motion_index);
    if (shortcut_it != qnode_op3_.motion_shortcut_table_.end())
      motion_button->setShortcut(QKeySequence(shortcut_it->second));

    signal_mapper->setMapping(motion_button, motion_index);
    QObject::connect(motion_button, SIGNAL(clicked()), signal_mapper, SLOT(map()));

    index += button_size;
  }

  int num_row = index / 4;
  num_row = (index % 4 == 0) ? num_row : num_row + 1;
  QSpacerItem *vertical_spacer = new QSpacerItem(20, 400, QSizePolicy::Minimum, QSizePolicy::Expanding);
  motion_layout->addItem(vertical_spacer, num_row, 0, 1, 4);

  QObject::connect(signal_mapper, SIGNAL(mapped(int)), &qnode_op3_, SLOT(playMotion(int)));

  // ui_.scroll_widget_motion->setLayout(motion_layout);
}

void MainWindow::setMode(QString mode_name)
{
  qnode_op3_.setControlMode(mode_name.toStdString());
}

void MainWindow::readSettings()
{
  QSettings settings("Qt-Ros Package", "op3_gui_demo");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::writeSettings()
{
  QSettings settings("Qt-Ros Package", "op3_gui_demo");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  QMainWindow::closeEvent(event);
}

/*****************************************************************************
 ** Implementation [Util]
 *****************************************************************************/

Eigen::MatrixXd MainWindow::rotateX(double s)
{
  Eigen::MatrixXd rotation_matrix(3, 3);

  rotation_matrix << 1.0, 0.0, 0.0, 0.0, cos(s), -sin(s), 0.0, sin(s), cos(s);

  return rotation_matrix;
}

Eigen::MatrixXd MainWindow::rotateY(double s)
{
  Eigen::MatrixXd rotation_matrix(3, 3);

  rotation_matrix << cos(s), 0.0, sin(s), 0.0, 1.0, 0.0, -sin(s), 0.0, cos(s);

  return rotation_matrix;
}

Eigen::MatrixXd MainWindow::rotateZ(double s)
{
  Eigen::MatrixXd rotation_matrix(3, 3);

  rotation_matrix << cos(s), -sin(s), 0.0, sin(s), cos(s), 0.0, 0.0, 0.0, 1.0;

  return rotation_matrix;
}

Eigen::MatrixXd MainWindow::convertRpy2Rotation(double r, double p, double y)
{
  Eigen::MatrixXd rotation_matrix = rotateZ(y) * rotateY(p) * rotateX(r);

  return rotation_matrix;
}

Eigen::Quaterniond MainWindow::convertRpy2Quaternion(double r, double p, double y)
{
  Eigen::MatrixXd rotation_matrix = convertRpy2Rotation(r, p, y);

  Eigen::Matrix3d rotation_matrix_block;
  rotation_matrix_block = rotation_matrix.block(0, 0, 3, 3);

  Eigen::Quaterniond quaternion;
  quaternion = rotation_matrix_block;

  return quaternion;
}

}  // namespace robotis_op

