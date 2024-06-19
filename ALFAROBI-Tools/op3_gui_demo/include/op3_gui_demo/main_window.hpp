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

#ifndef OP3_DEMO_MAIN_WINDOW_H
#define OP3_DEMO_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#endif
/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace robotis_op
{

#define DEGREE2RADIAN     (M_PI / 180.0)
#define RADIAN2DEGREE     (180.0 / M_PI)

/*****************************************************************************
 ** Interface [MainWindow]
 *****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
Q_OBJECT

 public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void readSettings();  // Load up qt program settings at startup
  void writeSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage();

 public Q_SLOTS:

  /******************************************
   ** Auto-connections (connectSlotsByName())
   *******************************************/
  void on_actionAbout_triggered();
  void on_button_init_pose_clicked(bool check);

  // Walking
  void on_button_init_gyro_clicked(bool check);
  void on_button_walking_start_clicked(bool check);
  void on_button_walking_stop_clicked(bool check);

  void on_zmp_start_button_clicked(bool check);
  void on_zmp_stop_button_clicked(bool check);
  void on_zmp_reset_button_clicked(bool check);

  void on_button_param_refresh_clicked(bool check);
  void on_button_param_apply_clicked(bool check);
  void on_button_param_save_clicked(bool check);

  void on_zmp_apply_button_clicked(bool check);
  void on_zmp_save_button_clicked(bool check);
  void on_zmp_refresh_button_clicked(bool check);

  //Capture Step

  void on_button_capture_step_apply_clicked(bool check);
  void on_button_capture_step_refresh_clicked(bool check);
  void on_button_capture_step_save_clicked(bool check);
  void on_button_capture_step_start_clicked(bool check);
  void on_button_capture_step_stop_clicked(bool check);

  // Quintic
  void on_button_quintic_walk_apply_clicked(bool check);
  void on_button_quintic_walk_refresh_clicked(bool check);
  void on_button_quintic_walk_save_clicked(bool check);
  void on_button_quintic_walk_start_clicked(bool check);
  void on_button_quintic_walk_stop_clicked(bool check);

  // Kicking
  void on_left_kick_clicked(bool check);
  void on_right_kick_clicked(bool check);
  void on_stop_kick_clicked(bool check);
  void on_kick_apply_button_clicked(bool check);
  void on_kick_save_button_clicked(bool check);
  void on_kick_refresh_button_clicked(bool check);
  void updateKickingParams(kicking_module_msgs::KickingParam params);

  // Feedback
  void on_Left_SSP_clicked(bool check);
  void on_Right_SSP_clicked(bool check);
  void on_DSP_clicked(bool check);
  void on_fb_apply_button_clicked(bool check);
  void on_fb_save_button_clicked(bool check);
  void on_fb_refresh_button_clicked(bool check);
  void updateFeedbackParams(feedback_msgs::FeedbackParam params);

  void on_checkBox_balance_on_clicked(bool check);
  void on_checkBox_balance_off_clicked(bool check);

  // Head Control
  void on_head_center_button_clicked(bool check);

  /******************************************
   ** Manual connections
   *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically
  void setMode(bool check);
  void updateCurrentJointMode(std::vector<int> mode);
  void setMode(QString mode_name);

  // Head Control
  void updateHeadAngles(double pan, double tilt);

  // Walking
  void updateWalkingParams(op3_walking_module_msgs::WalkingParam params);
  void updateZMPWalkingParams(zmp_walking_module_msgs::ZMPWalkingParam params);
  void updateCaptureStepParams(capture_step_module_msgs::WalkingParam params);
  void updateQuinticWalkParams(quintic_walk_msgs::WalkingParam params);
  void walkingCommandShortcut();

 protected Q_SLOTS:
  void setHeadAngle();

 private:
  enum Motion_Index
  {
    InitPose = 1,
    WalkingReady = 9,
    GetUpFront = 122,
    GetUpBack = 123,
    RightKick = 121,
    LeftKick = 120,
    Ceremony = 85,
  };

  void setUserShortcut();
  void initModeUnit();
  void initMotionUnit();

  void updateModuleUI();
  void setHeadAngle(double pan, double tilt);
  void applyWalkingParams();
  void applyZMPWalkingParams();
  void applyKickingParams();
  void applyCaptureStepParams();
  void applyQuinticWalkParams();
  void applyFeedbackParams();

  /******************************************
   ** Transformation
   *******************************************/
  Eigen::MatrixXd rotateX(double s);
  Eigen::MatrixXd rotateY(double s);
  Eigen::MatrixXd rotateZ(double s);
  Eigen::MatrixXd convertRpy2Rotation(double r, double p, double y);
  Eigen::Quaterniond convertRpy2Quaternion(double r, double p, double y);

  Ui::MainWindowDesign ui_;
  QNodeOP3 qnode_op3_;
  bool debug_;

  bool is_updating_;
  bool is_walking_;
  std::map<std::string, QList<QWidget *> > module_ui_table_;
};

}  // namespace robotis_op

#endif // OP3_DEMO_MAIN_WINDOW_H
