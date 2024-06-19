#include <op3_gui_demo/main_window.hpp>

namespace robotis_op {
// Capture StepsetZMPWalkingCommand

void MainWindow::on_button_quintic_walk_apply_clicked(bool check)
{
    applyQuinticWalkParams();
}

void MainWindow::on_button_quintic_walk_refresh_clicked(bool check)
{
    qnode_op3_.refreshQuinticWalkParam();
}

void MainWindow::on_button_quintic_walk_save_clicked(bool check)
{
    qnode_op3_.setQuinticWalkCommand("save");

}

void MainWindow::on_button_quintic_walk_start_clicked(bool check)
{
    qnode_op3_.setQuinticWalkCommand("start");
}

void MainWindow::on_button_quintic_walk_stop_clicked(bool check)
{
    qnode_op3_.setQuinticWalkCommand("stop");
}

void MainWindow::updateQuinticWalkParams(quintic_walk_msgs::WalkingParam params)
{
    ui_.dspin_qw_freq->setValue(params.freq);
    ui_.dspin_qw_dspratio->setValue(params.doubleSupportRatio);
    ui_.dspin_qw_foot_Distance->setValue(params.footDistance);
    ui_.dspin_qw_foot_Raise->setValue(params.footRise);
    ui_.dspin_qw_foot_putdownOffset->setValue(params.footPutDownZOffset);
    ui_.dspin_qw_foot_putdownPhase->setValue(params.footPutDownPhase);
    ui_.dspin_qw_foot_apexphase->setValue(params.footApexPhase);
    ui_.dspin_qw_foot_overshootRatio->setValue(params.footOvershootRatio);
    ui_.dspin_qw_foot_overshootPahse->setValue(params.footOvershootPhase);
    ui_.dspin_qw_trunk_height->setValue(params.trunkHeight);
    ui_.dspin_qw_trunk_pitch->setValue(params.trunkPitch * 180/M_PI);
    ui_.dspin_qw_trunk_phase->setValue(params.trunkPhase);
    ui_.dspin_qw_trunk_pause->setValue(params.trunkPause);
    ui_.dspin_qw_trunk_xoffset->setValue(params.trunkXOffset);
    ui_.dspin_qw_trunk_yoffset->setValue(params.trunkYOffset);
    ui_.dspin_qw_trunk_swing->setValue(params.trunkSwing);
    ui_.dspin_qw_trunk_XOffsetforward->setValue(params.trunkXOffsetPCoefForward);
    ui_.dspin_qw_trunk_XOffsetturn->setValue(params.trunkXOffsetPCoefTurn);
    ui_.dspin_pitch_error_tol->setValue(params.pitch_error_tol);
    ui_.checkBox_tuning->setChecked(params.tuning);

    ui_.dspin_quintic_walk_XMove->setValue(params.XMove);
    ui_.dspin_quintic_walk_YMove->setValue(params.YMove);
    ui_.dspin_quintic_walk_angle->setValue(params.ZMove);

    //feedback
    ui_.dspin_D_ANKLE_P_qw->setValue(params.D_ANKLE_P_qw);
    ui_.dspin_D_KNEE_qw->setValue(params.D_KNEE_qw);
    ui_.dspin_D_HIP_P_qw->setValue(params.D_HIP_P_qw);


    ui_.dspin_KP_P_qw->setValue(params.KP_P_qw);
    ui_.dspin_KD_P_qw->setValue(params.KD_P_qw);
    ui_.dspin_KI_P_qw->setValue(params.KI_P_qw);

    ui_.dspin_setpointPitch->setValue(params.setpointPitch);
    ui_.dspin_pitch_error_tol->setValue(params.pitch_error_tol);

    ui_.dspin_shoulder_gain_qw->setValue(params.ShoulderGain_qw);
    ui_.dspin_Angle_0->setValue(params.Angle_0);
    ui_.dspin_Angle_1->setValue(params.Angle_1);
    ui_.dspin_Angle_2->setValue(params.Angle_2);
    ui_.dspin_Angle_3->setValue(params.Angle_3);
    ui_.dspin_Angle_4->setValue(params.Angle_4);

    ui_.dspin_KD_0->setValue(params.KD_0);
    ui_.dspin_KD_1->setValue(params.KD_1);
    ui_.dspin_KD_2->setValue(params.KD_2);
    ui_.dspin_KD_3->setValue(params.KD_3);
    ui_.dspin_KD_4->setValue(params.KD_4);

    ui_.dspin_Gyro_0->setValue(params.Gyro_0);
    ui_.dspin_Gyro_1->setValue(params.Gyro_1);
    ui_.dspin_Gyro_2->setValue(params.Gyro_2);
    ui_.dspin_Gyro_3->setValue(params.Gyro_3);
    ui_.dspin_Gyro_4->setValue(params.Gyro_4);

    ui_.dspin_KP_0->setValue(params.KP_0);
    ui_.dspin_KP_1->setValue(params.KP_1);
    ui_.dspin_KP_2->setValue(params.KP_2);
    ui_.dspin_KP_3->setValue(params.KP_3);
    ui_.dspin_KP_4->setValue(params.KP_4);

}

void MainWindow::applyQuinticWalkParams()
{
    quintic_walk_msgs::WalkingParam walking_param;

    walking_param.freq = ui_.dspin_qw_freq->value();
    walking_param.doubleSupportRatio = ui_.dspin_qw_dspratio->value();
    walking_param.footDistance = ui_.dspin_qw_foot_Distance->value();
    walking_param.footRise = ui_.dspin_qw_foot_Raise->value();
    walking_param.footPutDownZOffset = ui_.dspin_qw_foot_putdownOffset->value();
    walking_param.footPutDownPhase = ui_.dspin_qw_foot_putdownPhase->value();
    walking_param.footApexPhase = ui_.dspin_qw_foot_apexphase->value();
    walking_param.footOvershootRatio = ui_.dspin_qw_foot_overshootRatio->value();
    walking_param.footOvershootPhase = ui_.dspin_qw_foot_overshootPahse->value();
    walking_param.trunkHeight = ui_.dspin_qw_trunk_height->value();
    walking_param.trunkPitch = ui_.dspin_qw_trunk_pitch->value() * M_PI/180;
    walking_param.trunkPhase = ui_.dspin_qw_trunk_phase->value();
    walking_param.trunkPause = ui_.dspin_qw_trunk_pause->value();
    walking_param.trunkXOffset = ui_.dspin_qw_trunk_xoffset->value();
    walking_param.trunkYOffset = ui_.dspin_qw_trunk_yoffset->value();
    walking_param.trunkSwing = ui_.dspin_qw_trunk_swing->value();
    walking_param.trunkXOffsetPCoefForward = ui_.dspin_qw_trunk_XOffsetforward->value();
    walking_param.trunkXOffsetPCoefTurn = ui_.dspin_qw_trunk_XOffsetturn->value();
    walking_param.pitch_error_tol = ui_.dspin_pitch_error_tol->value();
    walking_param.setpointPitch = ui_.dspin_setpointPitch->value();
    walking_param.tuning = ui_.checkBox_tuning->isChecked();

    walking_param.XMove = ui_.dspin_quintic_walk_XMove->value();
    walking_param.YMove = ui_.dspin_quintic_walk_YMove->value();
    walking_param.ZMove = ui_.dspin_quintic_walk_angle->value();

    walking_param.D_ANKLE_P_qw = ui_.dspin_D_ANKLE_P_qw->value();
    walking_param.D_KNEE_qw = ui_.dspin_D_KNEE_qw->value();
    walking_param.D_HIP_P_qw = ui_.dspin_D_HIP_P_qw->value();




    walking_param.KP_P_qw = ui_.dspin_KP_P_qw->value();
    walking_param.KD_P_qw = ui_.dspin_KD_P_qw->value();
    walking_param.KI_P_qw = ui_.dspin_KI_P_qw->value();

    walking_param.ShoulderGain_qw = ui_.dspin_shoulder_gain_qw->value();
    walking_param.Angle_0 = ui_.dspin_Angle_0->value();
    walking_param.Angle_1 = ui_.dspin_Angle_1->value();
    walking_param.Angle_2 = ui_.dspin_Angle_2->value();
    walking_param.Angle_3 = ui_.dspin_Angle_3->value();
    walking_param.Angle_4 = ui_.dspin_Angle_4->value();

    walking_param.KD_0 = ui_.dspin_KD_0->value();
    walking_param.KD_1 = ui_.dspin_KD_1->value();
    walking_param.KD_2 = ui_.dspin_KD_2->value();
    walking_param.KD_3 = ui_.dspin_KD_3->value();
    walking_param.KD_4 = ui_.dspin_KD_4->value();

    walking_param.Gyro_0 = ui_.dspin_Gyro_0->value();
    walking_param.Gyro_1 = ui_.dspin_Gyro_1->value();
    walking_param.Gyro_2 = ui_.dspin_Gyro_2->value();
    walking_param.Gyro_3 = ui_.dspin_Gyro_3->value();
    walking_param.Gyro_4 = ui_.dspin_Gyro_4->value();

    walking_param.KP_0 = ui_.dspin_KP_0->value();
    walking_param.KP_1 = ui_.dspin_KP_1->value();
    walking_param.KP_2 = ui_.dspin_KP_2->value();
    walking_param.KP_3 = ui_.dspin_KP_3->value();
    walking_param.KP_4 = ui_.dspin_KP_4->value();


    qnode_op3_.applyQuinticWalkParam(walking_param);
}

// // Kicking
// void MainWindow::on_kick_apply_button_clicked(bool check)
// {
//     applyKickingParam();
// }

// void MainWindow::on_kick_refresh_button_clicked(bool check)
// {
//     qnode_op3_.refreshQuinticWalkParam();
// }

// void MainWindow::on_kick_save_button_clicked(bool check)
// {
//     qnode_op3_.setQuinticWalkCommand("save");

// }

// void MainWindow::on_button_quintic_walk_start_clicked(bool check)
// {
//     qnode_op3_.setQuinticWalkCommand("start");
// }

// void MainWindow::on_button_quintic_walk_stop_clicked(bool check)
// {
//     qnode_op3_.setQuinticWalkCommand("stop");
// }


}
