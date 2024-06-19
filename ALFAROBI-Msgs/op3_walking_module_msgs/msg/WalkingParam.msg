####### walking init pose #######
float32 init_x_offset
float32 init_y_offset
float32 init_z_offset
float32 init_roll_offset
float32 init_pitch_offset
float32 init_yaw_offset
float32 r_x_offset
float32 r_y_offset
float32 r_z_offset
float32 r_roll_offset
float32 r_pitch_offset
float32 r_yaw_offset
float32 l_x_offset
float32 l_y_offset
float32 l_z_offset
float32 l_roll_offset
float32 l_pitch_offset
float32 l_yaw_offset


####### time parameter #####
float32 period_time
float32 dsp_ratio
float32 step_fb_ratio

########## walking parameter ########
float32 x_move_amplitude
float32 y_move_amplitude
float32 z_move_amplitude
float32 angle_move_amplitude
bool move_aim_on

########## balance parameter ##########
bool balance_enable
float32 balance_hip_roll_gain
float32 balance_knee_gain
float32 balance_ankle_roll_gain
float32 balance_ankle_pitch_gain
float32 y_swap_amplitude
float32 z_swap_amplitude
float32 arm_swing_gain
float32 pelvis_offset
float32 hip_pitch_offset

########## balance parameter ##########
float32 KP_P
float32 KD_P
float32 KI_P
float32 D_ANKLE_P
float32 D_KNEE
float32 D_HIP_P

float32 KP_R
float32 KD_R
float32 KI_R
float32 D_ANKLE_R
float32 D_HIP_R
float32 shoulder_gain

########## gain parameter ##########
int32 p_gain
int32 i_gain
int32 d_gain

########## ZMP WALKING BIT-BOTS ###########
float32 zmp_vx
float32 zmp_vy
float32 zmp_vphi

bool zmp_useGyro
