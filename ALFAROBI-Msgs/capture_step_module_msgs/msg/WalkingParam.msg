####### Halt Position #######
float32 halt_pos_leg_ext
float32 halt_pos_leg_roll_ext
float32 halt_pos_leg_pitch_ext
float32 halt_pos_foot_roll_ext
float32 halt_pos_foot_pitch_ext

####### Leg Lifting #####
float32 const_ground_push
float32 propt_ground_push
float32 const_step_height
float32 propt_step_height

########## Leg Swing ########
float32 swing_start
float32 swing_stop
float32 sagittal_swg_fwd
float32 sagittal_swg_bwd
float32 lateral_swg
float32 lateral_swg_offset
float32 trng_lateral_swg_offset
float32 rotational_swg
float32 rotational_swg_offset

########## Lateral Hip Swing ##########
float32 lateral_hip_swg

########## Leaning ##########
float32 fwd_lean
float32 bwd_lean
float32 fwd_trng_lean

########## Step ##########
float32 gait_vel_limit
float32 sagittal_acc
float32 lateral_acc
float32 rotational_acc
float32 const_step_freq
float32 sagittal_prop_step_freq
float32 lateral_prop_step_freq

########## walking parameter ########
float32 x_move_amplitude
float32 y_move_amplitude
float32 angle_move_amplitude
bool halt_position
bool leg_lifting
bool leg_swing
bool lateral_swing
bool leaning

