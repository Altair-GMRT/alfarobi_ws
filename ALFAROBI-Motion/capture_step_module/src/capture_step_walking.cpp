#include <capture_step_module/capture_step.h>

namespace robotis_op {

CaptureStepModule::CaptureStepModule()
    : control_cycle_msec_(8),
      debug_print_(false)

{
    enable_ = false;
    module_name_ = "capture_step_module";
    control_mode_ = robotis_framework::PositionControl;

    init_pose_count_ = 0;
    walking_state_ = WalkingInitPose;

    op3_kd_ = new robotis_op::OP3KinematicsDynamics(robotis_op::WholeBody);

    // result
    result_["r_hip_yaw"] = new robotis_framework::DynamixelState();
    result_["r_hip_roll"] = new robotis_framework::DynamixelState();
    result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
    result_["r_knee"] = new robotis_framework::DynamixelState();
    result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
    result_["r_ank_roll"] = new robotis_framework::DynamixelState();

    result_["l_hip_yaw"] = new robotis_framework::DynamixelState();
    result_["l_hip_roll"] = new robotis_framework::DynamixelState();
    result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
    result_["l_knee"] = new robotis_framework::DynamixelState();
    result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
    result_["l_ank_roll"] = new robotis_framework::DynamixelState();

    result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
    result_["l_sho_pitch"] = new robotis_framework::DynamixelState();

    // joint table
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

    joint_table_["r_sho_pitch"] = 12;
    joint_table_["l_sho_pitch"] = 13;

    target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
    current_position_ = Eigen::MatrixXd::Zero(1, result_.size());
    goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());
    init_position_ = Eigen::MatrixXd::Zero(1, result_.size());
    joint_axis_direction_ = Eigen::MatrixXi::Zero(1, result_.size());
}

CaptureStepModule::~CaptureStepModule()
{
    queue_thread_.join();
}

void CaptureStepModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
    queue_thread_ = boost::thread(boost::bind(&CaptureStepModule::queueThread, this));
    control_cycle_msec_ = control_cycle_msec;
    ros::Time::init();

    walking_param_.halt_pos_leg_ext = 0.03;      // Halt Position Leg Extension
    walking_param_.halt_pos_leg_roll_ext = 0.18;       // Halt Position Leg Roll Angle
    walking_param_.halt_pos_leg_pitch_ext = 0.035; //0.045;      // Halt Position Leg Pitch Angle
    walking_param_.halt_pos_foot_roll_ext = 0.03;      // Halt Position Foot Roll Angle
    walking_param_.halt_pos_foot_pitch_ext = 0.0;       // Halt Position Foot Pitch Angle
    walking_param_.const_ground_push = 0.2; //0.02;      // Constant Ground Push (must not be more than walking_param_.halt_pos_leg_ext)
    walking_param_.propt_ground_push = 0.0;       // Proportional Ground Push
    walking_param_.const_step_height = 0.6;     // Constant Step Height
    walking_param_.propt_step_height = 0.16;  //0.12    // Proportional Step Height

    walking_param_.swing_start = 0.0;    // Swing Start Timing
    walking_param_.swing_stop = 3.14; // Swing Stop Timing
    walking_param_.sagittal_swg_fwd = 0.37;     // Sagittal Swing Amplitude Fwd
    walking_param_.sagittal_swg_bwd = 0.2;     // Sagittal Swing Amplitude Bwd
    walking_param_.lateral_swg = 0.22;      // Lateral Swing Amplitude
    walking_param_.lateral_swg_offset = 0.1;     // Lateral Swing Amplitude Offset
    walking_param_.trng_lateral_swg_offset = 0.015;    // Turning Lateral Swing Amplitude Offset
    walking_param_.rotational_swg = 0.37;      // Rotational Swing Amplitude
    walking_param_.rotational_swg_offset = 0.1;     // Rotational Swing Amplitude Offset

    walking_param_.lateral_hip_swg = 0.08;    // Lateral Hip Swing Amplitude

    walking_param_.fwd_lean = 0.0;      // Forward Lean
    walking_param_.bwd_lean = 0.0;      // Backward Lean
    walking_param_.fwd_trng_lean = -0.07;    // Forward and Turning Lean
    walking_param_.gait_vel_limit = 3.5;      // Gait Velocity Limiting Norm p
    walking_param_.sagittal_acc = 0.01;   // Sagittal Acceleration
    walking_param_.lateral_acc = 0.014;     // Lateral Acceleration
    walking_param_.rotational_acc = 0.009;    // Rotational Acceleration
    walking_param_.const_step_freq = 0.09; // 0.09;     // Constant Step Frequency
    walking_param_.sagittal_prop_step_freq = 0.008;    // Sagittal Proportional Step Frequency
    walking_param_.lateral_prop_step_freq = 0.0;      // Lateral Proportional Step Frequency

    m_Tau = 0;
    m_TauRate = 0;

    walking_param_.x_move_amplitude = 0.0;
    walking_param_.y_move_amplitude = 0.0;
    walking_param_.angle_move_amplitude = 0.0;

    walking_param_.halt_position = false;
    walking_param_.leg_lifting = false;
    walking_param_.leg_swing = false;
    walking_param_.lateral_swing = false;
    walking_param_.leaning = false;

    m_VxTarget = 0;
    m_VyTarget = 0;
    m_VphiTarget = 0;
    m_V << 0,0,0;

    ctrl_running_ = false;
    real_running_ = false;
    time_ = 0;

    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,
    //                     L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL,
    //                     R_ARM_SWING, L_ARM_SWING
    joint_axis_direction_ <<      -1,      1,            1,            1,        -1,            -1,
            -1,      1,           -1,           -1,       1,            -1,
            1,         -1;
    init_position_        <<     0.0,        0.0,         0.0,    0.0,           0.0,          0.0,
            0.0,        0.0,         0.0,    0.0,           0.0,          0.0,
            5.0,       -5.0;
    init_position_ *= DEGREE2RADIAN;

    ros::NodeHandle ros_node;

    std::string default_param_path = ros::package::getPath("capture_step_module") + "/config/param.yaml";
    ros_node.param<std::string>("capture_step_param_path", param_path_, default_param_path);

    loadWalkingParam(default_param_path);
}

void CaptureStepModule::queueThread()
{
    ros::NodeHandle ros_node;
    ros::CallbackQueue callback_queue;

    ros_node.setCallbackQueue(&callback_queue);

    /* publish topics */
    status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);

    /* ROS Service Callback Functions */
    ros::ServiceServer get_walking_param_server = ros_node.advertiseService("/robotis/walking/capture_step_get_params",
                                                                            &CaptureStepModule::getWalkigParameterCallback,
                                                                            this);
    /* sensor topic subscribe */
    ros::Subscriber walking_command_sub = ros_node.subscribe("/robotis/walking/capture_step_command", 10,
                                                             &CaptureStepModule::walkingCommandCallback, this);
    ros::Subscriber walking_param_sub = ros_node.subscribe("/robotis/walking/capture_step_set_params", 10,
                                                           &CaptureStepModule::walkingParameterCallback, this);
    ros::Subscriber gazebo_imu_sub = ros_node.subscribe("robotis/imu", 10, &CaptureStepModule::gazeboImuCallback, this);

    ros::Subscriber current_joint_states_sub_ = ros_node.subscribe("/robotis/present_joint_states", 10, &CaptureStepModule::currentJointStatesCallback,
                                                                   this);
    ros::Subscriber gazebo_mag_sub = ros_node.subscribe("/magnetic", 10, &CaptureStepModule::gazeboMagCallback, this);

    ros::WallDuration duration(control_cycle_msec_ / 1000.0);
    while(ros_node.ok())
        callback_queue.callAvailable(duration);
}

void CaptureStepModule::publishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg status_msg;
    status_msg.header.stamp = ros::Time::now();
    status_msg.type = type;
    status_msg.module_name = "Walking";
    status_msg.status_msg = msg;

    status_msg_pub_.publish(status_msg);
}

void CaptureStepModule::walkingCommandCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "start")
        startWalking();
    else if (msg->data == "stop")
        stop();
    else if (msg->data == "save")
        saveWalkingParam(param_path_);
}

void CaptureStepModule::walkingParameterCallback(const capture_step_module_msgs::WalkingParam::ConstPtr &msg)
{
    walking_param_ = *msg;
}


bool CaptureStepModule::getWalkigParameterCallback(capture_step_module_msgs::GetWalkingParam::Request &req,
                                                   capture_step_module_msgs::GetWalkingParam::Response &res)
{
    res.parameters = walking_param_;

    return true;
}

void CaptureStepModule::gazeboImuCallback(const sensor_msgs::ImuConstPtr &msg){
    //Imu State Callback From Gazebo
    gx = msg->angular_velocity.x;
    gy = msg->angular_velocity.y;
    gz = msg->angular_velocity.z;
    ax = msg->linear_acceleration.x;
    ay = msg->linear_acceleration.y;
    az = msg->linear_acceleration.z;
}

void CaptureStepModule::gazeboMagCallback(const geometry_msgs::Vector3StampedPtr &msg)
{
    mx = msg->vector.x;
    my = msg->vector.y;
    mz = msg->vector.z;
}


void CaptureStepModule::currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{

}

void CaptureStepModule::startWalking()
{
    ctrl_running_ = true;
    real_running_ = true;
    walking_state_ = WalkingReady;

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start walking");
}

void CaptureStepModule::stop()
{  walking_param_.x_move_amplitude = 0.0;
    walking_param_.y_move_amplitude = 0.0;
    walking_param_.angle_move_amplitude = 0.0;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Stop walking");
    walking_state_=WalkingDisable;
    ctrl_running_ = false;
    real_running_ = false;
}

bool CaptureStepModule::isRunning()
{
    return real_running_ || (walking_state_ == WalkingInitPose);
}

/* Control Interface
 * Omnidirectional Walking with Compliant Joints
 */

void CaptureStepModule::controlInterface(double m_VxTarget,double m_VyTarget,double m_VphiTarget)
{

    double Vx_norm,Vy_norm,Vphi_norm;
    // Linear Mapping
    Vx_norm = m_VxTarget/VX_MAX;
    Vy_norm = m_VyTarget/VY_MAX;
    Vphi_norm = m_VphiTarget/VPHI_MAX;

    // P-norm
    double P_norm;
    P_norm = pow((pow(fabs(Vx_norm),walking_param_.gait_vel_limit) + pow(fabs(Vy_norm),walking_param_.gait_vel_limit) + pow(fabs(Vphi_norm),walking_param_.gait_vel_limit)),
                 (double)(1/walking_param_.gait_vel_limit));
    if (P_norm > 1){
        Vx_norm = Vx_norm/P_norm;
        Vy_norm = Vy_norm/P_norm;
        Vphi_norm = Vphi_norm/P_norm;
    }

    // Update Velocity
    m_V.x() += fmax(-walking_param_.sagittal_acc,fmin((Vx_norm-m_V.x()),walking_param_.sagittal_acc));
    m_V.y() += fmax(-walking_param_.lateral_acc,fmin((Vy_norm-m_V.y()),walking_param_.lateral_acc));
    m_V.z() += fmax(-walking_param_.rotational_acc,fmin((Vphi_norm-m_V.z()),walking_param_.rotational_acc));

    // Update Motion Phase
    m_TauRate = (walking_param_.const_step_freq + (fabs(m_V.x())*walking_param_.sagittal_prop_step_freq) + (fabs(m_V.y())*walking_param_.lateral_prop_step_freq));
    m_Tau += m_TauRate;
    if (m_Tau > M_PI) m_Tau -= 2*M_PI;
}

/* Gait Control
 * Input V -> Control Interface -> Reference Trajectory -> Balance Control -> Motion Pattern -> Output
 * Omnidirectional Capture Steps for Bipedal Walking
 * Balanced Walking with Capture Steps
 *
 */

void CaptureStepModule::setV(Eigen::Vector3d m_Vtarget){
    //  m_V(0) = m_Vtarget(0) / VX_MAX;
    //  m_V(1) = m_Vtarget(1) / VY_MAX;
    //  m_V(2) = m_Vtarget(2) / VPHI_MAX;
}

CaptureStepModule::exchangeState CaptureStepModule::getReferenceTrajectory(Eigen::Vector3d m_V, double leg){
    CaptureStepModule::exchangeState desS;

    desS.Sx = m_V(0) * (sigma/C) * sinh(C*m_Tau);
    desS.Sxdot = m_V(0) * sigma * cosh(C*m_Tau);
    xi = delta * fabs(m_V(1)) * (omega - delta);
    if (leg == sgn(m_V(1)))
        desS.Sy=leg * delta;
    else desS.Sy = leg * xi;
    desS.Sydot = leg * C *sqrt(pow(desS.Sy, 2) - pow(alpha, 2));

    desS.halfT = 1/C * log((xi/alpha)+ sqrt((pow(xi, 2) / pow(alpha,2)) - 1));

    return desS;

}

Eigen::Vector2d CaptureStepModule::balanceControl(double leg, CaptureStepModule::COMState curC, CaptureStepModule::exchangeState desS){
    //calculate lateral zmp offset Zy
    CaptureStepModule::COMState newC;
    Eigen::Vector2d newStep;
    double Zx, Zy, T, stepT, fx, fy;
    stepT = 2*desS.halfT;

    Zy = ( desS.Sy*2*C*exp(C*stepT) - curC.Cy*C*(1 + exp(2*C*stepT)) + curC.Cydot*(1 - exp(2*C*stepT)) ) /
            (C*(exp(2*C*stepT) - 2*exp(C*stepT) + 1));

    double a = pow(desS.Sy - Zy, 2)/(pow(curC.Cy - Zy + (curC.Cydot/C),2));
    double b = (curC.Cy - Zy - (curC.Cydot/C)) / (curC.Cy - Zy + (curC.Cydot/C));

    //calculate step time T

    T = 1/C * log((desS.Sy - Zy) / (curC.Cy - Zy + (curC.Cydot/C)) + sqrt(a-b));

    //calculate saggital zmp offset Zx

    Zx = (desS.Sx + (desS.Sxdot/C) - exp(C*T) * (curC.Cx + (curC.Cxdot/C))) /
            (1 - exp(C*T));

    //calculate end-of-state com

    newC.Cx     = (curC.Cx - Zx)*cosh(C*T) + (curC.Cxdot/C) * sinh(C*T);
    newC.Cxdot  = (curC.Cx - Zx)*C*sinh(C*T) + curC.Cxdot * cosh(C*T);
    newC.Cy     = (curC.Cy - Zy)*cosh(C*T) + (curC.Cydot/C) * sinh(C*T);
    newC.Cydot  = (curC.Cy - Zy)*C*sinh(C*T) + curC.Cydot * cosh(C*T);
    // is chosen over acceleration-

    fx = (newC.Cxdot/C)*tanh(C*desS.halfT);
    fy = leg*sqrt(pow(newC.Cydot,2) / pow(C,2) + pow(alpha,2));

    newStep(0) = fx + newC.Cx;
    newStep(1) = fy + newC.Cy;

    return newStep;


}

void CaptureStepModule::calculateJointAngles(int leg, double& eta, LegPose& Leg){
    //m, angle

    double pitchLeg_ = (cos(Leg.hip.z)*Leg.hip.y) + (sin(Leg.hip.z)*Leg.hip.x);
    double rollLeg_ = (-sin(Leg.hip.z)*Leg.hip.y) + (cos(Leg.hip.z)*Leg.hip.x);
    lambda = acos(1-eta);
    hipYaw = Leg.hip.z;
    hipRoll = rollLeg_;
    hipPitch = pitchLeg_ - lambda;
    knee = 2*lambda;
    ankPitch = Leg.ankle.y - pitchLeg_ - lambda;
    ankRoll = Leg.ankle.x - rollLeg_;

    if(leg == RIGHT_LEG){ //right
        it[0] = hipYaw;
        it[1]  = hipRoll;    // present angle
        it[2]  = hipPitch;
        it[3]  = knee;
        it[4]  = ankPitch;
        it[5]  = ankRoll;
    }

    else{ //left
        it[6]  = hipYaw;
        it[7]  = hipRoll;
        it[8] = hipPitch;
        it[9]  = knee;
        it[10]  = ankPitch;
        it[11]  = ankRoll;
    }
}

void CaptureStepModule::calculateWalkPattern(int leg, double tau, double& eta, LegPose& Leg)
{
    // Initialize the parameters
    eta = 0.0;
    Leg.hip.x = 0.0;
    Leg.hip.y = 0.0;
    Leg.hip.z = 0.0;
    Leg.ankle.x = 0.0;
    Leg.ankle.y = 0.0;

    double tau2 = tau;
    if(leg==LEFT_LEG){
        tau2+= M_PI;
        if (tau2 > M_PI) tau2 -= 2*M_PI;
    }

    // Halt Pose
    eta += walking_param_.halt_pos_leg_ext;
    Leg.hip.x += (double)leg*walking_param_.halt_pos_leg_roll_ext;
    Leg.hip.y += walking_param_.halt_pos_leg_pitch_ext;
    Leg.ankle.x += walking_param_.halt_pos_foot_roll_ext;
    Leg.ankle.y += walking_param_.halt_pos_foot_pitch_ext;

    // Leg Lifting
    if(walking_param_.leg_lifting)
    {
        if (tau2 <= 0.0)
            eta += sin(tau2)*(walking_param_.const_ground_push + (walking_param_.propt_ground_push * fmax(fabs(m_V.x()),fabs(m_V.y()))));
        else
            eta += sin(tau2)*(walking_param_.const_step_height + (walking_param_.propt_step_height * fmax(fabs(m_V.x()),fabs(m_V.y()))));
    }

    if(walking_param_.leg_swing)
    {
        // Leg Swing
        double gamma=0.0;
        if (tau2 >= walking_param_.swing_start && tau2 < walking_param_.swing_stop)
        {
            gamma = cos(M_PI * (tau2 - walking_param_.swing_start) / (walking_param_.swing_stop - walking_param_.swing_start));
        } else if (tau2 >= walking_param_.swing_stop && tau2 < M_PI)
        {
            gamma = ((2 * (tau2 - walking_param_.swing_stop)) / ((2 * M_PI )- walking_param_.swing_stop + walking_param_.swing_start)) - 1.0;
        } else if (tau2 >= -M_PI && tau2 < walking_param_.swing_start)
        {
            gamma = ((2 * (tau2 + (2 * M_PI) - walking_param_.swing_stop)) / ((2 * M_PI) - walking_param_.swing_stop + walking_param_.swing_start)) - 1.0;
        }

        if (m_V.x() >= 0.0)
        {
            Leg.hip.y += (gamma*m_V.x()*walking_param_.sagittal_swg_fwd);
        } else if(m_V.x() < 0.0) {
            Leg.hip.y += (gamma*m_V.x()*walking_param_.sagittal_swg_bwd);
        }

        Leg.hip.x += ((-gamma*m_V.y()*walking_param_.lateral_swg) - ((double)leg * fmax(fabs(m_V.y())*walking_param_.lateral_swg_offset, fabs(m_V.z())*walking_param_.trng_lateral_swg_offset)));

        Leg.hip.z += ((gamma*m_V.z()*walking_param_.rotational_swg) - ((double)leg * fabs(m_V.z())*walking_param_.rotational_swg_offset));
    }

    // Lateral Hip Swing
    if(walking_param_.lateral_swing)
    {
        double tau_l, tau_r;

        if (tau2 < walking_param_.swing_start)
            tau_l = tau2 - walking_param_.swing_stop + (2 * M_PI);

        else if (tau2 > walking_param_.swing_stop)
            tau_l = tau2 - walking_param_.swing_stop;

        else
            tau_l = 0.0;


        if ((tau2 + M_PI) < walking_param_.swing_start)
            tau_r = tau2 - walking_param_.swing_stop + (3 * M_PI);

        else if ((tau2 + M_PI) > walking_param_.swing_stop)
            tau_r = tau2 - walking_param_.swing_stop + M_PI;

        else
            tau_r = 0.0;


        double delta = walking_param_.swing_start - walking_param_.swing_stop + (2 * M_PI);
        Leg.hip.x += walking_param_.lateral_hip_swg * (sin((tau_l*M_PI)/delta) - sin((tau_r*M_PI)/delta) );

    }

    // Leaning
    if(walking_param_.leaning)
    {
        if (m_V.x() >= 0.0)
        {
            Leg.hip.y += (m_V.x()*walking_param_.fwd_lean);
        } else if(m_V.x() < 0.0) {
            Leg.hip.y += (m_V.x()*walking_param_.bwd_lean);
        }
        Leg.hip.x += (-m_V.z()*fabs(m_V.x())*walking_param_.fwd_trng_lean);
    }

}

void CaptureStepModule::goToHaltPose(int sign){
    //m, angle
    double eta = walking_param_.halt_pos_leg_ext;
    double legRoll = (double)sign*walking_param_.halt_pos_leg_roll_ext;
    double legPitch = walking_param_.halt_pos_leg_pitch_ext;
    double footRoll = walking_param_.halt_pos_foot_roll_ext;
    double footPitch = walking_param_.halt_pos_foot_pitch_ext;

    //Degree
    lambda = acos(1-eta);
    hipYaw = 0;
    hipRoll = -sin(0)*legPitch + cos(0)*legRoll;
    hipPitch = (cos(0)*legPitch + sin(0)*legRoll) - lambda;
    knee = 2*lambda;
    ankPitch = footPitch - (cos(0)*legPitch + sin(0)*legRoll) - lambda;
    ankRoll = footRoll - (-sin(0)*legPitch + cos(0)*legRoll);

    if(sign == RIGHT_LEG){ //right
        it[0] = hipYaw;
        it[1]  = hipRoll;
        it[2]  = hipPitch;
        it[3]  = knee;
        it[4]  = ankPitch;
        it[5]  = ankRoll;
    }

    else{ //left
        it[6]  = hipYaw;
        it[7]  = hipRoll;
        it[8] = hipPitch;
        it[9]  = knee;
        it[10]  = ankPitch;
        it[11]  = ankRoll;
    }

}

void CaptureStepModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors){

    if(!enable_)
        return;
    ros::Time t = ros::Time::now();
    ros::Duration delta_t = t - last_t;
    //  robot_state_.updateState(gx, gy, gz, ax, ay, az, mx, my, mz);

    //     present angle
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
         state_iter != result_.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;

        robotis_framework::Dynamixel *dxl = NULL;
        std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(_joint_name);
        if (dxl_it != dxls.end())
            dxl = dxl_it->second;
        else
            continue;

        int joint_id = joint_table_[_joint_name];
        //    current_position_.coeff(0, joint_id) = dxl->dxl_state_->present_position_;
    }

    //  Pose measuredPose;

    //  measuredPose.leftLegPose.hip.setPos(op3_kd_->op3_link_data_[10]->joint_angle_,op3_kd_->op3_link_data_[12]->joint_angle_,op3_kd_->op3_link_data_[8]->joint_angle_);
    //  measuredPose.leftLegPose.knee.setPos(0,op3_kd_->op3_link_data_[14]->joint_angle_,0);
    //  measuredPose.leftLegPose.ankle.setPos(op3_kd_->op3_link_data_[18]->joint_angle_, op3_kd_->op3_link_data_[16]->joint_angle_,0);
    //  measuredPose.rightLegPose.hip.setPos(op3_kd_->op3_link_data_[9]->joint_angle_, op3_kd_->op3_link_data_[11]->joint_angle_, op3_kd_->op3_link_data_[7]->joint_angle_);
    //  measuredPose.rightLegPose.knee.setPos(0, op3_kd_->op3_link_data_[13]->joint_angle_,0);
    //  measuredPose.rightLegPose.ankle.setPos(op3_kd_->op3_link_data_[17]->joint_angle_, op3_kd_->op3_link_data_[15]->joint_angle_,0);

    if (walking_state_ == WalkingInitPose || walking_state_ == WalkingDisable)
    {
        goToHaltPose(1);
        goToHaltPose(-1);
        for (int id = 1; id < 12; id++)
        {
            double goal_position = it[id] * joint_axis_direction_(id);
            target_position_.coeffRef(0, id) = goal_position;
        }

        if (debug_print_)
            std::cout << "End moving : " << init_pose_count_ << std::endl;
    }

    else if (walking_state_ == WalkingReady)
    {

        //    double fusedX = robot_state_.robotFRollPR();
        //    double fusedY = robot_state_.robotFPitchPR();

        //    robot_model_.process(measuredPose, fusedX, fusedY);
        //    Eigen::Vector3d comVector = robot_model_.suppComVector();

        //    std::cout << "Com X : " << comVector(0,0) << "Com Y : " << comVector(1,0) << "Com Z : " << comVector(2,0) << std::endl;

        /*
    setV(Eigen::Vector3d(walking_param_.x_move_amplitude, walking_param_.y_move_amplitude,
                         walking_param_.angle_move_amplitude));

    //Calcultimeate Reference Trajectory
    desSR = getReferenceTrajectory(m_V, RIGHT_LEG);
    desSL = getReferenceTrajectory(m_V, LEFT_LEG);

    //Calculate Balance Control

    Eigen::Vector2d stepR = balanceControl(RIGHT_LEG, com, desSR);
    Eigen::Vector2d stepL = balanceControl(LEFT_LEG, com, desSL);

    //Calculate Step Size
*/

        controlInterface(walking_param_.x_move_amplitude, walking_param_.y_move_amplitude,
                         walking_param_.angle_move_amplitude);

        //Motion Pattern
        double R_Eta = 0;
        double L_Eta = 0;

//            std::cout << "Tau : " << m_Tau << " | " << sin(m_Tau) << std::endl;
        calculateWalkPattern(RIGHT_LEG, m_Tau, R_Eta, RightLeg);
        calculateWalkPattern(LEFT_LEG, m_Tau,  L_Eta, LeftLeg);

        calculateJointAngles(RIGHT_LEG, R_Eta, RightLeg);
        calculateJointAngles(LEFT_LEG, L_Eta, LeftLeg);

        double err_total = 0.0, err_max = 0.0;
        for (int idx = 0; idx < 12; idx++)
        {
            double goal_position = 0.0;
            goal_position = init_position_.coeff(0, idx) + it[idx] * joint_axis_direction_(idx);

            target_position_.coeffRef(0, idx) = goal_position;
        }
    }

    // set result
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
         state_it != result_.end(); state_it++)
    {
        std::string joint_name = state_it->first;
        int joint_index = joint_table_[joint_name];

        result_[joint_name]->goal_position_ = target_position_.coeff(0, joint_index);
//        std::cout << joint_name << " : " << result_[joint_name]->goal_position_ <<std::endl;

    }

    last_t = t;
}

void CaptureStepModule::updateState(){
    //Com State
    //  op3_kd_->calcForwardKinematics(0);
    //  //op3_kd_->calcForwardVelocity(23);

    //  double totalMass = op3_kd_->calcTotalMass(29);
    //  Eigen::MatrixXd mc = op3_kd_->calcMC(29);
    //  Eigen::MatrixXd comPos = op3_kd_->calcCOM(mc);

    //  //  Eigen::MatrixXd P = op3_kd_ -> calcP(0);
    //  //  Eigen::MatrixXd comVel = P / totalMass;
    //  com.Cx = comPos(1,1);
    //  com.Cy = comPos(2,1);

}

void CaptureStepModule::saveWalkingParam(std::string &path)
{
    YAML::Node node;
    try
    {
        // load yaml
        node = YAML::LoadFile(path.c_str());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return;
    }

    //  YAML::Node doc = node["walk_zmp"];

    node["halt_pos_leg_ext"] =  walking_param_.halt_pos_leg_ext;
    node["halt_pos_leg_roll_ext"] =  walking_param_.halt_pos_leg_roll_ext;
    node["halt_pos_leg_pitch_ext"] =  walking_param_.halt_pos_leg_pitch_ext;
    node["halt_pos_foot_roll_ext"] =  walking_param_.halt_pos_foot_roll_ext;
    node["halt_pos_foot_pitch_ext"] =  walking_param_.halt_pos_foot_pitch_ext;
    node["const_ground_push"] =  walking_param_.const_ground_push ;
    node["propt_ground_push"] =  walking_param_.propt_ground_push;
    node["const_step_height"] =  walking_param_.const_step_height;
    node["propt_step_height"] =  walking_param_.propt_step_height;

    node["swing_start"] =  walking_param_.swing_start;
    node["swing_stop"] =  walking_param_.swing_stop;
    node["sagittal_swg_fwd"] =  walking_param_.sagittal_swg_fwd;
    node["sagittal_swg_bwd"] =  walking_param_.sagittal_swg_bwd;
    node["lateral_swg"] =  walking_param_.lateral_swg;
    node["lateral_swg_offset"] =  walking_param_.lateral_swg_offset;
    node["trng_lateral_swg_offset"] =  walking_param_.trng_lateral_swg_offset;
    node["rotational_swg"] =  walking_param_.rotational_swg;
    node["rotational_swg_offset"] =  walking_param_.rotational_swg_offset;

    node["lateral_hip_swg"] =  walking_param_.lateral_hip_swg;

    node["fwd_lean"] =  walking_param_.fwd_lean;
    node["bwd_lean"] =  walking_param_.bwd_lean;
    node["fwd_trng_lean"] =  walking_param_.fwd_trng_lean;
    node["gait_vel_limit"] =  walking_param_.gait_vel_limit;
    node["sagittal_acc"] =  walking_param_.sagittal_acc;
    node["lateral_acc"] =  walking_param_.lateral_acc;
    node["rotational_acc"] =  walking_param_.rotational_acc;

    node["const_step_freq"] =  walking_param_.const_step_freq;
    node["sagittal_prop_step_freq"] =  walking_param_.sagittal_prop_step_freq;
    node["lateral_prop_step_freq"] =  walking_param_.lateral_prop_step_freq;

    // output to file
    std::ofstream fout(path.c_str());
    fout << node;
}

void CaptureStepModule::loadWalkingParam(const std::string &path)
{
    YAML::Node doc;

    // load yaml
    doc = YAML::LoadFile(path.c_str());


    walking_param_.halt_pos_leg_ext = doc["halt_pos_leg_ext"].as<double>();
    walking_param_.halt_pos_leg_roll_ext = doc["halt_pos_leg_roll_ext"].as<double>();
    walking_param_.halt_pos_leg_pitch_ext = doc["halt_pos_leg_pitch_ext"].as<double>();
    walking_param_.halt_pos_foot_roll_ext = doc["halt_pos_foot_roll_ext"].as<double>();
    walking_param_.halt_pos_foot_pitch_ext = doc["halt_pos_foot_pitch_ext"].as<double>();
    walking_param_.const_ground_push = doc["const_ground_push"].as<double>();
    walking_param_.propt_ground_push = doc["propt_ground_push"].as<double>();
    walking_param_.const_step_height = doc["const_step_height"].as<double>();
    walking_param_.propt_step_height = doc["propt_step_height"].as<double>();

    walking_param_.swing_start = doc["swing_start"].as<double>();
    walking_param_.swing_stop= doc["swing_stop"].as<double>();
    walking_param_.sagittal_swg_fwd = doc["sagittal_swg_fwd"].as<double>();
    walking_param_.sagittal_swg_bwd = doc["sagittal_swg_bwd"].as<double>();
    walking_param_.lateral_swg = doc["lateral_swg"].as<double>();
    walking_param_.lateral_swg_offset= doc["lateral_swg_offset"].as<double>();
    walking_param_.trng_lateral_swg_offset = doc["trng_lateral_swg_offset"].as<double>();
    walking_param_.rotational_swg = doc["rotational_swg"].as<double>();
    walking_param_.rotational_swg_offset = doc["rotational_swg_offset"].as<double>();

    walking_param_.lateral_hip_swg = doc["lateral_hip_swg"].as<double>();

    walking_param_.fwd_lean= doc["fwd_lean"].as<double>();
    walking_param_.bwd_lean = doc["bwd_lean"].as<double>();
    walking_param_.fwd_trng_lean = doc["fwd_trng_lean"].as<double>();
    walking_param_.gait_vel_limit = doc["gait_vel_limit"].as<double>();
    walking_param_.sagittal_acc = doc["sagittal_acc"].as<double>();
    walking_param_.lateral_acc = doc["lateral_acc"].as<double>();
    walking_param_.rotational_acc = doc["rotational_acc"].as<double>();
    walking_param_.const_step_freq = doc["const_step_freq"].as<double>();
    walking_param_.sagittal_prop_step_freq = doc["sagittal_prop_step_freq"].as<double>();
    walking_param_.lateral_prop_step_freq = doc["lateral_prop_step_freq"].as<double>();

}

void CaptureStepModule::onModuleEnable()
{
    walking_state_ = WalkingInitPose;
    ROS_INFO("Walking Init Pose");
}

void CaptureStepModule::onModuleDisable()
{
    ROS_INFO("Walking Disable");
    walking_state_ = WalkingDisable;
}

}

