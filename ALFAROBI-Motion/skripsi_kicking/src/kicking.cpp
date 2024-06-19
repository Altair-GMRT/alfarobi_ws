// #include "kicking/kicking.h"

// namespace robotis_op
// {
// Kicking::Kicking()
//   : 
//     control_cycle_msec_(8),
//     mbc(ros::package::getPath("kicking")),
//     so(ros::package::getPath("kicking")),
//     densis(ros::package::getPath("kicking"))
// {
//     enable_ = false;
//     module_name_ = "kicking";
//     control_mode_ = robotis_framework::PositionControl;
//     kicking_state_ = KickingInitPose;

//     op3_kd_ = new robotis_op::OP3KinematicsDynamics(robotis_op::WholeBody);

//     // result
//     result_["r_hip_yaw"] = new robotis_framework::DynamixelState();
//     result_["r_hip_roll"] = new robotis_framework::DynamixelState();
//     result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
//     result_["r_knee"] = new robotis_framework::DynamixelState();
//     result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
//     result_["r_ank_roll"] = new robotis_framework::DynamixelState();

//     result_["l_hip_yaw"] = new robotis_framework::DynamixelState();
//     result_["l_hip_roll"] = new robotis_framework::DynamixelState();
//     result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
//     result_["l_knee"] = new robotis_framework::DynamixelState();
//     result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
//     result_["l_ank_roll"] = new robotis_framework::DynamixelState();

//     result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
//     result_["r_sho_roll"] = new robotis_framework::DynamixelState();
//     result_["r_el"] = new robotis_framework::DynamixelState();

//     result_["l_sho_pitch"] = new robotis_framework::DynamixelState();
//     result_["l_sho_roll"] = new robotis_framework::DynamixelState();
//     result_["l_el"] = new robotis_framework::DynamixelState();

//     // joint table
//     joint_table_["r_hip_yaw"] = 0;
//     joint_table_["r_hip_roll"] = 1;
//     joint_table_["r_hip_pitch"] = 2;
//     joint_table_["r_knee"] = 3;
//     joint_table_["r_ank_pitch"] = 4;
//     joint_table_["r_ank_roll"] = 5;

//     joint_table_["l_hip_yaw"] = 6;
//     joint_table_["l_hip_roll"] = 7;
//     joint_table_["l_hip_pitch"] = 8;
//     joint_table_["l_knee"] = 9;
//     joint_table_["l_ank_pitch"] = 10;
//     joint_table_["l_ank_roll"] = 11;

//     joint_table_["r_sho_pitch"] = 12;
//     joint_table_["r_sho_roll"] = 13;
//     joint_table_["r_el"] = 14;

//     joint_table_["l_sho_pitch"] = 15;
//     joint_table_["l_sho_roll"] = 16;
//     joint_table_["l_el"] = 17;


//     //offset
//     joint[0]  = "r_hip_yaw";
//     joint[1]  = "r_hip_roll";
//     joint[2]  = "r_hip_pitch";
//     joint[3]  = "r_knee";
//     joint[4]  = "r_ank_pitch";
//     joint[5]  = "r_ank_roll";

//     joint[6]  = "l_hip_yaw";
//     joint[7]  = "l_hip_roll";
//     joint[8]  = "l_hip_pitch";
//     joint[9]  = "l_knee";
//     joint[10] = "l_ank_pitch";
//     joint[11] = "l_ank_roll";

//     //op3_offset
//     op3_joint[1]  = "r_sho_pitch";
//     op3_joint[2]  = "l_sho_pitch";
//     op3_joint[3]  = "r_sho_roll";
//     op3_joint[4]  = "l_sho_roll";
//     op3_joint[5]  = "r_el";
//     op3_joint[6]  = "l_el";
//     op3_joint[7]  = "r_hip_yaw";
//     op3_joint[8]  = "l_hip_yaw";
//     op3_joint[9]  = "r_hip_roll";
//     op3_joint[10]  = "l_hip_roll";
//     op3_joint[11]  = "r_hip_pitch";
//     op3_joint[12]  = "l_hip_pitch";
//     op3_joint[13]  = "r_knee";
//     op3_joint[14]  = "l_knee";
//     op3_joint[15]  = "r_ank_pitch";
//     op3_joint[16]  = "l_ank_pitch";
//     op3_joint[17]  = "r_ank_roll";
//     op3_joint[18]  = "l_ank_roll";
//     op3_joint[19]  = "head_pan";
//     op3_joint[20]  = "head_tilt";

//     R_X=0; R_Y=0; R_Z=0; R_X_Kick  = 0;  R_Y_Kick  = 0;  R_Z_Kick = 0; R_P = 0;
//     L_X=0; L_Y=0; L_Z=0; L_X_Kick  = 0;  L_Y_Kick  = 0;  L_Z_Kick = 0; L_P = 0;

//     target_position_ = MatrixXd::Zero(1, result_.size());
//     current_position_ = MatrixXd::Zero(1, result_.size());
//     current_velocity_ = MatrixXd::Zero(1, result_.size());
//     init_position_ = Eigen::MatrixXd::Zero(1, result_.size());
//     joint_axis_direction_ = MatrixXi::Zero(1, result_.size());


//     BASE     = Affine3d(Translation3d(Vector3d(0,0,0)));
//     R_HIP    = Affine3d(Translation3d(Vector3d(k.HIP_OFFSET_X,-k.LEG_SIDE_OFFSET,-k.HIP_OFFSET_Z)));
//     R_KNEE   = Affine3d(Translation3d(Vector3d(0,0,-k.THIGH_LENGTH)));
//     R_ANKLE  = Affine3d(Translation3d(Vector3d(0,0,-k.CALF_LENGTH)));
//     R_FOOT   = Affine3d(Translation3d(Vector3d(0,0,-k.ANKLE_LENGTH)));
//     m_R_FOOT = Affine3d(Translation3d(Vector3d(0,0,0)));
//     L_HIP    = Affine3d(Translation3d(Vector3d(k.HIP_OFFSET_X,k.LEG_SIDE_OFFSET,-k.HIP_OFFSET_Z)));
//     L_KNEE   = Affine3d(Translation3d(Vector3d(0,0,-k.THIGH_LENGTH)));
//     L_ANKLE  = Affine3d(Translation3d(Vector3d(0,0,-k.CALF_LENGTH)));
//     L_FOOT   = Affine3d(Translation3d(Vector3d(0,0,-k.ANKLE_LENGTH)));
//     m_L_FOOT = Affine3d(Translation3d(Vector3d(0,0,0)));

//     VectorXd LFootWorld(4);
//     VectorXd RFootWorld(4);

//     torqueRoll = 0; torquePitch = 0;

//     m_imuAcc = Eigen::Vector3d::Zero(3);
//     m_imuAccFil = Eigen::Vector3d::Zero(3);
//     rawData = Eigen::Vector3d::Zero(3);
//     filteredData = Eigen::Vector3d::Zero(3);
//     cumsum = Eigen::Vector3d::Zero(3);
// }

// Kicking::~Kicking()
// {
//     queue_thread_.join();
// }

// void Kicking::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
// {
//     queue_thread_ = boost::thread(boost::bind(&Kicking::queueThread, this));
//     control_cycle_msec_ = control_cycle_msec;
//     real_running_ = false;

//     config_path_ = ros::package::getPath("op3_manager") + "/config/GlobalConfig.yaml";
//     offset_path_ = ros::package::getPath("op3_manager") + "/config/offset.yaml";
//     kickManip_path_ = ros::package::getPath("kicking") + "/config/kickManip.yaml";
//     physicalParam_path_ = ros::package::getPath("op3_kinematics_dynamics") + "/config/physicalParam.yaml";
//     feedbackParam_path_ = ros::package::getPath("kicking") + "/config/feedbackParam.yaml";

//     ROS_DEBUG("Motion: Kicking entry");

//     loadConfig();
//     loadKickManip();
//     loadFeedbackParam();

//     m_imuAcc = Eigen::Vector3d::Zero(3);
//     m_imuAccFil = Eigen::Vector3d::Zero(3);
//     rawData = Eigen::Vector3d::Zero(3);
//     filteredData = Eigen::Vector3d::Zero(3);
//     cumsum = Eigen::Vector3d::Zero(3);
//     // erase vector before resetting with new window length
//     bufferAccX.clear();
//     bufferAccY.clear();
//     bufferAccZ.clear();
//     for(int i=0; i<MeasurementsToAverage; i++)
//     {
//         bufferAccX.push_back(0.0);
//         bufferAccY.push_back(0.0);
//         bufferAccZ.push_back(0.0);
//     }
//     for(int i=0; i < 2; i++)
//     {
//         so.r_ank_pitch.servoStates.push_back(0.0);
//         so.l_ank_pitch.servoStates.push_back(0.0);
//     }
//     feedbackActive = false;
//     servoFeedbackActive = false;
//     this->stop();
// }

// void Kicking::queueThread()
// {
//     ros::CallbackQueue callback_queue;
//     nh_.setCallbackQueue(&callback_queue);

//     ros::Subscriber kicking_command_sub_ = nh_.subscribe("/alfarobi/kicking/command", 0, &Kicking::kickingCommandCallback, this);
//     ros::Subscriber kicking_param_sub_ = nh_.subscribe("/alfarobi/kicking/kicking_set_params", 0, &Kicking::kickingParamCallback, this);
//     ros::ServiceServer get_kicking_param_server = nh_.advertiseService("/alfarobi/kicking/kicking_get_params", &Kicking::getKickingParamCallback, this);

//     ros::Subscriber angle_sub_ = nh_.subscribe("/arduino_controller/imu", 0, &Kicking::imuCallback, this);
//     // ros::Subscriber angle_velocity_sub_ = nh_.subscribe("/alfarobi/angular_velocity", 0, &Kicking::angleCallback, this);
//     kick_status_pub_ = nh_.advertise<std_msgs::String>("/alfarobi/kicking/status",0);
//     data_log_pub_ =  nh_.advertise<data_log_msgs::DataLog>("/alfarobi/data_log",0);
//     densis_pub_ = nh_.advertise<densis_msgs::densis>("/alfarobi/densis",0);

//     ros::WallDuration duration(control_cycle_msec_ / 1000.0);
//     while(nh_.ok())
//         callback_queue.callAvailable(duration);
// }

// bool Kicking::isRunning()
// {
//     return real_running_ || (kicking_state_ == KickingInitPose);
// }

// void Kicking::start()
// {
//     if(!m_active)
//     {
//         m_active = true;
//         m_start = true;
//         rollAnkle << 0,0;
//         feedback << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//     }
// }

// void Kicking::stop()
// {
//     if(m_active)
//     {
//         m_active = false;
//         m_start = false;
//         feedback << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//         m_delta_time = 0;
//         //For Densis Input
//         for(int i=0; i < 2; i++)
//         {
//             so.r_ank_pitch.servoStates.push_back(0.0);
//             so.l_ank_pitch.servoStates.push_back(0.0);
//         }
//     }

// }

// void Kicking::kickingCommandCallback(const std_msgs::String::ConstPtr &_msg)
// {
//     if(_msg->data == "right_kick")
//     {
//         start();
//         m_position = Right_Kick;
//     }
//     else if(_msg->data == "left_kick")
//     {
//         start();
//         m_position = Left_Kick;
//     }
//     else if(_msg->data == "rsk")
//     {
//         start();
//         m_position = Right_Side_Kick;
//     }
//     else if(_msg->data == "lsk")
//     {
//         start();
//         m_position = Left_Side_Kick;
//     }
//     else if (_msg->data == "save")
//         saveConfig(config_path_);
//     else if (_msg->data == "stop")
//         stop();
// }

// void Kicking::kickingParamCallback(const kicking_module_msgs::KickingParam::ConstPtr &msg)
// {
//     kicking_param_ = *msg;
// }

// bool Kicking::getKickingParamCallback(kicking_module_msgs::GetKickingParam::Request &req, kicking_module_msgs::GetKickingParam::Response &res)
// {
//     res.parameters = kicking_param_;
//     return true;
// }

// // void Kicking::imuCallback(const read_data::controller::ConstPtr& msg)
// void Kicking::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
// {
//     imu(0) = msg->angular_velocity.x;
//     imu(1) = msg->angular_velocity.y;
//     imu(2) = msg->angular_velocity.z;

//     imu_orientation.x() = msg->orientation.x;
//     imu_orientation.y() = msg->orientation.y;
//     imu_orientation.z() = msg->orientation.z;
//     imu_orientation.w() = msg->orientation.w;

//     m_imuGyr = imu;
//     m_imuOri = robotis_framework::convertQuaternionToRPY(imu_orientation);
//     m_imuAcc.x() = msg->linear_acceleration.x;
//     m_imuAcc.y() = msg->linear_acceleration.y;
//     m_imuAcc.z() = msg->linear_acceleration.z;

//     // // ROSSERIAL
//     // m_imuGyr.x() = msg->gyro.x*PI/180;
//     // m_imuGyr.y() = msg->gyro.y*PI/180;
//     // m_imuGyr.z() = msg->gyro.z*PI/180;

//     // m_imuOri.x() = msg->orientation.x*PI/180;
//     // m_imuOri.y() = msg->orientation.y*PI/180;
//     // m_imuOri.z() = msg->orientation.z*PI/180;

//     // m_imuAcc.x() = msg->acceleration.x;
//     // m_imuAcc.y() = msg->acceleration.y;
//     // m_imuAcc.z() = msg->acceleration.z;
// }

// void Kicking::loadKickManip()
// {
//     YAML::Node kp;

//     try
//     {
//         kp = YAML::LoadFile(kickManip_path_.c_str());
//     }
//     catch(const std::exception& e)
//     {
//         ROS_ERROR("Fail to load yaml file.");
//         return;
//     }
//     printKickVar    = kp["printKickVar"].as<bool>();
//     feedbackActive  = kp["feedbackActive"].as<bool>();
//     servoFeedbackActive = kp["servoFeedbackActive"].as<bool>();
//     initPose        = kp["initPose"].as<bool>();
//     ankleMode  = kp["ankleMode"].as<int>();
//     liftFoot  = kp["liftFoot"].as<bool>();
//     holdPose  = kp["holdPose"].as<bool>();
//     FKfromgoalPos = kp["FKfromgoalPos"].as<bool>();
//     samplingRate = kp["samplingRateEMA"].as<double>();

//     YAML::Node openLoop = kp["openLoop"];
//     L_Body_Tilt     = openLoop["L_Tilt"].as<double>();
//     R_Body_Tilt     = openLoop["R_Tilt"].as<double>();
//     L_X_Manip       = openLoop["L_X_Manip"].as<double>();
//     R_X_Manip       = openLoop["R_X_Manip"].as<double>();
//     ballPosX        = openLoop["ballPosX"].as<double>();
//     ballPosZ        = openLoop["ballPosZ"].as<double>();
//     R_Z_Lift        = openLoop["R_Z_Lift"].as<double>();
//     L_Z_Lift        = openLoop["L_Z_Lift"].as<double>();
//     std::cout<<"Done Loading Kick Manip"<<std::endl;
// }

// void Kicking::loadFeedbackParam()
// {
//     YAML::Node fb;

//     try
//     {
//         fb = YAML::LoadFile(feedbackParam_path_.c_str());
//     }
//     catch(const std::exception& e)
//     {
//         ROS_ERROR("Fail to load FEEDBACK yaml file.");
//         return;
//     }
//     MeasurementsToAverage = fb["M"].as<int>();
//     YAML::Node fD = fb["DSP"];
//     fbDSP.ZMPrefX       = fD["ZMPrefX"].as<double>();
//     fbDSP.ZMPrefY       = fD["ZMPrefY"].as<double>();
//     fbDSP.KrZMPx        = fD["KrZMPx"].as<double>();
//     fbDSP.KrZMPy        = fD["KrZMPy"].as<double>();
//     fbDSP.KAngleRoll    = fD["KAngleRoll"].as<double>();
//     fbDSP.KGyroRoll     = fD["KGyroRoll"].as<double>();
//     fbDSP.KAnglePitch   = fD["KAnglePitch"].as<double>();
//     fbDSP.KGyroPitch    = fD["KGyroPitch"].as<double>();
//     fbDSP.pitchErrorMAX = fD["pitchErrorMAX"].as<double>();
//     fbDSP.pitchErrorMIN = fD["pitchErrorMIN"].as<double>();
//     fbDSP.rollErrorMAX  = fD["rollErrorMAX"].as<double>();
//     fbDSP.rollhErrorMIN = fD["rollhErrorMIN"].as<double>();

//     YAML::Node fL = fb["LSSP"];
//     fbLSSP.ZMPrefX       = fL["ZMPrefX"].as<double>();
//     fbLSSP.ZMPrefY       = fL["ZMPrefY"].as<double>();
//     fbLSSP.KrZMPx        = fL["KrZMPx"].as<double>();
//     fbLSSP.KrZMPy        = fL["KrZMPy"].as<double>();
//     fbLSSP.KAngleRoll    = fL["KAngleRoll"].as<double>();
//     fbLSSP.KGyroRoll     = fL["KGyroRoll"].as<double>();
//     fbLSSP.KAnglePitch   = fL["KAnglePitch"].as<double>();
//     fbLSSP.KGyroPitch    = fL["KGyroPitch"].as<double>();
//     fbLSSP.pitchErrorMAX = fL["pitchErrorMAX"].as<double>();
//     fbLSSP.pitchErrorMIN = fL["pitchErrorMIN"].as<double>();
//     fbLSSP.rollErrorMAX  = fL["rollErrorMAX"].as<double>();
//     fbLSSP.rollhErrorMIN = fL["rollhErrorMIN"].as<double>();

//     YAML::Node fR = fb["RSSP"];
//     fbRSSP.ZMPrefX       = fR["ZMPrefX"].as<double>();
//     fbRSSP.ZMPrefY       = fR["ZMPrefY"].as<double>();
//     fbRSSP.KrZMPx        = fR["KrZMPx"].as<double>();
//     fbRSSP.KrZMPy        = fR["KrZMPy"].as<double>();
//     fbRSSP.KAngleRoll    = fR["KAngleRoll"].as<double>();
//     fbRSSP.KGyroRoll     = fR["KGyroRoll"].as<double>();
//     fbRSSP.KAnglePitch   = fR["KAnglePitch" ].as<double>();
//     fbRSSP.KGyroPitch    = fR["KGyroPitch"].as<double>();
//     fbRSSP.pitchErrorMAX = fR["pitchErrorMAX"].as<double>();
//     fbRSSP.pitchErrorMIN = fR["pitchErrorMIN"].as<double>();
//     fbRSSP.rollErrorMAX  = fR["rollErrorMAX"].as<double>();
//     fbRSSP.rollhErrorMIN = fR["rollhErrorMIN"].as<double>();
//     std::cout<<"Done Loading Feedback"<<std::endl;
// }

// void Kicking::loadConfig()
// {
//     YAML::Node offset_node;
//     YAML::Node config_node;

//     try
//     {
//         offset_node = YAML::LoadFile(offset_path_.c_str());
//         config_node = YAML::LoadFile(config_path_.c_str());
//     }
//     catch(const std::exception& e)
//     {
//         ROS_ERROR("Fail to load yaml file.");
//         return;
//     }

//     YAML::Node offset_ = offset_node["offset"];

//     for(int idx = 0; idx <12; idx++)
//     {
//         offset[idx] = offset_[joint[idx]].as<double>();
//     }

//     for(int idx = 1; idx<21; idx++)
//     {
//         op3_offset[idx] = offset_[op3_joint[idx]].as<double>();
//     }

//     ROS_INFO("Load yaml file KICKING");
//     YAML::Node config_ = config_node["kicking"];

//     kicking_param_.Body_Height          = config_["Body_Height"].as<double>();
//     kicking_param_.Body_Tilt            = config_["Body_Tilt"].as<double>();
//     kicking_param_.Leg_X                = config_["Leg_X"].as<double>();
//     kicking_param_.Leg_Y                = config_["Leg_Y"].as<double>();
//     kicking_param_.Leg_Z                = config_["Leg_Z"].as<double>();
//     kicking_param_.X_Retract            = config_["X_Retract"].as<double>();
//     kicking_param_.Z_Retract            = config_["Z_Retract"].as<double>();

//     kicking_param_.Leg_R_X              = config_["Leg_R_X"].as<double>();
//     kicking_param_.R_Shift              = config_["R_Shift"].as<double>();
//     kicking_param_.R_Lift               = config_["R_Lift"].as<double>();
//     kicking_param_.R_CoolDown           = config_["R_CoolDown"].as<double>();
//     kicking_param_.R_Roll               = config_["R_Roll"].as<double>();

//     kicking_param_.Leg_L_X              = config_["Leg_L_X"].as<double>();
//     kicking_param_.L_Shift              = config_["L_Shift"].as<double>();
//     kicking_param_.L_Lift               = config_["L_Lift"].as<double>();
//     kicking_param_.L_CoolDown           = config_["L_CoolDown"].as<double>();
//     kicking_param_.L_Roll               = config_["L_Roll"].as<double>();

//     kicking_param_.Time_Shift           = config_["Time_Shift"].as<double>();
//     kicking_param_.Time_Lift            = config_["Time_Lift"].as<double>();
//     kicking_param_.Time_Retract         = config_["Time_Retract"].as<double>();
//     kicking_param_.Time_Kick            = config_["Time_Kick"].as<double>();
//     kicking_param_.Time_CoolDown        = config_["Time_CoolDown"].as<double>();
//     kicking_param_.Time_BringDown       = config_["Time_BringDown"].as<double>();

//     kicking_param_.Gain_Angle_Pitch     = config_["Gain_Angle_Pitch"].as<double>();
//     kicking_param_.Gain_Velocity_Pitch  = config_["Gain_Velocity_Pitch"].as<double>();
//     kicking_param_.Gain_Angle_Roll      = config_["Gain_Angle_Roll"].as<double>();
//     kicking_param_.Gain_Velocity_Roll   = config_["Gain_Velocity_Roll"].as<double>();
//     kicking_param_.Gain_Integral        = config_["Gain_Integral"].as<double>();
// }

// void Kicking::saveConfig(const std::string &path)
// {
//     YAML::Node config_node;

//     try
//     {
//         config_node = YAML::LoadFile(config_path_.c_str());
//     }
//     catch(const std::exception& e)
//     {
//         ROS_ERROR("Fail to load yaml file.");
//         return;
//     }

//     YAML::Node config_ = config_node["kicking"];

//     config_["Body_Height"]         = kicking_param_.Body_Height;
//     config_["Body_Tilt"]           = kicking_param_.Body_Tilt;
//     config_["Leg_X"]               = kicking_param_.Leg_X;
//     config_["Leg_Y"]               = kicking_param_.Leg_Y;
//     config_["Leg_Z"]               = kicking_param_.Leg_Z;
//     config_["X_Retract"]           = kicking_param_.X_Retract;
//     config_["Z_Retract"]           = kicking_param_.Z_Retract;

//     config_["Leg_R_X"]             = kicking_param_.Leg_R_X;
//     config_["R_Shift"]             = kicking_param_.R_Shift;
//     config_["R_Lift"]              = kicking_param_.R_Lift;
//     config_["R_CoolDown"]          = kicking_param_.R_CoolDown;
//     config_["R_Roll"]              = kicking_param_.R_Roll;

//     config_["Leg_L_X"]             = kicking_param_.Leg_L_X;
//     config_["L_Shift"]             = kicking_param_.L_Shift;
//     config_["L_Lift"]              = kicking_param_.L_Lift;
//     config_["L_CoolDown"]          = kicking_param_.L_CoolDown;
//     config_["L_Roll"]              = kicking_param_.L_Roll;

//     config_["Time_Shift"]          = kicking_param_.Time_Shift;
//     config_["Time_Lift"]           = kicking_param_.Time_Lift;
//     config_["Time_Retract"]        = kicking_param_.Time_Retract;
//     config_["Time_Kick"]           = kicking_param_.Time_Kick;
//     config_["Time_CoolDown"]       = kicking_param_.Time_CoolDown;
//     config_["Time_BringDown"]      = kicking_param_.Time_BringDown;

//     config_["Gain_Angle_Pitch"]    =  kicking_param_.Gain_Angle_Pitch;
//     config_["Gain_Velocity_Pitch"] =  kicking_param_.Gain_Velocity_Pitch;
//     config_["Gain_Angle_Roll"]     =  kicking_param_.Gain_Angle_Roll;
//     config_["Gain_Velocity_Roll"]  =  kicking_param_.Gain_Velocity_Roll;
//     config_["Gain_Integral"]       =  kicking_param_.Gain_Integral;
//     config_["ballPosX"] = ballPosX;
//     config_["ballPosZ"] = ballPosZ;
//     // output to file
//     std::ofstream fout(config_path_.c_str());
//     fout << config_node;
// }

// void Kicking::onModuleEnable()
// {
//     firstExc = true;
//     kicking_state_ = KickingEnable;
//     ROS_INFO("Kicking Enable");
// }

// void Kicking::onModuleDisable()
// {
//     kicking_state_ = KickingDisable;
//     ROS_INFO("Kicking Disable");
// }

// void Kicking::setValue()
// {
//     for (int idx = 0; idx < 18; idx++)
//     {
//         if(initPose == true)
//         {
//            double goal_position = init_position_.coeff(0, idx) + 0;
//            target_position_.coeffRef(0, idx) = goal_position;
//         }
//         else
//         {
//             if(idx == 5) //R_Ankle_Roll
//                 getJoints(idx) += rollAnkle(0);
//             else if(idx == 11) //L_Ankle_Roll
//                 getJoints(idx) += rollAnkle(1);

//             // Servo Feedback Angle Limitter
//             if(feedback(4) <= -30*DEGREE2RADIAN) //-60
//                 feedback(4)=-30*DEGREE2RADIAN; //Right_Ankle Pitch
//             else if(feedback(4) > 30*DEGREE2RADIAN) //20
//                 feedback(4)=30*DEGREE2RADIAN; //Right_Ankle Pitch
//             else if(feedback(5) <= -30*DEGREE2RADIAN) //-30
//                 feedback(5)=-30*DEGREE2RADIAN; //Right Ankle Roll
//             else if(feedback(5) > 30*DEGREE2RADIAN) //30
//                 feedback(5)=30*DEGREE2RADIAN; //Right Ankle Roll
//             else if(feedback(10) <= -30*DEGREE2RADIAN) //-60
//                 feedback(10)=-30*DEGREE2RADIAN; //Left Ankle Pitch
//             else if(feedback(10) > 30*DEGREE2RADIAN) //20
//                 feedback(10)=30*DEGREE2RADIAN; //Left Ankle Pitch
//             else if(feedback(11) <= -30*DEGREE2RADIAN) //-30
//                 feedback(11)=-30*DEGREE2RADIAN; //Left Ankle Roll
//             else if(feedback(11) > 30*DEGREE2RADIAN) //30
//                 feedback(11)=30*DEGREE2RADIAN; //Left Ankle Roll

//             double goal_position = init_position_.coeff(0, idx) + (getJoints(idx) * joint_axis_direction_(idx) + (feedback(idx) * joint_axis_direction_(idx)));
//             target_position_.coeffRef(0, idx) = goal_position;
//         }
//     }


//     // set result
//     for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
//             state_it != result_.end(); state_it++)
//     {
//         std::string joint_name = state_it->first;
//         int joint_index = joint_table_[joint_name];

//         result_[joint_name]->goal_position_ = target_position_.coeff(0, joint_index);
//         if(FKfromgoalPos == true)
//         {
//             op3_kd_->setJointPos(joint_name, result_[joint_name]->goal_position_);//getJoints(joint_index));
//         }
//     }
// }

// void Kicking::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
// {
//     auto t1 = boost::chrono::high_resolution_clock::now();
//     if(enable_ == false)
//         return;

//     current_joint_pos = KDL::JntArray(12);
//     current_joint_vel = KDL::JntArray(12);

//     //                     R_HIP_YAW,   R_HIP_ROLL, R_HIP_PITCH, R_KNEE,      R_ANKLE_PITCH, R_ANKLE_ROLL,
//     //                     L_HIP_YAW,   L_HIP_ROLL, L_HIP_PITCH, L_KNEE,      L_ANKLE_PITCH, L_ANKLE_ROLL,
//     //                     R_ARM_PITCH, R_ARM_ROLL, R_ARM_ELBOW, L_ARM_PITCH, L_ARM_ROLL,    L_ARM_ELBOW
//     joint_axis_direction_ <<    -1,        -1,            1,          1,          -1,            1,
//                                 -1,        -1,           -1,         -1,           1,            1,
//                                 -1,         1,           -1,          1,          -1,            1;

//     init_position_        <<    0.0,       0.0,          0.0,        0.0,         0.0,          0.0,
//                                 0.0,       0.0,          0.0,        0.0,         0.0,          0.0,
//                                 0.0,       0.0,          0.0,        0.0,         0.0,          0.0;
//     init_position_ *= DEGREE2RADIAN;

//     loadKickManip();
//     updateKickingParam();

//     loadFeedbackParam();

//     Torso = Matrix<double, 6, 1>();
//     Torso.head<3>() = Vector3d(0, 0, Body_Height);
//     Torso.tail<3>() = Vector3d(0, Body_Tilt, 0);

//     // present angle
//     for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
//             state_iter != result_.end(); state_iter++)
//     {
//         std::string _joint_name = state_iter->first;
//         int joint_index = joint_table_[_joint_name];

//         robotis_framework::Dynamixel *dxl = NULL;
//         std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(_joint_name);
//         if (dxl_it != dxls.end())
//             dxl = dxl_it->second;
//         else
//             continue;

//         current_position_.coeffRef(0, joint_index) = dxl->dxl_state_->present_position_;
//         current_velocity_.coeffRef(0, joint_index) = dxl->dxl_state_->present_velocity_;
//         // std::cout<<joint_index<<" present position: "<<dxl->dxl_state_->present_position_<<std::endl;
//         // std::cout<<joint_index<<" present velocity: "<<dxl->dxl_state_->present_velocity_<<std::endl;
//         // std::cout<<joint_index<<" present torque: "<<dxl->dxl_state_->present_torque_<<std::endl;
//         uint data = dxl->dxl_state_->bulk_read_table_["hardware_error_status"];
//         if(data != 0)
//         ROS_ERROR("Device Error %s : %d", _joint_name, data);
//         double valle = dxl->dxl_state_->present_position_;
//         if(FKfromgoalPos == false)
//             op3_kd_->setJointPos(_joint_name, valle);
//     }

//     for(int idx = 0; idx < 12; idx++)
//     {
//         current_joint_pos(idx) = joint_axis_direction_(idx)*(current_position_.coeffRef(0, idx)-offset[idx]);
//         current_joint_vel(idx) = joint_axis_direction_(idx)*current_velocity_.coeffRef(0, idx);
//     }

//     // Save servo position & velocity to struct
//     so.r_ank_pitch.positionNow = current_joint_pos(4);
//     so.r_ank_roll.positionNow = current_joint_pos(5);
//     so.l_ank_pitch.positionNow = current_joint_pos(10);
//     so.l_ank_roll.positionNow = current_joint_pos(11);

//     so.r_ank_pitch.velocityNow = current_joint_vel(4);
//     so.r_ank_roll.velocityNow = current_joint_vel(5);
//     so.l_ank_pitch.velocityNow = current_joint_vel(10);
//     so.l_ank_roll.velocityNow = current_joint_vel(11);

//     forwardKinematic();

//     // if(FKfromgoalPos == false)
//     // {
//     //     wholeBodyCOM();
//     //     wholeBodyCOM_LSSP();
//     //     wholeBodyCOM_RSSP();
//     // }

//     if(firstExc)
//         kicking_state_ = KickingInitPose;

//     if(kicking_state_ == KickingInitPose)
//     {
//         if(firstExc)
//         {
//             int max_=0;
//             init_pose_count_ = 0;
//             m_time_start = 0;

//             L_X = Leg_X;        R_X = Leg_X;
//             L_Y = Leg_Y;        R_Y = -Leg_Y;
//             L_Z = Leg_Z;        R_Z = Leg_Z;
//             L_X_Kick = 0;       R_X_Kick = 0;
//             L_Y_Kick = 0;       R_Y_Kick = 0;
//             L_Z_Kick = 0;       R_Z_Kick = 0;

//             L_Leg << L_X, L_Y, L_Z, 0, 0, 0;//L_R*DEGREE2RADIAN, L_P*DEGREE2RADIAN, L_A*DEGREE2RADIAN;
//             R_Leg << R_X, R_Y, R_Z, 0, 0, 0;//R_R*DEGREE2RADIAN, R_P*DEGREE2RADIAN, R_A*DEGREE2RADIAN;

//             getJoints.head<12>() = inverse_legs(L_Leg, R_Leg, Torso);

//             for (int idx = 0; idx < 12; idx++)
//                 max_ = std::max((int)fabs(current_joint_pos(idx) - getJoints(idx)), max_);

//             double mov_time = (max_ / 30) < 0.5 ? 0.5 : (max_/30);
//             double smp_time = control_cycle_msec_ * 0.001;
//             int all_time_steps = int(mov_time / smp_time) + 1;
//             calc_joint_tra_.resize(all_time_steps, 12);

//             for (int idx = 0; idx < 12; idx++)
//                 calc_joint_tra_.block(0, idx, all_time_steps, 1) = robotis_framework::calcMinimumJerkTra(current_joint_pos(idx), 0.0, 0.0,
//                                                                                                          getJoints(idx), 0.0, 0.0,
//                                                                                                          smp_time, mov_time);
//         }
//         firstExc = false;

//         for (int idx = 1; idx < 12; idx++)
//             getJoints(idx) = calc_joint_tra_(init_pose_count_, idx);

//         init_pose_count_ += 1;

//         if(init_pose_count_ >= calc_joint_tra_.rows())
//         {
//             kicking_state_ = KickingReady;
//             init_LLegY = L_FOOT_WORLD(1);
//             init_RLegY = R_FOOT_WORLD(1);
//             firstTimeExc = ros::Time::now().toSec();
//         }
//     }
//     else if (kicking_state_ == KickingReady || kicking_state_ == KickingEnable)
//     {
//         timeExc = ros::Time::now().toSec() - firstTimeExc;
//         // if(printKickVar)
//         // {
//         //     std::cout<<"InitLLegY: "<<init_LLegY<<"\tInitRLegY: "<<init_RLegY<<std::endl;
//         //     if((L_FOOT_WORLD(1) - init_LLegY)<0)
//         //         std::cout<<"1_deltaLShift: "<<-(L_FOOT_WORLD(1) - init_LLegY);
//         //     else
//         //         std::cout<<"2_deltaLShift: "<<(L_FOOT_WORLD(1) - init_LLegY);
//         //     if((R_FOOT_WORLD(1) - init_RLegY)>0)
//         //         std::cout<<"\t1_deltaRShift: "<<-(R_FOOT_WORLD(1) - init_RLegY)<<std::endl;
//         //     else
//         //         std::cout<<"\t2_deltaRShift: "<<(R_FOOT_WORLD(1) - init_RLegY)<<std::endl;
//         // }
//         if(mbc.tuneGain == true)
//         {
//             std::cout<<"Tuning Gain, input set to zero"<<std::endl;
//             so.resetServoValues();
//             mbc.kalman.K = mbc.kalmanGain(&mbc.sys_d,&mbc.kalman);
//             mbc.dlqr.K = mbc.dlqrGain(&mbc.sys_d,&mbc.dlqr);  
//             // Calculate Fuzzy Gain here ... (cc: Gaby)
//             // mbc.dlqr.Kr = mbc.feedforwardGain(&mbc.sys_d, &mbc.dlqr);
//             mbc.dlqr.Kr = mbc.feedforwardGain(&mbc.sys_d, mbc.dlqr.K);
//             feedback(4) = 0; feedback(5) = 0;
//             feedback(10) = 0; feedback(11) = 0;
//         }
//         else
//         {
//             std::cout<<"Gain done, applied to system, get ready to engage"<<std::endl;
//             mbc.ymeas[0] = COMDSPrpy(1)-mbc.pitchRef*M_PI/180;//asin(DSP_COMrefX/COM_DSP_Length); //pitch
//             mbc.ymeas[1] = m_imuGyr(1);
//             if(mbc.testObserver == true)
//             {
//                 std::cout<<"Testing Observer Response, no feedback control effort applied"<<std::endl;
//                 if(densis.densisMode == true)
//                     densisInput();
//                 mbc.wholeBodyStateObserver(so.r_ank_pitch.refPosFromPos, mbc.wholeBodyStates, mbc.kalman.K, mbc.yest, mbc.ymeas);
//             }
//             else
//             {
//                 std::cout<<"Testing Controller Closed Loop Response, control effort applied!"<<std::endl;
//                 // for observers
//                 mbc.wholeBodyStateObserver(so.r_ank_pitch.refPosFromPos, mbc.wholeBodyStates, mbc.kalman.K, mbc.yest, mbc.ymeas);
//                 // for controller
//                 feedbackDSPAnklePitchPos2Pos();
//                 // so.r_ank_pitch.refPosFromPos = mbc.outputFeedback(mbc.pitchRef*M_PI/180,mbc.wholeBodyStates,mbc.dlqr.K,mbc.dlqr.Kr,balance_angle);
//             }
//         }

//         if(m_active == true)
//         {
//             kickStatus("kicking");

//             firstExc = false;

//             if(m_start)
//             {
//                 m_time_start = ros::Time::now().toSec();
//                 // c1 = boost::chrono::high_resolution_clock::now();
//                 //IF MEPET ? THEN KONDISI 1: ELSE KONDISI 2
//                 //IF kaki kiri mepet ke dalam (-) ? Lebarkan ke arah kiri (+): Pepetkan ke arah kanan (-)
//                 delta_L_Shift = ((L_FOOT_WORLD(1) - init_LLegY)<0)?-(L_FOOT_WORLD(1) - init_LLegY):(L_FOOT_WORLD(1) - init_LLegY);
//                  //IF kaki kanan mepet ke dalam (+) ? Lebarkan ke arah kanan (-): Pepetkan ke arah kiri (+)
//                 delta_R_Shift = ((R_FOOT_WORLD(1) - init_RLegY)>0)?-(R_FOOT_WORLD(1) - init_RLegY):(R_FOOT_WORLD(1) - init_RLegY);
//                 // delta_L_Shift = 0; delta_R_Shift = 0;
//             }
//             m_start = false;

//             if(printKickVar) std::cout<<"m_position"<<m_position<<std::endl;

//             m_delta_time = ros::Time::now().toSec() - m_time_start;
//             auto c2 = boost::chrono::high_resolution_clock::now();
//             auto deltaC = boost::chrono::duration_cast<boost::chrono::milliseconds>(c2-c1).count();
            
//             gyro_y = m_imuGyr.y();                                            //radian

//             movAvg.push_back(gyro_y);
//             for (int i = 0; i < movAvg.size(); i++) {
//             gyro_y_ += movAvg[i];
//             }

//             if(movAvg.size() >= samplingRate)
//             movAvg.erase(movAvg.begin());

//             gyro_y_ = EMAV2(movAvg, samplingRate);
            
//             if(densis.densisMode == true)
//             {
//                 densisInput();
//             }
//             else
//             {
//                 if(liftFoot == true) //ngangkat kaki, bisa gerak, bisa nahan
//                 {
//                     if(holdPose == true) //tahan ngangkat satu kaki
//                     {
//                         if(m_delta_time <= Time_Shift)
//                             shifting(Time_Shift, m_time_start);
//                         else if(m_delta_time <= (Time_Shift + Time_Roll))
//                             rolling(Time_Roll, m_time_start + Time_Shift);
//                         else if(m_delta_time <= (Time_Shift + Time_Roll + Time_Lift))
//                             lifting(Time_Lift, m_time_start + Time_Shift + Time_Roll);
//                         else
//                         {
//                             //Open Loop Steady State Response
//                             updateKickingParam();
//                             liftingHold();
//                             //With Feedback (Closed Loop)
//                             feedbackDecision();
//                         }
//                     }
//                     else // tendang normal
//                     {
//                         // double divC = deltaC;
//                         // std::cout<<"Delta Chronos: "<<divC<<std::endl;
//                         if(m_delta_time <= Time_Shift)
//                             shifting(Time_Shift, m_time_start);
//                         else if(m_delta_time <= (Time_Shift + Time_Roll))
//                             rolling(Time_Roll, m_time_start + Time_Shift);
//                         else if(m_delta_time <= (Time_Shift + Time_Roll + Time_Lift))
//                             lifting(Time_Lift, m_time_start + Time_Shift + Time_Roll);
//                         else if(m_delta_time <= (Time_Shift + Time_Roll + Time_Lift + Time_Retract))
//                         {
//                             retract(Time_Retract, m_time_start + Time_Shift + Time_Roll + Time_Lift);
//                             feedbackDecision();
//                         }
//                         else if(m_delta_time <= (Time_Shift + Time_Roll + Time_Lift + Time_Retract + Time_Kick))
//                         {
//                             kicking(Time_Kick, m_time_start + Time_Shift + Time_Roll + Time_Lift + Time_Retract);
//                             feedbackDecision();
//                         }
//                         else if(m_delta_time <= (Time_Shift + Time_Roll + Time_Lift + Time_Retract + Time_Kick + Time_CoolDown))
//                         {
//                             coolingDown(Time_CoolDown, m_time_start + Time_Shift + Time_Roll + Time_Lift + Time_Retract + Time_Kick);
//                             feedbackDecision();
//                         }
//                         else if(m_delta_time <= (Time_Shift + Time_Roll + Time_Lift + Time_Retract + Time_Kick + Time_CoolDown + Time_BringDown))
//                         {
//                             bringDown(Time_BringDown, m_time_start + Time_Shift + Time_Roll + Time_Lift + Time_Retract + Time_Kick + Time_CoolDown);
//                             feedbackDSPAnklePitchPos2Pos();
//                         }
//                         else
//                         {
//                             // m_time_start = 0;
//                             // DSPHold();
//                             kickStatus("finished_kick");
//                             // stop();
//                         }
//                     }

//                 }
//                 else //not liftFoot, nggak ngangkat kaki
//                 {
//                     DSPHold();
//                     // if(feedbackActive)
//                     // {
//                     //      // wholeBodyStateObserver(so.r_ank_pitch.refPosFromPos, wholeBodyStates, ymeas,gyroThreshold);
//                     //     feedbackDSPAnklePitchPos2Pos();
//                     // }
//                     // else
//                     // {
//                     //     feedback(4) = 0; feedback(5) = 0;
//                     //     feedback(10) = 0; feedback(11) = 0;
//                     // }
//                 }
//             }
//         }
//         else
//         {
//             DSPHold();
//             // if(feedbackActive)
//             // {
//             //     //  wholeBodyStateObserver(so.r_ank_pitch.refPosFromPos, wholeBodyStates, ymeas,gyroThreshold);
//             //     feedbackDSPAnklePitchPos2Pos();
//             // }
//             // else
//             // {
//             //     feedback(4) = 0; feedback(5) = 0;
//             //     feedback(10) = 0; feedback(11) = 0;
//             // }
//         }

//         if(m_position == Right_Kick)
//         {
//             L_X = L_X_Kick + Leg_R_X;      R_X = R_X_Kick + Leg_R_X;
//             L_Y = L_Y_Kick + Leg_Y;        R_Y = R_Y_Kick - Leg_Y;
//             L_Z = L_Z_Kick + Leg_Z;        R_Z = R_Z_Kick + Leg_Z;
//         }
//         else if(m_position == Right_Side_Kick)
//         {
//             L_X = L_X_Kick + Leg_R_X;      R_X = R_X_Kick + Leg_R_X;
//             L_Y = L_Y_Kick + Leg_Y;        R_Y = R_Y_Kick - Leg_Y;
//             L_Z = L_Z_Kick + Leg_Z;        R_Z = R_Z_Kick + Leg_Z;
//         }
//         else
//         {
//             L_X = L_X_Kick + Leg_L_X;      R_X = R_X_Kick + Leg_L_X;
//             L_Y = L_Y_Kick + Leg_Y;        R_Y = R_Y_Kick - Leg_Y;
//             L_Z = L_Z_Kick + Leg_Z;        R_Z = R_Z_Kick + Leg_Z;
//         }

//         //Limitter so Kicking Foot doesn't Hit Support Foot
//         if(m_position == Right_Kick)
//         {
//             if(R_Y > L_Y-2.5*k.LEG_SIDE_OFFSET)
//                 R_Y = L_Y-2.5*k.LEG_SIDE_OFFSET;
//         }
//         else if(m_position == Left_Kick)
//         {
//             if(L_Y < R_Y+2.5*k.LEG_SIDE_OFFSET)
//                 L_Y = R_Y+2.5*k.LEG_SIDE_OFFSET;
//         }
//         else
//         {
//             R_Y = R_Y; L_Y = L_Y;
//         }

//         //Prevent Bruce Lee Kick
//         if((fabs(R_Y_Kick)>0.16 || fabs(L_Y_Kick)>0.16) && m_position != 0)
//         {
//             delta_L_Shift = 0; delta_R_Shift = 0;
//             if(m_position == Right_Kick)
//             {
//                 L_X = Leg_R_X;      R_X = R_X_Kick + Leg_R_X;
//                 L_Y = (R_Shift + delta_L_Shift) + Leg_Y;                R_Y = (R_Shift + delta_R_Shift) - Leg_Y;
//                 L_Z =  Leg_Z;       R_Z =  0.1+Leg_Z;
//             }
//             else if(m_position == Left_Kick)
//             {
//                 L_X = L_X_Kick + Leg_L_X;      R_X = R_X_Kick + Leg_L_X;
//                 L_Y = (L_Shift + delta_L_Shift) + Leg_Y;                R_Y = (L_Shift + delta_R_Shift) - Leg_Y;
//                 L_Z =  0.1+Leg_Z;       R_Z =  Leg_Z;
//             }
//             else if(m_position == Right_Side_Kick)
//             {
//                 L_X = L_X_Kick + Leg_R_X;      R_X = R_X_Kick + Leg_R_X;
//                 L_Y = percentRSK*(R_Shift + delta_L_Shift) + Leg_Y;                R_Y = percentRSK*(R_Shift + delta_R_Shift) - Leg_Y;
//                 L_Z =  Leg_Z;       R_Z = 0.1+Leg_Z;
//             }
//             else
//             {
//                 L_X = L_X_Kick  + Leg_L_X;      R_X = R_X_Kick  + Leg_L_X;
//                 L_Y = percentLSK*(L_Shift + delta_L_Shift) + Leg_Y;                R_Y = percentLSK*(L_Shift + delta_R_Shift) - Leg_Y;
//                 L_Z =  0.1+Leg_Z;       R_Z =  Leg_Z;
//             }
//             std::cout<<"HUWWWWAAAAAAAA WADAAAAAAAAAAA"<<std::endl;
//         }

//         if(printKickVar)
//         {
//             std::cout<<"->^|Feedback Output|v->"<<std::endl;
//             std::cout<<"LAnklePitch: "<<feedback(10)<<"\tRAnklePitch: "<<feedback(4)<<std::endl;
//             std::cout<<"LAnkleRoll: "<<feedback(11)<<"\tRAnkleRoll: "<<feedback(5)<<std::endl;
//         }

//         L_Leg << L_X, L_Y, L_Z, 0, 0, 0;//L_R*DEGREE2RADIAN, L_P*DEGREE2RADIAN, L_A*DEGREE2RADIAN;
//         R_Leg << R_X, R_Y, R_Z, 0, 0, 0;//R_R*DEGREE2RADIAN, R_P*DEGREE2RADIAN, R_A*DEGREE2RADIAN;
//         getJoints.head<12>() = inverse_legs(L_Leg, R_Leg, Torso);
//         motion_arms();
//         // Limit Angle to Prevent Damage to Servos
//         if(getJoints(4) <= -60*DEGREE2RADIAN)
//             getJoints(4)=-60*DEGREE2RADIAN; //Right_Ankle Pitch
//         else if(getJoints(4) > 20*DEGREE2RADIAN)
//             getJoints(4)=20*DEGREE2RADIAN; //Right_Ankle Pitch
//         else if(getJoints(10) <= -60*DEGREE2RADIAN)
//             getJoints(10)=-60*DEGREE2RADIAN; //Left Ankle Pitch
//         else if(getJoints(10) > 20*DEGREE2RADIAN)
//             getJoints(10)=20*DEGREE2RADIAN; //Left Ankle Pitch
//        if(printKickVar) printValue();
//     }

//     so.r_ank_pitch.positionIK = getJoints(4);
//     so.r_ank_roll.positionIK = getJoints(5);
//     so.l_ank_pitch.positionIK = getJoints(10);
//     so.l_ank_roll.positionIK = getJoints(11);

//     setValue();

//     if(FKfromgoalPos == true)
//     {
//         wholeBodyCOM();
//         wholeBodyCOM_LSSP();
//         wholeBodyCOM_RSSP();
//         wholeBodyCOM_DSP();
//         globalInertia();
//         globalInertiaLSSP();
//         globalInertiaRSSP();
//         globalInertiaDSP();
//         calculateZMP();
//     }
//     auto t2 = boost::chrono::high_resolution_clock::now();
//     auto elapsed_time_2 = boost::chrono::duration_cast<boost::chrono::milliseconds>(t2-t1).count();
//     std::cout << "Elapsed Process Time : " << elapsed_time_2 << std::endl;
//     dataLogPublish();
//     densisPublish();
//     auto t3 = boost::chrono::high_resolution_clock::now();
//     auto elapsed_time_3 = boost::chrono::duration_cast<boost::chrono::milliseconds>(t3-t1).count();
//     std::cout << "Elapsed Process + Publish Time : " << elapsed_time_3 << std::endl;
// }

// void Kicking::printValue()
// {
//     std::cout<<"IMU_Roll(deg): "<<m_imuOri.x()*180/M_PI<<"\tIMU_Pitch(deg): "<<m_imuOri.y()*180/M_PI<<"\tIMU_Yaw(deg): "<<m_imuOri.z()*180/M_PI<<std::endl;
//     std::cout<<"gyro_Roll(deg/s): "<<m_imuGyr.x()*180/M_PI<<"\tgyro_Pitch(deg/s): "<<m_imuGyr.y()*180/M_PI<<"\tgyro_Yaw(deg/s): "<<m_imuGyr.z()*180/M_PI<<std::endl;
//     std::cout<<"InverseKinematic:"<<"\t"<<"ForwardKinematic:"<<std::endl;
//     // std::cout<<"L_X: "<<L_X<<"\tR_X: "<<R_X<<"\t||\tL_Y: "<<L_Y<<"\tR_Y: "<<R_Y<<"\t||\tL_Z: "<<L_Z<<"\tR_Z: "<<R_Z<<std::endl;
//     std::cout<<"ikL_X: "<<L_X<<"\tikR_X: "<<R_X<<"\tfkL_X: "<<L_FOOT_WORLD(0)<<"\tfkR_X: "<<R_FOOT_WORLD(0)<<std::endl;
//     std::cout<<"ikL_Y: "<<L_Y<<"\tikR_Y: "<<R_Y<<"\tfkL_Y: "<<L_FOOT_WORLD(1)<<"\tfkR_Y: "<<R_FOOT_WORLD(1)<<std::endl;
//     std::cout<<"ikL_Z: "<<L_Z<<"\tikR_Z: "<<R_Z<<"\tfkL_Z: "<<L_FOOT_WORLD(2)<<"\tfkR_Z: "<<R_FOOT_WORLD(2)<<std::endl;
//     std::cout<<6<<"\tIK: "<<getJoints(6)*180/M_PI<<"\t"<<0<<"\tIK: "<<getJoints(0)*180/M_PI<<"\t"<<6<<"\tFK: "<<current_joint_pos(6)*180/M_PI<<"\t"<<0<<"\tFK: "<<current_joint_pos(0)*180/M_PI<<std::endl;
//     std::cout<<7<<"\tIK: "<<getJoints(7)*180/M_PI<<"\t"<<1<<"\tIK: "<<getJoints(1)*180/M_PI<<"\t"<<7<<"\tFK: "<<current_joint_pos(7)*180/M_PI<<"\t"<<1<<"\tFK: "<<current_joint_pos(1)*180/M_PI<<std::endl;
//     std::cout<<8<<"\tIK: "<<getJoints(8)*180/M_PI<<"\t"<<2<<"\tIK: "<<getJoints(2)*180/M_PI<<"\t"<<8<<"\tFK: "<<current_joint_pos(8)*180/M_PI<<"\t"<<2<<"\tFK: "<<current_joint_pos(2)*180/M_PI<<std::endl;
//     std::cout<<9<<"\tIK: "<<getJoints(9)*180/M_PI<<"\t"<<3<<"\tIK: "<<getJoints(3)*180/M_PI<<"\t"<<9<<"\tFK: "<<current_joint_pos(9)*180/M_PI<<"\t"<<3<<"\tFK: "<<current_joint_pos(3)*180/M_PI<<std::endl;
//     std::cout<<10<<"\tIK: "<<getJoints(10)*180/M_PI<<"\t"<<4<<"\tIK: "<<getJoints(4)*180/M_PI<<"\t"<<10<<"\tFK: "<<current_joint_pos(10)*180/M_PI<<"\t"<<4<<"\tFK: "<<current_joint_pos(4)*180/M_PI<<std::endl;
//     std::cout<<11<<"\tIK: "<<getJoints(11)*180/M_PI<<"\t"<<5<<"\tIK: "<<getJoints(5)*180/M_PI<<"\t"<<11<<"\tFK: "<<current_joint_pos(11)*180/M_PI<<"\t"<<5<<"\tFK: "<<current_joint_pos(5)*180/M_PI<<std::endl;
// }

// void Kicking::shifting(double duration, double totalPrevious)
// {
//     double delta_time = ros::Time::now().toSec() - totalPrevious;
//     // std::cout<<"Persen Shifting : "<<delta_time/Time_Shift<<std::endl;

//     if(m_position == Right_Kick)
//     {
//         L_X_Kick = 0;                                                       R_X_Kick = 0;
//         L_Y_Kick = (R_Shift + delta_L_Shift) * sigmoid(2.7,10,0.4,delta_time/duration);// * delta_time / Time_Shift;
//         R_Y_Kick = (R_Shift + delta_R_Shift) * sigmoid(2.7,10,0.4,delta_time/duration);// * delta_time / Time_Shift;
//         L_Z_Kick = 0;                                                       R_Z_Kick = 0;
//     }
//     else if(m_position == Left_Kick)
//     {
//         L_X_Kick = 0;                                                       R_X_Kick = 0;
//         L_Y_Kick = (L_Shift + delta_L_Shift) *sigmoid(2.7,10,0.4,delta_time/duration); // * delta_time / Time_Shift;
//         R_Y_Kick = (L_Shift + delta_R_Shift) *sigmoid(2.7,10,0.4,delta_time/duration); // * delta_time / Time_Shift;
//         L_Z_Kick = 0;                                                       R_Z_Kick = 0;
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         L_X_Kick = 0;                                                       R_X_Kick = 0;
//         L_Y_Kick = (percentRSK*R_Shift + delta_L_Shift) *sigmoid(2.7,10,0.4,delta_time/duration); // * delta_time / Time_Shift;
//         R_Y_Kick = (percentRSK*R_Shift + delta_R_Shift) *sigmoid(2.7,10,0.4,delta_time/duration); // * delta_time / Time_Shift;
//         L_Z_Kick = 0;                                                       R_Z_Kick = 0;
//     }
//     else
//     {
//         L_X_Kick = 0;                                                       R_X_Kick = 0;
//         L_Y_Kick = (percentLSK*L_Shift + delta_L_Shift) *sigmoid(2.7,10,0.4,delta_time/duration); // * delta_time / Time_Shift;
//         R_Y_Kick = (percentLSK*L_Shift + delta_R_Shift) *sigmoid(2.7,10,0.4,delta_time/duration); // * delta_time / Time_Shift;
//         L_Z_Kick = 0;                                                       R_Z_Kick = 0;
//     }

//     Torso.tail<3>() = Vector3d(0, Body_Tilt, 0);
// }

// void Kicking::shiftingHold()
// {
//     if(m_position == Right_Kick)
//     {
//         L_X_Kick = 0;                             R_X_Kick = 0;
//         L_Y_Kick = (R_Shift + delta_L_Shift);     R_Y_Kick = (R_Shift + delta_R_Shift);
//         L_Z_Kick = 0;                             R_Z_Kick = 0;
//     }
//     else if(m_position == Left_Kick)
//     {
//         L_X_Kick = 0;                             R_X_Kick = 0;
//         L_Y_Kick = (L_Shift + delta_L_Shift);     R_Y_Kick = (L_Shift + delta_R_Shift);
//         L_Z_Kick = 0;                             R_Z_Kick = 0;
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         L_X_Kick = 0;                                       R_X_Kick = 0;
//         L_Y_Kick = (percentRSK*R_Shift + delta_L_Shift);    R_Y_Kick = (percentRSK*R_Shift + delta_R_Shift);
//         L_Z_Kick = 0;                                       R_Z_Kick = 0;
//     }
//     else
//     {
//         L_X_Kick = 0;                                        R_X_Kick = 0;
//         L_Y_Kick = (percentLSK*L_Shift + delta_L_Shift);     R_Y_Kick = (percentLSK*L_Shift + delta_R_Shift);
//         L_Z_Kick = 0;                                        R_Z_Kick = 0;
//     }

//     Torso.tail<3>() = Vector3d(0, Body_Tilt, 0);
// }

// void Kicking::rolling(double duration, double totalPrevious)
// {
//     double delta_time = ros::Time::now().toSec() - totalPrevious;
//     // std::cout<<"Persen Rolling : "<<delta_time/Time_Roll<<std::endl;

//     if(m_position == Right_Kick)
//     {
//         L_X_Kick = 0;     R_X_Kick = 0;
//         L_Y_Kick = (R_Shift + delta_L_Shift);
//         R_Y_Kick = (R_Shift + delta_R_Shift);
//         L_Z_Kick = 0;     R_Z_Kick = 0;//R_Z_Lift * delta_time / Time_Lift;
//         //Override L_HIP
//         feedback(7) = -getJoints(7)*delta_time/duration;
//         //Override L_ANKLE
//         rollAnkle(1) = R_Roll * PI/180 * 10 * delta_time/duration;
//         // feedback(11) = feedbackRoll;
//         // feedback(10) = feedbackPitch;
//     }
//     else if(m_position == Left_Kick)
//     {
//         L_X_Kick = 0;        R_X_Kick = 0;
//         L_Y_Kick = (L_Shift + delta_L_Shift);
//         R_Y_Kick = (L_Shift + delta_R_Shift);
//         L_Z_Kick = 0;//L_Z_Lift * delta_time / Time_Lift;
//         R_Z_Kick = 0;
//         //Override R_HIP
//         feedback(1) = -getJoints(1)*delta_time/duration;
//         //Override R_ANKLE
//         rollAnkle(0) = L_Roll * PI/180 * 10 * delta_time/duration;
//         // feedback(5) = feedbackRoll;
//         // feedback(4) = feedbackPitch;
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         L_X_Kick = 0; R_X_Kick = 0;
//         L_Y_Kick = (R_Shift + delta_L_Shift);
//         R_Y_Kick = (R_Shift + delta_R_Shift);
//         L_Z_Kick = 0; R_Z_Kick = 0;//R_Z_Lift * delta_time / Time_Lift;
//         //Override L_HIP
//         feedback(7) = -getJoints(7)*delta_time/duration;
//         //Override L_ANKLE_ROLL
//         rollAnkle(1) = R_Roll * PI/180 * 10 * delta_time/duration;
//         // feedback(11) = feedbackRoll;
//         // feedback(10) = feedbackPitch;
//     }
//     else
//     {
//         L_X_Kick = 0;  R_X_Kick = 0;
//         L_Y_Kick = (L_Shift + delta_L_Shift);
//         R_Y_Kick = (L_Shift + delta_R_Shift);
//         L_Z_Kick = 0;
//         R_Z_Kick = 0;
//         //Override R_HIP
//         feedback(1) = -getJoints(1)*delta_time/duration;
//         //Override R_ANKLE_ROLL
//         rollAnkle(0) = L_Roll * PI/180 * 10 * delta_time/duration;
//         // feedback(5) = feedbackRoll;
//         // feedback(4) = feedbackPitch;
//     }
//     Torso.tail<3>() = Vector3d(0, Body_Tilt, 0);
// }

// void Kicking::lifting(double duration, double totalPrevious)
// {
//     double delta_time = ros::Time::now().toSec() - totalPrevious;

//     if(m_position == Right_Kick)
//     {
//         L_X_Kick = 0;     R_X_Kick = 0;
//         L_Y_Kick = (R_Shift + delta_L_Shift) + R_Lift*sigmoid(2.7,10,0.4,delta_time/duration);
//         R_Y_Kick = (R_Shift + delta_R_Shift);
//         L_Z_Kick = 0;     R_Z_Kick = R_Z_Lift*sigmoid(2.7,10,0.4,delta_time/duration);
//         //Override L_HIP
//         feedback(7) = -getJoints(7);
//         //Override L_ANKLE
//         rollAnkle(1) = R_Roll * PI/180 * 10;
//     }
//     else if(m_position == Left_Kick)
//     {
//         L_X_Kick = 0;     R_X_Kick = 0;
//         L_Y_Kick = (L_Shift + delta_L_Shift);
//         R_Y_Kick = (L_Shift + delta_R_Shift) + L_Lift*sigmoid(2.7,10,0.4,delta_time/duration);
//         L_Z_Kick = L_Z_Lift*sigmoid(2.7,10,0.4,delta_time/duration);
//         R_Z_Kick = 0;
//         //Override_R_HIP
//         feedback(1) = -getJoints(1);
//         //Override R_ANKLE
//         rollAnkle(0) = L_Roll * PI/180 * 10;
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         L_X_Kick = 0;     R_X_Kick = 0;
//         L_Y_Kick = (R_Shift + delta_L_Shift) + R_Lift*sigmoid(2.7,10,0.4,delta_time/duration);
//         R_Y_Kick = (R_Shift + delta_R_Shift);
//         L_Z_Kick = 0;     R_Z_Kick = R_Z_Lift*sigmoid(2.7,10,0.4,delta_time/duration);
//         //Override L_HIP
//         feedback(7) = -getJoints(7);
//         //Override L_ANKLE
//         rollAnkle(1) = R_Roll * PI/180 * 10;
//     }
//     else
//     {
//         L_X_Kick = 0;           R_X_Kick = 0;
//         L_Y_Kick = (L_Shift + delta_L_Shift);
//         R_Y_Kick = (L_Shift + delta_R_Shift) + L_Lift*sigmoid(2.7,10,0.4,delta_time/duration);
//         L_Z_Kick = L_Z_Lift*sigmoid(2.7,10,0.4,delta_time/duration);
//         R_Z_Kick = 0;
//         //Override R_HIP
//         feedback(1) = -getJoints(1);
//         //Override R_ANKLE
//         rollAnkle(0) = L_Roll * PI/180 * 10;
//     }

//     Torso.tail<3>() = Vector3d(0, Body_Tilt, 0);
// }

// void Kicking::liftingHold()
// {
//     if(m_position == Right_Kick)
//     {
//         L_X_Kick = 0;     R_X_Kick = 0;
//         L_Y_Kick = (R_Shift + delta_L_Shift) + R_Lift;
//         R_Y_Kick = (R_Shift + delta_R_Shift);
//         L_Z_Kick = 0;     R_Z_Kick = R_Z_Lift;
//         //Override L_HIP
//         feedback(7) = -getJoints(7);
//         //Override L_ANKLE
//         rollAnkle(1) = R_Roll * PI/180 * 10;
//     }
//     else if(m_position == Left_Kick)
//     {
//         L_X_Kick = 0;     R_X_Kick = 0;
//         L_Y_Kick = (L_Shift + delta_L_Shift);
//         R_Y_Kick = (L_Shift + delta_R_Shift) + L_Lift;
//         L_Z_Kick = L_Z_Lift;     R_Z_Kick = 0;
//         //Override_R_HIP
//         feedback(1) = -getJoints(1);
//         //Override R_ANKLE
//         rollAnkle(0) = L_Roll * PI/180 * 10;
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         L_X_Kick = 0;     R_X_Kick = 0;
//         L_Y_Kick = (R_Shift + delta_L_Shift) + R_Lift;
//         R_Y_Kick = (R_Shift + delta_R_Shift);
//         L_Z_Kick = 0;     R_Z_Kick = R_Z_Lift;
//         //Override L_HIP
//         feedback(7) = -getJoints(7);
//         //Override L_ANKLE
//         rollAnkle(1) = R_Roll * PI/180 * 10;
//     }
//     else
//     {
//         L_X_Kick = 0;           R_X_Kick = 0;
//         L_Y_Kick = (L_Shift + delta_L_Shift);
//         R_Y_Kick = (L_Shift + delta_R_Shift) + L_Lift;
//         L_Z_Kick = L_Z_Lift;     R_Z_Kick = 0;
//         //Override R_HIP
//         feedback(1) = -getJoints(1);
//         //Override R_ANKLE
//         rollAnkle(0) = L_Roll * PI/180 * 10;
//     }

//     Torso.tail<3>() = Vector3d(0, Body_Tilt, 0);
// }

// void Kicking::DSPHold()
// {
//     L_X_Kick = 0; R_X_Kick = 0;
//     L_Y_Kick = 0; R_Y_Kick = 0;
//     L_Z_Kick = 0; R_Z_Kick = 0;
// }

// double Kicking::sigmoid(double C, double G, double H, double x)
// {
//    return (exp(G*(x-H))/(exp(G*(x-H))+C));
// }

// void Kicking::retract(double duration, double totalPrevious)
// {
//     Eigen::MatrixXf PS(1,4), CM(4,4), CP(4,3), Output(1,3);

//     //Control Matrix (jangan diubah)
//     CM << 1,  0,  0,  0,
//          -3,  3,  0,  0,
//           3, -6,  3,  0,
//          -1,  3, -3,  1;
//     double delta_time = ros::Time::now().toSec() - totalPrevious;
//     double tr = delta_time / duration;

//     // std::cout<<"Persen Retract : "<<delta_time/Time_Retract<<std::endl;

//     // Power Series (jangan diubah)
//     PS << 1, tr, pow(tr,2), pow(tr,3);
//     if(m_position == Right_Kick || Left_Kick)
//     {
//         // Control Point Matrix (boleh diubah)
//         CP  <<  0,                 0,   0.09, //Titik Awal Retract
//                 -0.4*X_Retract,    0,   0.4*(Z_Retract-0.09)+0.09,
//                 -0.75*X_Retract,   0,   0.75*(Z_Retract-0.09)+0.09,
//                 -X_Retract,        0,   Z_Retract; //Titik Akhir Retract
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         CP  <<  0.0*(ballPosX),    0,         ballPosZ,
//                 0.2*(ballPosX),    -0.02,     ballPosZ,
//                 0.8*(ballPosX),    -0.04,     ballPosZ,
//                 ballPosX,          -0.05,     ballPosZ;
//     }
//     else
//     {
//         CP  <<  0.0*(ballPosX),    0,       ballPosZ,
//                 0.2*(ballPosX),    0.02,    ballPosZ,
//                 0.8*(ballPosX),    0.04,    ballPosZ,
//                 ballPosX,          0.05,    ballPosZ;
//     }

//     Torso.tail<3>() = Vector3d(0, Body_Tilt, 0);
//     Output = PS * CM * CP;
//     if(m_position == Right_Kick)
//     {
//         L_X_Kick = 0;                     R_X_Kick = Output(0,0);
//         L_Y_Kick = (R_Shift) + R_Lift;    R_Y_Kick = (R_Shift) + Output(0,1);
//         L_Z_Kick = 0;                     R_Z_Kick = Output(0,2);
//         //Override L_HIP
//         feedback(7) = -getJoints(7);
//         //Override L_ANKLE_ROLL
//         // feedback(11)=feedbackRoll;
//         // feedback(10) = feedbackPitch;
//         rollAnkle(1) = R_Roll * PI/180 * 10;
//     }
//     else if(m_position == Left_Kick)
//     {
//         L_X_Kick = Output(0,0);                R_X_Kick = 0;
//         L_Y_Kick = (L_Shift) + Output(0,1);    R_Y_Kick = (L_Shift) + L_Lift;
//         L_Z_Kick = Output(0,2);                R_Z_Kick = 0;
//         //Override_R_HIP
//         feedback(1) = -getJoints(1);
//         //Override R_ANKLE_ROLL
//         // feedback(5)=feedbackRoll;
//         // feedback(4) = feedbackPitch;
//         rollAnkle(0) = L_Roll * PI/180 * 10;
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         L_X_Kick = L_X_Shift;           R_X_Kick = Output(0,0);
//         L_Y_Kick = (percentRSK*R_Shift);
//         R_Y_Kick = (percentRSK*R_Shift) + Output(0,1);
//         L_Z_Kick = 0;                   R_Z_Kick = Output(0,2);
//         //Override L_HIP
//         feedback(7) = -getJoints(7);
//         //Override L_ANKLE_ROLL
//         // feedback(11)=feedbackRoll;
//         // feedback(10) = feedbackPitch;
//         rollAnkle(1) = R_Roll * PI/180 * 10;
//     }
//     else
//     {
//         L_X_Kick = Output(0,0);         R_X_Kick = R_X_Shift;
//         L_Y_Kick = (percentLSK*L_Shift) + Output(0,1);
//         R_Y_Kick = (percentLSK*L_Shift);
//         L_Z_Kick = Output(0,2);         R_Z_Kick = 0;
//         //Override_R_HIP
//         feedback(1) = -getJoints(1);
//         //Override R_ANKLE_ROLL
//         // feedback(5)=feedbackRoll;
//         // feedback(4) = feedbackPitch;
//         rollAnkle(0) = L_Roll * PI/180 * 10;
//     }
// }

// void Kicking::kicking(double duration, double totalPrevious)
// {
//     Eigen::MatrixXf PS(1,4), CM(4,4), CP(4,3), Output(1,3);
//     //Control Matrix (jangan diubah)
//     CM << 1,  0,  0,  0,
//          -3,  3,  0,  0,
//           3, -6,  3,  0,
//          -1,  3, -3,  1;
//     double delta_time = ros::Time::now().toSec() - totalPrevious;
//     double tr = delta_time / duration;
//     // std::cout<<"Persen Kick : "<<delta_time/Time_Kick<<std::endl;

//     // Power Series (jangan diubah)
//     PS << 1, tr, pow(tr,2), pow(tr,3);
//     if(m_position == Right_Kick || m_position == Left_Kick)
//     {
//         // Control Point Matrix (boleh diubah)
//         CP  <<  0.0*(ballPosX+X_Retract)-X_Retract,    0,    Z_Retract,
//                 0.0*(ballPosX+X_Retract)-X_Retract,    0,    0.7*(ballPosZ-Z_Retract)+Z_Retract,
//                 1.0*(ballPosX+X_Retract)-X_Retract,    0,    0.45*(ballPosZ-Z_Retract)+Z_Retract,
//                 ballPosX,                              0,    ballPosZ;
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         // Control Point Matrix (boleh diubah)
//         CP  <<  ballPosX,    -0.05,     ballPosZ,
//                 ballPosX,    0.1,    ballPosZ,
//                 ballPosX,    0.15,    ballPosZ,
//                 ballPosX,    0.15,    ballPosZ;
//     }
//     else
//     {
//         // Control Point Matrix (boleh diubah)
//         CP  <<  ballPosX,    0.05,    ballPosZ,
//                 ballPosX,    -0.1,    ballPosZ,
//                 ballPosX,    -0.15,    ballPosZ,
//                 ballPosX,    -0.15,    ballPosZ;
//     }

//     Torso.tail<3>() = Vector3d(0, Body_Tilt, 0);
//     Output = PS * CM * CP;
//     if(m_position == Right_Kick)
//     {
//         L_X_Kick = 0;                       R_X_Kick = Output(0,0);
//         L_Y_Kick = (R_Shift) + R_Lift;      R_Y_Kick = (R_Shift) + Output(0,1);
//         L_Z_Kick = 0;                       R_Z_Kick = Output(0,2);
//         //Override L_HIP
//         feedback(7) = -getJoints(7);
//         //Override L_ANKLE_ROLL
//         // feedback(11)=feedbackRoll;
//         // feedback(10) = feedbackPitch;
//         rollAnkle(1) = R_Roll * PI/180 * 10;
//     }
//     else if(m_position == Left_Kick)
//     {
//         L_X_Kick = Output(0,0);                  R_X_Kick = 0;
//         L_Y_Kick = (L_Shift) + Output(0,1);      R_Y_Kick = (L_Shift) + L_Lift;
//         L_Z_Kick = Output(0,2);                  R_Z_Kick = 0;
//         //Override_R_HIP
//         feedback(1) = -getJoints(1);
//         //Override R_ANKLE_ROLL
//         // feedback(5)=feedbackRoll;
//         // feedback(4) = feedbackPitch;
//         rollAnkle(0) = L_Roll * PI/180 * 10;
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         L_X_Kick = L_X_Shift;           R_X_Kick = Output(0,0);
//         L_Y_Kick = (percentRSK*R_Shift);
//         R_Y_Kick = (percentRSK*R_Shift) + Output(0,1);
//         L_Z_Kick = 0;                   R_Z_Kick = Output(0,2);
//         //Override L_HIP
//         feedback(7) = -getJoints(7);
//         //Override L_ANKLE_ROLL
//         // feedback(11)=feedbackRoll;
//         // feedback(10) = feedbackPitch;
//         rollAnkle(1) = R_Roll * PI/180 * 10;
//     }
//     else
//     {
//         L_X_Kick = Output(0,0);         R_X_Kick = R_X_Shift;
//         L_Y_Kick = (percentLSK*L_Shift) + Output(0,1);
//         R_Y_Kick = (percentLSK*L_Shift);
//         L_Z_Kick = Output(0,2);         R_Z_Kick = 0;
//         //Override_R_HIP
//         feedback(1) = -getJoints(1);
//         //Override R_ANKLE_ROLL
//         // feedback(5)=feedbackRoll;
//         // feedback(4) = feedbackPitch;
//         rollAnkle(0) = L_Roll * PI/180 * 10;
//     }
// }

// void Kicking::coolingDown(double duration, double totalPrevious)
// {
//     Eigen::MatrixXf PS(1,4), CM(4,4), CP(4,3), Output(1,3);
//     //Control Matrix (jangan diubah)
//     CM << 1,  0,  0,  0,
//          -3,  3,  0,  0,
//           3, -6,  3,  0,
//          -1,  3, -3,  1;
//     double delta_time = ros::Time::now().toSec() - totalPrevious;
//     double tr = delta_time / duration;
//     // std::cout<<"Persen CoolDown : "<<delta_time/Time_CoolDown<<std::endl;

//     // Power Series (jangan diubah)
//     PS << 1, tr, pow(tr,2), pow(tr,3);
//     if(m_position == Right_Kick || m_position == Left_Kick)
//     {
//         // Control Point Matrix (boleh diubah)
//         CP  <<  ballPosX,        0,    ballPosZ,
//                 0.5*ballPosX,    0,    ballPosZ,
//                 0.2*ballPosX,    0,    ballPosZ,
//                 0,               0,    0.5*ballPosZ;
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         // Control Point Matrix (boleh diubah)
//         CP  <<  ballPosX,        0.1,    ballPosZ,
//                 0.5*ballPosX,    0,    ballPosZ,
//                 0.2*ballPosX,    0,    ballPosZ,
//                 0,               0,    0.5*ballPosZ;
//     }
//     else
//     {
//         // Control Point Matrix (boleh diubah)
//         CP  <<  ballPosX,     -0.1,    ballPosZ,
//                 0.5*ballPosX,    0,    ballPosZ,
//                 0.2*ballPosX,    0,    ballPosZ,
//                 0,               0,    0.5*ballPosZ;
//     }

//     Torso.tail<3>() = Vector3d(0, Body_Tilt, 0);
//     R_CoolDown = kicking_param_.R_CoolDown;
//     L_CoolDown = kicking_param_.L_CoolDown;
//     Output = PS * CM * CP;
//     if(m_position == Right_Kick)
//     {
//         L_X_Kick = 0;                                  R_X_Kick = Output(0,0);
//         L_Y_Kick = (R_Shift) + R_Lift + R_CoolDown;    R_Y_Kick = (R_Shift) + Output(0,1)-R_CoolDown;
//         L_Z_Kick = 0;                                  R_Z_Kick = Output(0,2);
//         //Override L_HIP
//         feedback(7) = -getJoints(7);
//         //Override L_ANKLE_ROLL
//         // feedback(11)=feedbackRoll;
//         // feedback(10) = feedbackPitch;
//         rollAnkle(1) = R_Roll * PI/180 * 10 * (1-delta_time/duration);
//     }
//     else if(m_position == Left_Kick)
//     {
//         L_X_Kick = Output(0,0);                           R_X_Kick = 0;
//         L_Y_Kick = (L_Shift) + Output(0,1)-L_CoolDown;      R_Y_Kick = (L_Shift) + L_Lift + L_CoolDown;
//         L_Z_Kick = Output(0,2);                           R_Z_Kick = 0;
//         //Override R_HIP
//         feedback(1) = -getJoints(1);
//         //Override R_ANKLE_ROLL
//         // feedback(5)=feedbackRoll;
//         // feedback(4) = feedbackPitch;
//         rollAnkle(0) = L_Roll * PI/180 * 10 * (1-delta_time/duration);
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         L_X_Kick = L_X_Shift;                                  R_X_Kick = Output(0,0);
//         L_Y_Kick = (percentRSK*R_Shift) + R_CoolDown;
//         R_Y_Kick = (percentRSK*R_Shift) + Output(0,1)-R_CoolDown;
//         L_Z_Kick = 0;                                  R_Z_Kick = Output(0,2);
//         //Override L_HIP
//         feedback(7) = -getJoints(7);
//         //Override L_ANKLE_ROLL
//         // feedback(11)=feedbackRoll;
//         // feedback(10) = feedbackPitch;
//         rollAnkle(1) = R_Roll * PI/180 * 10 * (1-delta_time/duration);
//     }
//     else
//     {
//         L_X_Kick = Output(0,0);                           R_X_Kick = R_X_Shift;
//         L_Y_Kick = (percentLSK*L_Shift) + Output(0,1)-L_CoolDown;
//         R_Y_Kick = (percentLSK*L_Shift) + L_CoolDown;
//         L_Z_Kick = Output(0,2);                           R_Z_Kick = 0;
//         //Override R_HIP
//         feedback(1) = -getJoints(1);
//         //Override R_ANKLE_ROLL
//         // feedback(5)=feedbackRoll;
//         // feedback(4) = feedbackPitch;
//         rollAnkle(0) = L_Roll * PI/180 * 10 * (1-delta_time/duration);
//     }
// }

// void Kicking::bringDown(double duration, double totalPrevious)
// {
//     double delta_time = ros::Time::now().toSec() - totalPrevious;

//     if(m_position == Right_Kick)
//     {
//         L_X_Kick = 0;                                                                       R_X_Kick = 0;
//         L_Y_Kick = (R_Shift + R_Lift + R_CoolDown) * (1 - delta_time / duration);     R_Y_Kick = (R_Shift -R_CoolDown) * (1 - delta_time / duration);
//         L_Z_Kick = 0;                                                                       R_Z_Kick = 0;//0.2*0.09;
//         // L_HIP
//         feedback(7) = -getJoints(7)*(1-delta_time/duration);
//         //Override L_ANKLE_ROLL
//         // feedback(11)=feedbackRoll;
//         // feedback(10) = feedbackPitch;
//         rollAnkle(1) = 0;
//     }
//     else if(m_position == Left_Kick)
//     {
//         L_X_Kick = 0;                                                             R_X_Kick = 0;
//         L_Y_Kick = (L_Shift - L_CoolDown) * (1 - delta_time / duration);     R_Y_Kick = (L_Shift + L_Lift + L_CoolDown) * (1 - delta_time / duration);
//         L_Z_Kick = 0;/*0.2*0.09;*/                                                     R_Z_Kick = 0;
//         // R_HIP
//         feedback(1) = -getJoints(1)*(1-delta_time/duration);
//         //Override R_ANKLE_ROLL
//         // feedback(5)=feedbackRoll;
//         // feedback(4) = feedbackPitch;
//         rollAnkle(0) = 0;
//     }
//     else if(m_position == Right_Side_Kick)
//     {
//         L_X_Kick = 0;                                                                       R_X_Kick = 0;
//         L_Y_Kick = (percentRSK*R_Shift + percentRSK*R_CoolDown) * (1 - delta_time / duration);
//         R_Y_Kick = (percentRSK*R_Shift - percentRSK*R_CoolDown) * (1 - delta_time / duration) - R_CoolDown;
//         L_Z_Kick = 0;                                                                       R_Z_Kick = 0;//0.2*0.09;
//         //L_HIP
//         feedback(7) = -getJoints(7)*(1-delta_time/duration);
//         //Override L_ANKLE_ROLL
//         // feedback(11)=feedbackRoll;
//         // feedback(10) = feedbackPitch;
//         rollAnkle(1) = 0;
//     }
//     else
//     {
//         L_X_Kick = 0;                                                             R_X_Kick = 0;
//         L_Y_Kick = (percentLSK*L_Shift - percentLSK*L_CoolDown) * (1 - delta_time / duration);
//         R_Y_Kick = (percentLSK*L_Shift + percentLSK*L_CoolDown) * (1 - delta_time / duration);
//         L_Z_Kick = 0;/*0.2*0.09;*/                                                      R_Z_Kick = 0;
//         //R_HIP
//         feedback(1) = -getJoints(1)*(1-delta_time/duration);
//         //Override R_ANKLE_ROLL
//         // feedback(5)=feedbackRoll;
//         // feedback(4) = feedbackPitch;
//         rollAnkle(0) = 0;
//     }
//     Torso.tail<3>() = Vector3d(0, Body_Tilt, 0);
// }

// void Kicking::feedbackDecision()
// {
//     double input;
//     if(feedbackActive == true)
//     {
//         if(m_position == Right_Kick)
//         {
//             feedback(10) = so.r_ank_pitch.refPosFromPos; //feedbackPitch;
//             feedback(11) = so.r_ank_roll.refPosFromPos; //feedbackRoll;
//         }
//         else if(m_position == Left_Kick)
//         {
//             feedback(4) = so.l_ank_pitch.refPosFromPos; //feedbackPitch;
//             feedback(5) = so.l_ank_roll.refPosFromPos; //feedbackRoll;
//         }
//         else if(m_position == Right_Side_Kick)
//         {
//             feedback(10) = so.r_ank_pitch.refPosFromPos; //feedbackPitch;
//             feedback(11) = so.r_ank_roll.refPosFromPos; //feedbackRoll;
//         }
//         else
//         {
//             feedback(4) = so.l_ank_pitch.refPosFromPos; //feedbackPitch;
//             feedback(5) = so.l_ank_roll.refPosFromPos; //feedbackRoll;
//         }
//     }
//     else
//     {
//         feedback(4) = 0;
//         feedback(5) = 0;
//         feedback(10) = 0;
//         feedback(11) = 0;
//     }
// }

// void Kicking::feedbackDSPAnklePitchPos2Pos()
// {
//     double input;
//     input = mbc.outputFeedback(mbc.pitchRef*M_PI/180,mbc.wholeBodyStates,mbc.dlqr.K,mbc.dlqr.Kr);
//     so.r_ank_pitch.refPosFromPos = input;
//     so.l_ank_pitch.refPosFromPos = input;
//     if(servoFeedbackActive)
//     {
//         feedback(10) = so.l_ank_pitch.refPosFromPos;
//         feedback(4) = so.r_ank_pitch.refPosFromPos;
//     }
//     else
//     {
//         feedback(10) = 0;//so.l_ank_pitch.refPosFromPos;
//         feedback(4) = 0;//so.r_ank_pitch.refPosFromPos;
//     }
//     std::cout<<"L_Ankle_Pitch_Effort(DEG): "<<feedback(10)*180/PI<<std::endl;
//     std::cout<<"R_Ankle_Pitch_Effort(DEG): "<<feedback(4)*180/PI<<std::endl;
// }

// void Kicking::updateKickingParam()
// {
//     Body_Height         = kicking_param_.Body_Height;
//     Body_Tilt           = kicking_param_.Body_Tilt * DEGREE2RADIAN;
//     Leg_Z               = kicking_param_.Leg_Z;
//     Leg_X               = kicking_param_.Leg_X;
//     Leg_Y               = kicking_param_.Leg_Y;

//     Leg_R_X             = kicking_param_.Leg_R_X;
//     Leg_L_X             = kicking_param_.Leg_L_X;
//     R_Roll              = kicking_param_.R_Roll;
//     L_Roll              = kicking_param_.L_Roll;

//     R_Shift             = kicking_param_.R_Shift;
//     L_Shift             = kicking_param_.L_Shift;
//     R_Lift              = kicking_param_.R_Lift;
//     L_Lift              = kicking_param_.L_Lift;
//     X_Retract           = kicking_param_.X_Retract;
//     Z_Retract           = kicking_param_.Z_Retract;

//     Time_Shift          = kicking_param_.Time_Shift;
//     Time_Lift           = kicking_param_.Time_Lift;
//     Time_Retract        = kicking_param_.Time_Retract;
//     Time_Kick           = kicking_param_.Time_Kick;
//     Time_CoolDown       = kicking_param_.Time_CoolDown;
//     Time_BringDown      = kicking_param_.Time_BringDown;

//     Gain_Angle_Pitch      = kicking_param_.Gain_Angle_Pitch;
//     Gain_Velocity_Pitch   = kicking_param_.Gain_Velocity_Pitch;
//     Gain_Angle_Roll       = kicking_param_.Gain_Angle_Roll;
//     Gain_Velocity_Roll    = kicking_param_.Gain_Velocity_Roll;
//     Gain_Integral        = kicking_param_.Gain_Integral;
// }

// void Kicking::motion_arms()
// {
//     getJoints(12)=0;
//     getJoints(13)=-35*DEGREE2RADIAN;
//     getJoints(14)=-120*DEGREE2RADIAN;
//     getJoints(15)=0;
//     getJoints(16)=-35*DEGREE2RADIAN;
//     getJoints(17)=-120*DEGREE2RADIAN;
// }

// void Kicking::forwardKinematic()
// {
//     R_HIP.linear()   = Eigen::Matrix3d::Identity();
//     R_KNEE.linear()  = Eigen::Matrix3d::Identity();
//     R_ANKLE.linear() = Eigen::Matrix3d::Identity();
//     L_HIP.linear()   = Eigen::Matrix3d::Identity();
//     L_KNEE.linear()  = Eigen::Matrix3d::Identity();
//     L_ANKLE.linear() = Eigen::Matrix3d::Identity();

//     R_HIP.rotate(   AngleAxisd(current_joint_pos(0) , Vector3d( 0, 0, 1) * joint_axis_direction_(0)) *
//                     AngleAxisd(current_joint_pos(1) , Vector3d( 1, 0, 0) * -joint_axis_direction_(1)) *
//                     AngleAxisd(current_joint_pos(2) , Vector3d( 0, 1, 0) * joint_axis_direction_(2)));
//     R_KNEE.rotate(  AngleAxisd(current_joint_pos(3) , Vector3d( 0, 1, 0) * joint_axis_direction_(3)));
//     R_ANKLE.rotate( AngleAxisd(current_joint_pos(4) , Vector3d( 0, 1, 0) * joint_axis_direction_(4)) *
//                     AngleAxisd(current_joint_pos(5) , Vector3d( 1, 0, 0) * joint_axis_direction_(5)));
//     L_HIP.rotate(   AngleAxisd(current_joint_pos(6) , Vector3d( 0, 0, 1) * joint_axis_direction_(6)) *
//                     AngleAxisd(current_joint_pos(7) , Vector3d( 1, 0, 0) * -joint_axis_direction_(7)) *
//                     AngleAxisd(current_joint_pos(8) , Vector3d( 0, 1, 0) * joint_axis_direction_(8)));
//     L_KNEE.rotate(  AngleAxisd(current_joint_pos(9) , Vector3d( 0, 1, 0) * joint_axis_direction_(9)));
//     L_ANKLE.rotate( AngleAxisd(current_joint_pos(10), Vector3d( 0, 1, 0) * joint_axis_direction_(10)) *
//                     AngleAxisd(current_joint_pos(11), Vector3d( 1, 0, 0) * joint_axis_direction_(11)));

//     m_R_FOOT = BASE * R_HIP * R_KNEE * R_ANKLE * R_FOOT;
//     m_L_FOOT = BASE * L_HIP * L_KNEE * L_ANKLE * L_FOOT;

//     R_FOOT_WORLD = m_R_FOOT.translation();
//     L_FOOT_WORLD = m_L_FOOT.translation();

//     L_FOOT_WORLD(0) *= -1;

//     R_ROT_WORLD = robotis_framework::convertRotationToRPY(m_R_FOOT.linear());
//     L_ROT_WORLD = robotis_framework::convertRotationToRPY(m_L_FOOT.linear());

//     // std::cout<<"ForwardKinematic:"<<std::endl;
//     // std::cout<<"L_X: "<<L_FOOT_WORLD(0)<<"\tR_X: "<<R_FOOT_WORLD(0)<<"\t||\tL_Y: "<<L_FOOT_WORLD(1)<<"\tR_Y: "<<R_FOOT_WORLD(1)<<"\t||\tL_Z: "<<L_FOOT_WORLD(2)<<"\tR_Z: "<<R_FOOT_WORLD(2)<<std::endl;

//     ////std::cout<<"L_Z: "<<std::fixed << std::setprecision(4)<<L_FOOT_WORLD(2)<<"\tR_Z = "<<std::fixed << std::setprecision(4)<<R_FOOT_WORLD(2)<<std::endl;
//     if(L_FOOT_WORLD(2) > R_FOOT_WORLD(2) && fabs(L_FOOT_WORLD(2)-R_FOOT_WORLD(2))>0.01){
//         IndexSupport = 2;//1; //SSP Kanan
//     }else if(L_FOOT_WORLD(2) < R_FOOT_WORLD(2) && fabs(L_FOOT_WORLD(2)-R_FOOT_WORLD(2))>0.01){
//         IndexSupport = 1;//2; //SSP Kiri
//     }else if(fabs(L_FOOT_WORLD(2)-R_FOOT_WORLD(2))<=0.01){
//         IndexSupport = 0;
//     }
// }

// void Kicking::wholeBodyCOM()
// {
//     rotIMU = robotis_framework::convertRPYToRotation(m_imuOri(0),m_imuOri(1),0); // Tanpa Yaw
//     op3_kd_->op3_link_data_[0]->orientation_=rotIMU;
//     op3_kd_->calcForwardKinematics(0);
//     COM = op3_kd_->calcCOM(0);
//     //    std::cout<<"WholeBodyCOM x"<<COM(0)<<"\ty:"<<COM(1)<<"\tz"<<COM(2)<<std::endl;
//     tf::quaternionEigenToMsg(robotis_framework::convertRotationToQuaternion(op3_kd_->op3_link_data_[31]->orientation_),dataLogMsgs.RFootFK.orientation);
//     tf::quaternionEigenToMsg(robotis_framework::convertRotationToQuaternion(op3_kd_->op3_link_data_[30]->orientation_),dataLogMsgs.LFootFK.orientation);
//     tf::quaternionEigenToMsg(robotis_framework::convertRotationToQuaternion(op3_kd_->op3_link_data_[29]->orientation_),dataLogMsgs.TorsoIK.orientation);
// }
// void Kicking::globalInertia()
// {
//   op3_kd_->op3_link_data_[0]->orientation_ = MatrixXd::Identity(3, 3);
//   op3_kd_->calcForwardKinematics(0);
//   for(int id=0;id<32;id++) op3_kd_->calcGlobalInertia(id);
//   totalInertia = op3_kd_->calcTotalInertia(0);
// //   std::cout<<"Total Inertia: "<<totalInertia<<std::endl;
// }

// void Kicking::wholeBodyCOM_LSSP()
// {
//     // passive x,y,z = 23,24,25 //l_leg_end = 30
//   rotIMU = robotis_framework::convertRPYToRotation(m_imuOri(0),m_imuOri(1),0); // Tanpa Yaw
//   op3_kd_->op3_link_data_[0]->orientation_=rotIMU;
//   op3_kd_->op3_link_data_[0]->orientation_LSSP_ = rotIMU;

//   for(int i=0; i<=2; i++)
//     posLFootFromBase(i) = -op3_kd_->op3_link_data_[30]->position_(i);

//   passiveLSSP = rotIMU.inverse()*(posLFootFromBase);
//   for(int i=0; i<=2; i++)
//     op3_kd_->op3_link_data_[25]->relative_position_LSSP_(i) = passiveLSSP(i);

//   op3_kd_->calcFKLeftSSP(0);
//   COM_LSSP = op3_kd_->calcCOM_LSSP(0);
//   COM_LSSP_Length = COM_LSSP.norm();
//   COMLSSPrpy(0) = -atan(COM_LSSP(1)/COM_LSSP(2));
//   COMLSSPrpy(1) = atan(COM_LSSP(0)/COM_LSSP(2));
// //   std::cout<<"<<<Left SSP COM x"<<COM_LSSP(0)<<"\ty:"<<COM_LSSP(1)<<"\tz"<<COM_LSSP(2)<<std::endl;
// }

// void Kicking::wholeBodyCOM_RSSP()
// {
//     // passive x,y,z = 23,24,25 //r_leg_end = 31
//  op3_kd_->op3_link_data_[0]->orientation_RSSP_ = rotIMU;
//  for(int i=0; i<=2; i++)
//     posRFootFromBase(i) = -op3_kd_->op3_link_data_[31]->position_(i);

//   passiveRSSP = rotIMU.inverse()*(posRFootFromBase);
//   for(int i=0; i<=2; i++)
//     op3_kd_->op3_link_data_[25]->relative_position_RSSP_(i) = passiveRSSP(i);

//   op3_kd_->calcFKRightSSP(0);
//   COM_RSSP = op3_kd_->calcCOM_RSSP(0);
//   COM_RSSP_Length = COM_RSSP.norm();
//   COMRSSPrpy(0) = -atan(COM_RSSP(1)/COM_RSSP(2));
//   COMRSSPrpy(1) = atan(COM_RSSP(0)/COM_RSSP(2));
// //   std::cout<<"Right >>> SSP COM x"<<COM_RSSP(0)<<"\ty:"<<COM_RSSP(1)<<"\tz"<<COM_RSSP(2)<<std::endl;
// }

// void Kicking::wholeBodyCOM_DSP()
// {
//     // COM_DSP = (COM_LSSP+COM_RSSP)/2;
//     // passive x,y,z = 23,24,25 //l_leg_end = 30
//   op3_kd_->op3_link_data_[0]->orientation_DSP_ = rotIMU;

//   for(int i=0; i<=2; i++)
//   {
//       posLFootFromBase(i) = -op3_kd_->op3_link_data_[30]->position_(i);
//       posRFootFromBase(i) = -op3_kd_->op3_link_data_[31]->position_(i);
//   }
//   /*
//   if((op3_kd_->op3_link_data_[30]->orientation_ > angleTipMax)
//   ||((op3_kd_->op3_link_data_[31]->orientation_ > angleTipMax)))
//   {

//   }
//   else if((op3_kd_->op3_link_data_[30]->orientation_ > angleTipMin)
//   ||((op3_kd_->op3_link_data_[31]->orientation_ > angleTipMin)))
//   {

//   }
//   else
//   {

//   }
//   */

//   posDSPFromBase = (posLFootFromBase + posRFootFromBase)/2;

//   passiveDSP = rotIMU.inverse()*(posDSPFromBase);
//   for(int i=0; i<=2; i++)
//     op3_kd_->op3_link_data_[25]->relative_position_DSP_(i) = passiveDSP(i);

//   op3_kd_->calcFKDSP(0);
//   COM_DSP = op3_kd_->calcCOM_DSP(0);
//   COM_DSP_Length = COM_DSP.norm();
//   COMDSPrpy(0) = -atan(COM_DSP(1)/COM_DSP(2));
//   COMDSPrpy(1) = atan(COM_DSP(0)/COM_DSP(2));
// //   std::cout<<"<<<DSP COM x"<<COM_DSP(0)<<"\ty:"<<COM_DSP(1)<<"\tz"<<COM_DSP(2)<<std::endl;
// }

// void Kicking::globalInertiaLSSP()
// {
//   op3_kd_->op3_link_data_[0]->orientation_LSSP_ = MatrixXd::Identity(3, 3);
//   op3_kd_->op3_link_data_[25]->relative_position_LSSP_ = -op3_kd_->op3_link_data_[30]->position_;
//   op3_kd_->calcFKLeftSSP(0);
//   for(int id=0;id<32;id++) op3_kd_->calcGlobalInertiaLSSP(id);
//   totalInertiaLSSP = op3_kd_->calcTotalInertiaLSSP(0);
// //   std::cout<<"Total Inertia LSSP: "<<totalInertiaLSSP<<std::endl;
// }

// void Kicking::globalInertiaRSSP()
// {
//   op3_kd_->op3_link_data_[0]->orientation_RSSP_ = MatrixXd::Identity(3, 3);
//   op3_kd_->op3_link_data_[25]->relative_position_RSSP_ = -op3_kd_->op3_link_data_[31]->position_;
//   op3_kd_->calcFKRightSSP(0);
//   for(int id=0;id<32;id++) op3_kd_->calcGlobalInertiaRSSP(id);
//   totalInertiaRSSP = op3_kd_->calcTotalInertiaRSSP(0);
// //   std::cout<<"Total Inertia RSSP: "<<totalInertiaRSSP<<std::endl;
// }

// void Kicking::globalInertiaDSP()
// {
//   op3_kd_->op3_link_data_[0]->orientation_DSP_ = MatrixXd::Identity(3, 3);
//   op3_kd_->op3_link_data_[25]->relative_position_DSP_ = posDSPFromBase;
//   op3_kd_->calcFKDSP(0);
//   for(int id=0;id<32;id++) op3_kd_->calcGlobalInertiaDSP(id);
//   totalInertiaDSP = op3_kd_->calcTotalInertiaDSP(0);
// //   std::cout<<"Total Inertia DSP: "<<totalInertiaDSP<<std::endl;
// }

// void Kicking::kickStatus(const std::string &command)
// {
//   std_msgs::String _commnd_msg;
//   _commnd_msg.data = command;
//   kick_status_pub_.publish(_commnd_msg);
//   ////std::cout << "kickStatus : " << _commnd_msg.data << std::endl;
// }

// void Kicking::dataLogPublish()
// {
//     //Inverse Kinematic Data Log Message
//     tf::pointEigenToMsg(R_Leg.head<3>(), dataLogMsgs.RFootIK.position);
//     tf::pointEigenToMsg(L_Leg.head<3>(), dataLogMsgs.LFootIK.position);
//     tf::pointEigenToMsg(Torso.head<3>(), dataLogMsgs.TorsoIK.position);
//     tf::quaternionEigenToMsg(robotis_framework::convertRPYToQuaternion(R_Leg(3), R_Leg(4), R_Leg(5)), dataLogMsgs.RFootIK.orientation);
//     tf::quaternionEigenToMsg(robotis_framework::convertRPYToQuaternion(L_Leg(3), L_Leg(4), L_Leg(5)), dataLogMsgs.LFootIK.orientation);
//     tf::quaternionEigenToMsg(robotis_framework::convertRPYToQuaternion(Torso(3), Torso(4), Torso(5)), dataLogMsgs.TorsoIK.orientation);

//     //Forward Kinematic Data Log Message
//     tf::pointEigenToMsg(op3_kd_->op3_link_data_[31]->position_, dataLogMsgs.RFootFK.position);
//     tf::pointEigenToMsg(op3_kd_->op3_link_data_[30]->position_, dataLogMsgs.LFootFK.position);
//     tf::pointEigenToMsg(op3_kd_->op3_link_data_[29]->position_, dataLogMsgs.TorsoFK.position);

//     tf::vectorEigenToMsg(COM,dataLogMsgs.baseInertia.com);
//     tf::vectorEigenToMsg(COM_LSSP,dataLogMsgs.LSSPInertia.com);
//     tf::vectorEigenToMsg(COM_RSSP,dataLogMsgs.RSSPInertia.com);
//     tf::vectorEigenToMsg(COM_DSP,dataLogMsgs.DSPInertia.com);

//     dataLogMsgs.baseInertia.ixx = totalInertia(0,0);
//     dataLogMsgs.baseInertia.ixy = totalInertia(0,1);
//     dataLogMsgs.baseInertia.ixz = totalInertia(0,2);
//     dataLogMsgs.baseInertia.iyy = totalInertia(1,1);
//     dataLogMsgs.baseInertia.iyz = totalInertia(1,2);
//     dataLogMsgs.baseInertia.izz = totalInertia(2,2);

//     dataLogMsgs.LSSPInertia.ixx = totalInertiaLSSP(0,0);
//     dataLogMsgs.LSSPInertia.ixy = totalInertiaLSSP(0,1);
//     dataLogMsgs.LSSPInertia.ixz = totalInertiaLSSP(0,2);
//     dataLogMsgs.LSSPInertia.iyy = totalInertiaLSSP(1,1);
//     dataLogMsgs.LSSPInertia.iyz = totalInertiaLSSP(1,2);
//     dataLogMsgs.LSSPInertia.izz = totalInertiaLSSP(2,2);

//     dataLogMsgs.RSSPInertia.ixx = totalInertiaRSSP(0,0);
//     dataLogMsgs.RSSPInertia.ixy = totalInertiaRSSP(0,1);
//     dataLogMsgs.RSSPInertia.ixz = totalInertiaRSSP(0,2);
//     dataLogMsgs.RSSPInertia.iyy = totalInertiaRSSP(1,1);
//     dataLogMsgs.RSSPInertia.iyz = totalInertiaRSSP(1,2);
//     dataLogMsgs.RSSPInertia.izz = totalInertiaRSSP(2,2);

//     dataLogMsgs.DSPInertia.ixx = totalInertiaDSP(0,0);
//     dataLogMsgs.DSPInertia.ixy = totalInertiaDSP(0,1);
//     dataLogMsgs.DSPInertia.ixz = totalInertiaDSP(0,2);
//     dataLogMsgs.DSPInertia.iyy = totalInertiaDSP(1,1);
//     dataLogMsgs.DSPInertia.iyz = totalInertiaDSP(1,2);
//     dataLogMsgs.DSPInertia.izz = totalInertiaDSP(2,2);

//     tf::pointEigenToMsg(ZMPLSSP, dataLogMsgs.ZMPLSSP);
//     tf::pointEigenToMsg(ZMPRSSP, dataLogMsgs.ZMPRSSP);
//     tf::pointEigenToMsg(ZMPDSP, dataLogMsgs.ZMPDSP);

//     tf::pointEigenToMsg(ZMPLSSP_accFil, dataLogMsgs.ZMPLSSP_accFil);
//     tf::pointEigenToMsg(ZMPRSSP_accFil, dataLogMsgs.ZMPRSSP_accFil);
//     tf::pointEigenToMsg(ZMPDSP_accFil, dataLogMsgs.ZMPDSP_accFil);

//     tf::pointEigenToMsg(COMLSSPrpy, dataLogMsgs.COMLSSPrpy);
//     tf::pointEigenToMsg(COMRSSPrpy, dataLogMsgs.COMRSSPrpy);
//     tf::pointEigenToMsg(COMDSPrpy, dataLogMsgs.COMDSPrpy);

//     tf::pointEigenToMsg(m_imuOri*180/PI,dataLogMsgs.IMUdegrees);
//     tf::pointEigenToMsg(m_imuAccFil,dataLogMsgs.accelFiltered);

//     // Servo Data Log Messages
//     // dataLogMsgs.LAnklePitch_torqueIn      = so.l_ank_pitch.torqueIn;
//     dataLogMsgs.LAnklePitch_velocityNow   = so.l_ank_pitch.velocityNow;
//     dataLogMsgs.LAnklePitch_positionIK    = so.l_ank_pitch.positionIK;
//     dataLogMsgs.LAnklePitch_positionNow   = so.l_ank_pitch.positionNow;
//     // dataLogMsgs.LAnklePitch_torqueEst     = so.l_ank_pitch.torqueEst;
//     // dataLogMsgs.LAnklePitch_refPosFromTrq = so.l_ank_pitch.refPosFromTrq;
//     dataLogMsgs.LAnklePitch_refPosFromPos = so.l_ank_pitch.refPosFromPos;

//     // dataLogMsgs.RAnklePitch_torqueIn      = so.r_ank_pitch.torqueIn;
//     dataLogMsgs.RAnklePitch_velocityNow   = so.r_ank_pitch.velocityNow;
//     dataLogMsgs.RAnklePitch_positionIK    = so.r_ank_pitch.positionIK;
//     dataLogMsgs.RAnklePitch_positionNow   = so.r_ank_pitch.positionNow;
//     // dataLogMsgs.RAnklePitch_torqueEst     = so.r_ank_pitch.torqueEst;
//     // dataLogMsgs.RAnklePitch_refPosFromTrq = so.r_ank_pitch.refPosFromTrq;
//     dataLogMsgs.RAnklePitch_refPosFromPos = so.r_ank_pitch.refPosFromPos;

//     data_log_pub_.publish(dataLogMsgs);
// }

// double Kicking::EMAV2(const std::vector<double> movAvg,double n ){
//     double alpha = 2/(n+1);
//     double sum, numerator, denominator, sum_numerator, sum_denominator;
//     for (int i=0;i<=n;i++){
//         if (i==0){
//         numerator = movAvg[i];
//         denominator = 1;
//         }
//         else {
//         numerator = pow((1-alpha),i) * movAvg[i];
//         denominator = pow((1-alpha),i);
//         }
//         sum_numerator += numerator;
//         sum_denominator += denominator;
//         // equation = pow((1-alpha),i+1)*args[i]/pow((1-alpha),i+1);
//         sum = sum_numerator/sum_denominator;
//     }
//     return sum;
// }

// Eigen::Vector3d Kicking::accelCOM(Eigen::Vector3d COM,Eigen::Vector3d posFootFromBase)
// {
//     // omega_dot = (m_imuGyr - pre_omega) / dT;
//     omega_dot = (m_imuGyr - pre_omega) / mbc.sys_d.Ts;
//     omega_cross = robotis_framework::calcHatto(m_imuGyr);
//     pelvisToCOM = COM-posFootFromBase;
//     m_accelCOM = m_imuAcc + omega_cross*(omega_cross*pelvisToCOM) + robotis_framework::calcHatto(omega_dot)*pelvisToCOM;
//     pre_omega = m_imuGyr;
//     return m_accelCOM;
// }

// Eigen::Vector3d Kicking::accelCOMFil(Eigen::Vector3d COM,Eigen::Vector3d posFootFromBase)
// {
//     // Average Filter without Buffer
//     // for(int i = 0; i < MeasurementsToAverage; ++i)
//     // {
//     //     Average += m_imuAcc;
//     // }
//     // Average /= MeasurementsToAverage;
//     // m_imuAccFil = Average;
//     // Average Filter with Buffer
//     m_imuAccFil = movingAverage(m_imuAcc);
//     omega_dot(0) = -(m_imuAccFil(1))/posFootFromBase.norm();
//     omega_dot(1) = (m_imuAccFil(0))/posFootFromBase.norm();
//     omega_dot(2) = 0;
//     // omega_dot = (m_imuGyr - pre_omega) / dT;
//     omega_cross = robotis_framework::calcHatto(m_imuGyr);
//     pelvisToCOM = COM-posFootFromBase;
//     m_accelCOMFil = m_imuAccFil + omega_cross*(omega_cross*pelvisToCOM) + robotis_framework::calcHatto(omega_dot)*pelvisToCOM;
//     // pre_omega = m_imuGyr;
//     return m_accelCOMFil;
// }

// void Kicking::calculateZMP()
// {
//     ZMPLSSP(0) = COM_LSSP(0)-COM_LSSP(2)*accelCOM(COM_LSSP, posLFootFromBase).x()/G_CONST;
//     ZMPLSSP(1) = COM_LSSP(1)-COM_LSSP(2)*accelCOM(COM_LSSP, posLFootFromBase).y()/G_CONST;
//     ZMPRSSP(0) = COM_RSSP(0)-COM_RSSP(2)*accelCOM(COM_RSSP, posRFootFromBase).x()/G_CONST;
//     ZMPRSSP(1) = COM_RSSP(1)-COM_RSSP(2)*accelCOM(COM_RSSP, posRFootFromBase).y()/G_CONST;
//     ZMPDSP(0) = COM_DSP(0)-COM_DSP(2)*accelCOM(COM_DSP, posDSPFromBase).x()/G_CONST;
//     ZMPDSP(1) = COM_DSP(1)-COM_DSP(2)*accelCOM(COM_DSP, posDSPFromBase).y()/G_CONST;

//     //accel Filtered
//     ZMPLSSP_accFil(0) = COM_LSSP(0)-COM_LSSP(2)*accelCOMFil(COM_LSSP, posLFootFromBase).x()/(G_CONST);
//     ZMPLSSP_accFil(1) = COM_LSSP(1)-COM_LSSP(2)*accelCOMFil(COM_LSSP, posLFootFromBase).y()/(G_CONST);
//     ZMPRSSP_accFil(0) = COM_RSSP(0)-COM_RSSP(2)*accelCOMFil(COM_RSSP, posRFootFromBase).x()/(G_CONST);
//     ZMPRSSP_accFil(1) = COM_RSSP(1)-COM_RSSP(2)*accelCOMFil(COM_RSSP, posRFootFromBase).y()/(G_CONST);
//     ZMPDSP_accFil(0) = COM_DSP(0)-COM_DSP(2)*accelCOMFil(COM_DSP, posDSPFromBase).x()/(G_CONST);
//     ZMPDSP_accFil(1) = COM_DSP(1)-COM_DSP(2)*accelCOMFil(COM_DSP, posDSPFromBase).y()/(G_CONST);
// }

// void Kicking::densisInput()
// {
//     densis.m_time = ros::Time::now().toSec() - densis.m_time_start;
//     densis.inputSignal = densis.densisInput(densis.m_time);
//     if(ankleMode == densisDSP) //0
//     {
//         so.r_ank_pitch.refPosFromPos = densis.inputSignal;
//         so.l_ank_pitch.refPosFromPos = densis.inputSignal;
//         feedback(4) = so.r_ank_pitch.refPosFromPos; //R_Ankle_Pitch
//         feedback(5) = 0; //R_Ankle_Roll
//         feedback(10) = so.l_ank_pitch.refPosFromPos; //L_Ankle_Pitch
//         feedback(11) = 0; //L_Ankle_Roll
//     }
//     else if(ankleMode == densisRAnkPitch) //1
//     {
//         liftingHold();
//         so.r_ank_pitch.refPosFromPos = densis.inputSignal;
//         feedback(4) = so.r_ank_pitch.refPosFromPos; //R_Ankle_Pitch
//         feedback(5) = 0; //R_Ankle_Roll
//         feedback(10) = 0; //L_Ankle_Pitch
//         feedback(11) = 0; //L_Ankle_Roll
//     }
//     else if(ankleMode == densisLAnkPitch) //2
//     {
//         liftingHold();
//         so.l_ank_pitch.refPosFromPos = densis.inputSignal;
//         feedback(4) = 0; //R_Ankle_Pitch
//         feedback(5) = 0; //R_Ankle_Roll
//         feedback(10) = so.l_ank_pitch.refPosFromPos; //L_Ankle_Pitch
//         feedback(11) = 0; //L_Ankle_Roll
//         // so.servoStateObserver(so.l_ank_pitch.refPosFromPos, so.l_ank_pitch.servoStates, so.l_ank_pitch.servoOutputAngle);
//         // so.torqueObserver(&so.l_ank_pitch);
//     }
//     else if(ankleMode == densisRAnkRoll) //3
//     {
//         liftingHold();
//         so.r_ank_roll.refPosFromPos = densis.inputSignal;
//         feedback(4) = 0; //R_Ankle_Pitch
//         feedback(5) = so.r_ank_roll.refPosFromPos; //R_Ankle_Roll
//         feedback(10) = 0; //L_Ankle_Pitch
//         feedback(11) = 0; //L_Ankle_Roll
//     }
//     else if(ankleMode == densisLAnkRoll) //4
//     {
//         liftingHold();
//         so.l_ank_roll.refPosFromPos = densis.inputSignal;
//         feedback(4) = 0; //R_Ankle_Pitch
//         feedback(5) = 0; //R_Ankle_Roll
//         feedback(10) = 0; //L_Ankle_Pitch
//         feedback(11) = so.l_ank_roll.refPosFromPos; //L_Ankle_Roll
//     }
//     else
//     {
//         DSPHold();
//         so.resetServoValues();
//         feedback(4) = 0; //R_Ankle_Pitch
//         feedback(5) = 0; //R_Ankle_Roll
//         feedback(10) = 0; //L_Ankle_Pitch
//         feedback(11) = 0; //L_Ankle_Roll
//     }
//     if(servoFeedbackActive == true)
//     {
//         feedback(4) = so.r_ank_pitch.refPosFromPos;
//         feedback(5) = so.r_ank_roll.refPosFromPos;
//         feedback(10) = so.l_ank_pitch.refPosFromPos;
//         feedback(11) = so.l_ank_roll.refPosFromPos;
//     }
//     else
//     {
//         feedback(4) = 0; //R_Ankle_Pitch
//         feedback(5) = 0; //R_Ankle_Roll
//         feedback(10) = 0; //L_Ankle_Pitch
//         feedback(11) = 0; //L_Ankle_Roll
//     }
// }

// Eigen::Vector3d Kicking::movingAverage(Eigen::Vector3d rawData)
// {
//     // Average Filter with Buffer
//     for(int i=0; i<MeasurementsToAverage; i++)
//     {
//         cumsum.x() += bufferAccX[i];
//         cumsum.y() += bufferAccY[i];
//         cumsum.z() += bufferAccZ[i];
//     }
//     filteredData = cumsum/MeasurementsToAverage;
//     cumsum = Eigen::Vector3d::Zero(3);
//     // //Shift all 1 to left
//     for (size_t i = 0; i < MeasurementsToAverage - 1; i++)
//     {
//         bufferAccX.at(i) = bufferAccX.at(i + 1);
//         bufferAccY.at(i) = bufferAccY.at(i + 1);
//         bufferAccZ.at(i) = bufferAccZ.at(i + 1);
//     }
//     // //add new data to the left
//     bufferAccX.at(bufferAccX.size() - 1) = rawData.x();
//     bufferAccY.at(bufferAccY.size() - 1) = rawData.y();
//     bufferAccZ.at(bufferAccZ.size() - 1) = rawData.z();
//     return filteredData;
// }

// void Kicking::densisPublish()
// {
//     tf::pointEigenToMsg(m_imuOri,densisMsgs.IMUorientation);
//     tf::pointEigenToMsg(m_imuGyr,densisMsgs.IMUgyro);
//     tf::pointEigenToMsg(COM_LSSP, densisMsgs.COMLSSP);
//     tf::pointEigenToMsg(COM_RSSP, densisMsgs.COMRSSP);
//     tf::pointEigenToMsg(COM_DSP, densisMsgs.COMDSP);
//     tf::pointEigenToMsg(COMLSSPrpy, densisMsgs.COMLSSPrpy);
//     tf::pointEigenToMsg(COMRSSPrpy, densisMsgs.COMRSSPrpy);
//     tf::pointEigenToMsg(COMDSPrpy, densisMsgs.COMDSPrpy);
//     tf::pointEigenToMsg(ZMPLSSP_accFil, densisMsgs.ZMPLSSP);
//     tf::pointEigenToMsg(ZMPRSSP_accFil, densisMsgs.ZMPRSSP);
//     tf::pointEigenToMsg(ZMPDSP_accFil, densisMsgs.ZMPDSP);

//     // Servo Data Log Messages
//     densisMsgs.LAnklePitch_velocityNow   = so.l_ank_pitch.velocityNow;
//     densisMsgs.LAnklePitch_positionIK    = so.l_ank_pitch.positionIK;
//     densisMsgs.LAnklePitch_positionNow   = so.l_ank_pitch.positionNow;
//     densisMsgs.LAnklePitch_refPosFromPos = so.l_ank_pitch.refPosFromPos;

//     densisMsgs.RAnklePitch_velocityNow   = so.r_ank_pitch.velocityNow;
//     densisMsgs.RAnklePitch_positionIK    = so.r_ank_pitch.positionIK;
//     densisMsgs.RAnklePitch_positionNow   = so.r_ank_pitch.positionNow;
//     densisMsgs.RAnklePitch_refPosFromPos = so.r_ank_pitch.refPosFromPos;

//     densisMsgs.COMDSPrpy_pitch_Est = mbc.yest[0];//wholeBodyStates[0];
//     densisMsgs.gyro_pitch_Est = mbc.yest[1];//wholeBodyStates[1];

//     densisMsgs.COMDSPrpy_pitch_Meas = mbc.ymeas[0];//ymeas[0];
//     densisMsgs.gyro_pitch_Meas = mbc.ymeas[1];//ymeas[1];
//     densisMsgs.gyro_pitch_Fil = gyro_y_;
//     densis_pub_.publish(densisMsgs);
// }

// }
