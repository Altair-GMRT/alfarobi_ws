#include "kicking/kicking.h"

namespace robotis_op
{

    Kicking::Kicking()
        : control_cycle_msec_(8),
          phase(0.0)
    {
        enable_ = false;
        module_name_ = "kicking";
        control_mode_ = robotis_framework::PositionControl;
        kicking_state_ = KickingInitPose;

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
        result_["r_sho_roll"] = new robotis_framework::DynamixelState();
        result_["r_el"] = new robotis_framework::DynamixelState();

        result_["l_sho_pitch"] = new robotis_framework::DynamixelState();
        result_["l_sho_roll"] = new robotis_framework::DynamixelState();
        result_["l_el"] = new robotis_framework::DynamixelState();

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
        joint_table_["r_sho_roll"] = 13;
        joint_table_["r_el"] = 14;

        joint_table_["l_sho_pitch"] = 15;
        joint_table_["l_sho_roll"] = 16;
        joint_table_["l_el"] = 17;

        //offset
        joint[0] = "r_hip_yaw";
        joint[1] = "r_hip_roll";
        joint[2] = "r_hip_pitch";
        joint[3] = "r_knee";
        joint[4] = "r_ank_pitch";
        joint[5] = "r_ank_roll";

        joint[6] = "l_hip_yaw";
        joint[7] = "l_hip_roll";
        joint[8] = "l_hip_pitch";
        joint[9] = "l_knee";
        joint[10] = "l_ank_pitch";
        joint[11] = "l_ank_roll";

        //op3_offset
        op3_joint[1] = "r_sho_pitch";
        op3_joint[2] = "l_sho_pitch";
        op3_joint[3] = "r_sho_roll";
        op3_joint[4] = "l_sho_roll";
        op3_joint[5] = "r_el";
        op3_joint[6] = "l_el";
        op3_joint[7] = "r_hip_yaw";
        op3_joint[8] = "l_hip_yaw";
        op3_joint[9] = "r_hip_roll";
        op3_joint[10] = "l_hip_roll";
        op3_joint[11] = "r_hip_pitch";
        op3_joint[12] = "l_hip_pitch";
        op3_joint[13] = "r_knee";
        op3_joint[14] = "l_knee";
        op3_joint[15] = "r_ank_pitch";
        op3_joint[16] = "l_ank_pitch";
        op3_joint[17] = "r_ank_roll";
        op3_joint[18] = "l_ank_roll";
        op3_joint[19] = "head_pan";
        op3_joint[20] = "head_tilt";

        target_position_ = MatrixXd::Zero(1, result_.size());
        current_position_ = MatrixXd::Zero(1, result_.size());
        init_position_ = Eigen::MatrixXd::Zero(1, result_.size());
        joint_axis_direction_ = MatrixXi::Zero(1, result_.size());

        BASE = Affine3d(Translation3d(Vector3d(0, 0, 0)));
        R_HIP = Affine3d(Translation3d(Vector3d(k.HIP_OFFSET_X, -k.LEG_SIDE_OFFSET, -k.HIP_OFFSET_Z)));
        R_KNEE = Affine3d(Translation3d(Vector3d(0, 0, -k.THIGH_LENGTH)));
        R_ANKLE = Affine3d(Translation3d(Vector3d(0, 0, -k.CALF_LENGTH)));
        R_FOOT = Affine3d(Translation3d(Vector3d(0, 0, -k.ANKLE_LENGTH)));
        m_R_FOOT = Affine3d(Translation3d(Vector3d(0, 0, 0)));
        L_HIP = Affine3d(Translation3d(Vector3d(k.HIP_OFFSET_X, k.LEG_SIDE_OFFSET, -k.HIP_OFFSET_Z)));
        L_KNEE = Affine3d(Translation3d(Vector3d(0, 0, -k.THIGH_LENGTH)));
        L_ANKLE = Affine3d(Translation3d(Vector3d(0, 0, -k.CALF_LENGTH)));
        L_FOOT = Affine3d(Translation3d(Vector3d(0, 0, -k.ANKLE_LENGTH)));
        m_L_FOOT = Affine3d(Translation3d(Vector3d(0, 0, 0)));

        VectorXd LFootWorld(4);
        VectorXd RFootWorld(4);
    }

    Kicking::~Kicking()
    {
        queue_thread_.join();
    }

    void Kicking::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
    {
        queue_thread_ = boost::thread(boost::bind(&Kicking::queueThread, this));
        control_cycle_msec_ = control_cycle_msec;
        real_running_ = false;

        config_path_ = ros::package::getPath("op3_manager") + "/config/GlobalConfig.yaml";
        offset_path_ = ros::package::getPath("op3_manager") + "/config/offset.yaml";

        ROS_DEBUG("Motion: Kicking entry");

        loadConfig();

        //TORSO
        L_Shift_XYZ = Vector3d(kicking_param_.L_Shift_X, kicking_param_.L_Shift_Y, kicking_param_.L_Shift_Z);
        R_Shift_XYZ = Vector3d(kicking_param_.R_Shift_X , kicking_param_.R_Shift_Y, kicking_param_.R_Shift_Z);

        //LEG
        L_Lift_XYZ = Vector3d(kicking_param_.L_Lift_X, kicking_param_.L_Lift_Y, kicking_param_.L_Lift_Z);
        R_Lift_XYZ = Vector3d(kicking_param_.R_Lift_X, kicking_param_.R_Lift_Y, kicking_param_.R_Lift_Z);
        //TORSO
        L_Lift_RPY = Vector3d(kicking_param_.Lift_Roll * PI / 180, kicking_param_.Lift_Pitch * PI / 180, 0.0);
        R_Lift_RPY = Vector3d(-kicking_param_.Lift_Roll * PI / 180, kicking_param_.Lift_Pitch * PI / 180, 0.0);

        //LEG
        L_Swing_XYZ = Vector3d(kicking_param_.L_Swing_X, kicking_param_.L_Swing_Y, kicking_param_.L_Swing_Z);
        R_Swing_XYZ = Vector3d(kicking_param_.R_Swing_X, kicking_param_.R_Swing_Y, kicking_param_.R_Swing_Z);
        //TORSO
        L_Swing_RPY = Vector3d(kicking_param_.Swing_Roll * PI / 180, kicking_param_.Swing_Pitch * PI / 180, 0.0);
        R_Swing_RPY = Vector3d(-kicking_param_.Swing_Roll * PI / 180, kicking_param_.Swing_Pitch * PI / 180, 0.0);

        //LEG
        L_Retract_XYZ = Vector3d(kicking_param_.L_Retract_X, kicking_param_.L_Retract_Y, kicking_param_.L_Retract_Z);
        R_Retract_XYZ = Vector3d(kicking_param_.R_Retract_X, kicking_param_.R_Retract_Y, kicking_param_.R_Retract_Z);
        //TORSO
        L_Retract_RPY = Vector3d(kicking_param_.Retract_Roll * PI / 180, kicking_param_.Retract_Pitch * PI / 180, 0.0);
        R_Retract_RPY = Vector3d(-kicking_param_.Retract_Roll * PI / 180, kicking_param_.Retract_Pitch * PI / 180, 0.0);

        Ankle_Shift_RPY = Vector3d(0.0, 0.0, 0.0);

        this->stop();
    }

    void Kicking::queueThread()
    {
        ros::CallbackQueue callback_queue;
        nh_.setCallbackQueue(&callback_queue);

        ros::Subscriber kicking_command_sub_ = nh_.subscribe("/alfarobi/kicking/command", 0, &Kicking::kickingCommandCallback, this);
        ros::Subscriber kicking_param_sub_ = nh_.subscribe("/alfarobi/kicking/kicking_set_params", 0, &Kicking::kickingParamCallback, this);
        ros::ServiceServer get_kicking_param_server = nh_.advertiseService("/alfarobi/kicking/kicking_get_params", &Kicking::getKickingParamCallback, this);

        ros::Subscriber imu_sub_ = nh_.subscribe("/arduino_controller/imu", 0, &Kicking::imuCallback, this);
        kick_status_pub_ = nh_.advertise<std_msgs::String>("/alfarobi/kicking/status", 0);

        ros::WallDuration duration(control_cycle_msec_ / 1000.0);
        while (nh_.ok())
            callback_queue.callAvailable(duration);
    }

    bool Kicking::isRunning()
    {
        return real_running_ || (kicking_state_ == KickingInitPose);
    }

    void Kicking::start()
    {
        if (!m_active)
        {
            m_kick_state = 0;
            m_active = true;
            m_start = true;
        }
    }

    void Kicking::stop()
    {
        ROS_INFO("STOP");
        if (m_active)
        {
            m_kick_state = 0;
            m_active = false;
            m_start = false;
        }
    }

    void Kicking::kickingCommandCallback(const std_msgs::String::ConstPtr &_msg)
    {
        if (_msg->data == "right_kick")
        {
            m_kick_mode = Right_Kick;
            start();
            ROS_INFO("RIGHT KICKING");
        }
        else if (_msg->data == "left_kick")
        {
            m_kick_mode = Left_Kick;
            start();
        }

        else if (_msg->data == "save")
            saveConfig(config_path_);
        else if (_msg->data == "stop")
            stop();
    }

    void Kicking::kickingParamCallback(const kicking_module_msgs::KickingParam::ConstPtr &msg)
    {
        kicking_param_ = *msg;
        //TORSO
        L_Shift_XYZ = Vector3d(kicking_param_.L_Shift_X, kicking_param_.L_Shift_Y, kicking_param_.L_Shift_Z);
        R_Shift_XYZ = Vector3d(kicking_param_.R_Shift_X , kicking_param_.R_Shift_Y, kicking_param_.R_Shift_Z);

        //LEG
        L_Lift_XYZ = Vector3d(kicking_param_.L_Lift_X, kicking_param_.L_Lift_Y, kicking_param_.L_Lift_Z);
        R_Lift_XYZ = Vector3d(kicking_param_.R_Lift_X, kicking_param_.R_Lift_Y, kicking_param_.R_Lift_Z);
        //TORSO
        L_Lift_RPY = Vector3d(kicking_param_.Lift_Roll * PI / 180, kicking_param_.Lift_Pitch * PI / 180, 0.0);
        R_Lift_RPY = Vector3d(-kicking_param_.Lift_Roll * PI / 180, kicking_param_.Lift_Pitch * PI / 180, 0.0);

        //LEG
        L_Swing_XYZ = Vector3d(kicking_param_.L_Swing_X, kicking_param_.L_Swing_Y, kicking_param_.L_Swing_Z);
        R_Swing_XYZ = Vector3d(kicking_param_.R_Swing_X, kicking_param_.R_Swing_Y, kicking_param_.R_Swing_Z);
        //TORSO
        L_Swing_RPY = Vector3d(kicking_param_.Swing_Roll * PI / 180, kicking_param_.Swing_Pitch * PI / 180, 0.0);
        R_Swing_RPY = Vector3d(-kicking_param_.Swing_Roll * PI / 180, kicking_param_.Swing_Pitch * PI / 180, 0.0);

        //LEG
        L_Retract_XYZ = Vector3d(kicking_param_.L_Retract_X, kicking_param_.L_Retract_Y, kicking_param_.L_Retract_Z);
        R_Retract_XYZ = Vector3d(kicking_param_.R_Retract_X, kicking_param_.R_Retract_Y, kicking_param_.R_Retract_Z);
        //TORSO
        L_Retract_RPY = Vector3d(kicking_param_.Retract_Roll * PI / 180, kicking_param_.Retract_Pitch * PI / 180, 0.0);
        R_Retract_RPY = Vector3d(-kicking_param_.Retract_Roll * PI / 180, kicking_param_.Retract_Pitch * PI / 180, 0.0);



        m_state_time[0] = kicking_param_.SHIFT_TIME ;
        m_state_time[1] = kicking_param_.LIFT_TIME;
        m_state_time[2] = kicking_param_.SWING_TIME ;
        m_state_time[3] = kicking_param_.RETRACT_TIME;
        m_state_time[4] = kicking_param_.LANDING_TIME;
        m_state_time[5] = kicking_param_.FINISHED_TIME;

        // kicking_state_ =KickingInitPose;
        //  int max_ = 0;
        //         init_pose_count_ = 0;
        //         m_time_start = 0;

        //         Torso.head<3>() = Vector3d(kicking_param_.Torso_X, kicking_param_.Torso_Y, kicking_param_.Torso_Z);
        //         Torso.tail<3>() = Vector3d(0.0, kicking_param_.Torso_Pitch * PI / 180, 0.0);

        //         //Init Pose Kick

        //         L_Leg << 0, 0.065, 0, 0, 0, 0;
        //         R_Leg << 0, -0.065, 0, 0, 0, 0;

        //         getJoints.head<12>() = inverse_legs(L_Leg, R_Leg, Torso);

        //         for (int idx = 0; idx < 12; idx++)
        //             max_ = std::max((int)fabs(current_joint_pos(idx) - getJoints(idx)), max_);

        //         double mov_time = (max_ / 30) < 1.5 ? 1.5 : (max_ / 30);
        //         double smp_time = control_cycle_msec_ * 0.001;
        //         int all_time_steps = int(mov_time / smp_time) + 1;

        //         calc_joint_tra_.resize(all_time_steps, 12);

        //         for (int idx = 0; idx < 12; idx++)
        //             calc_joint_tra_.block(0, idx, all_time_steps, 1) = robotis_framework::calcMinimumJerkTra(current_joint_pos(idx), 0.0, 0.0,
        //                                                                                                      getJoints(idx), 0.0, 0.0,
        //                                                                                                      smp_time, mov_time);
        //         for (int idx = 1; idx < 12; idx++)
        //         getJoints(idx) = calc_joint_tra_(init_pose_count_, idx);

        //         init_pose_count_ += 1;

        //         if (init_pose_count_ >= calc_joint_tra_.rows())
        //         {
        //             kicking_state_ = KickingReady;
        //             firstTimeExc = ros::Time::now().toSec();
        //         }

        //         uTorso = Torso;
        //         uL_Leg = L_Leg;
        //         uR_Leg = R_Leg;
        // setValue();
    }

    bool Kicking::getKickingParamCallback(kicking_module_msgs::GetKickingParam::Request &req, kicking_module_msgs::GetKickingParam::Response &res)
    {
        res.parameters = kicking_param_;
        return true;
    }

    void Kicking::imuCallback(const sensor_msgs::ImuConstPtr &msg)
    {
        imu(0) = msg->angular_velocity.x;
        imu(1) = msg->angular_velocity.y;
        imu(2) = msg->angular_velocity.z;

        imu_orientation.x() = msg->orientation.x;
        imu_orientation.y() = msg->orientation.y;
        imu_orientation.z() = msg->orientation.z;
        imu_orientation.w() = msg->orientation.w;

        m_imuGyr = imu;

        m_imuOri = robotis_framework::convertQuaternionToRPY(imu_orientation);
        m_imuAcc.x() = msg->linear_acceleration.x;
        m_imuAcc.y() = msg->linear_acceleration.y;
        m_imuAcc.z() = msg->linear_acceleration.z;
    }

    void Kicking::kickStatus(const std::string &command)
    {
        std_msgs::String _commnd_msg;
        _commnd_msg.data = command;
        kick_status_pub_.publish(_commnd_msg);
    }

    void Kicking::loadConfig()
    {
        YAML::Node offset_node;
        YAML::Node config_node;

        try
        {
            offset_node = YAML::LoadFile(offset_path_.c_str());
            config_node = YAML::LoadFile(config_path_.c_str());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Fail to load yaml file.");
            return;
        }

        YAML::Node offset_ = offset_node["offset"];

        for (int idx = 0; idx < 12; idx++)
        {
            offset[idx] = offset_[joint[idx]].as<double>();
        }

        for (int idx = 1; idx < 21; idx++)
        {
            op3_offset[idx] = offset_[op3_joint[idx]].as<double>();
        }

        ROS_INFO("Load yaml file KICKING");
        YAML::Node config_ = config_node["kicking_v10"];

        // iTorso_X = config_["Torso_X"].as<double>();
        // iTorso_Y = config_["Torso_Y"].as<double>();
        // iTorso_Z = config_["Torso_Z"].as<double>();

        // L_Shift_X = config_["L_Shift_X"].as<double>();
        // L_Shift_Y = config_["L_Shift_Y"].as<double>();
        // L_Shift_Z = config_["L_Shift_Z"].as<double>();
        // //----------------------------------------------//
        // R_Shift_X = config_["R_Shift_X"].as<double>();
        // R_Shift_Y = config_["R_Shift_Y"].as<double>();
        // R_Shift_Z = config_["R_Shift_Z"].as<double>();

        // L_Lift_X = config_["L_Lift_X"].as<double>();
        // L_Lift_Y = config_["L_Lift_Y"].as<double>();
        // L_Lift_Z = config_["L_Lift_Z"].as<double>();
        // //--------------------------------------------//
        // R_Lift_X = config_["R_Lift_X"].as<double>();
        // R_Lift_Y = config_["R_Lift_Y"].as<double>();
        // R_Lift_Z = config_["R_Lift_Z"].as<double>();

        // L_Swing_X = config_["L_Swing_X"].as<double>();
        // L_Swing_Y = config_["L_Swing_Y"].as<double>();
        // L_Swing_Z = config_["L_Swing_Z"].as<double>();
        // //---------------------------------------------//
        // R_Swing_X = config_["R_Swing_X"].as<double>();
        // R_Swing_Y = config_["R_Swing_Y"].as<double>();
        // R_Swing_Z = config_["R_Swing_Z"].as<double>();

        // L_Retract_X = config_["L_Retract_X"].as<double>();
        // L_Retract_Y = config_["L_Retract_Y"].as<double>();
        // L_Retract_Z = config_["L_Retract_Z"].as<double>();
        // //---------------------------------------------//
        // R_Retract_X = config_["R_Retract_X"].as<double>();
        // R_Retract_Y = config_["R_Retract_Y"].as<double>();
        // R_Retract_Z = config_["R_Retract_Z"].as<double>();

        // iTorso_Pitch = config_["Torso_Pitch"].as<double>();

        // Shift_Roll = config_["Shift_Roll"].as<double>();

        // Lift_Roll = config_["Lift_Roll"].as<double>();
        // Lift_Pitch = config_["Lift_Pitch"].as<double>();

        // Swing_Roll = config_["Swing_Roll"].as<double>();
        // Swing_Pitch = config_["Swing_Pitch"].as<double>();

        // Retract_Roll = config_["Retract_Roll"].as<double>();
        // Retract_Pitch = config_["Retract_Pitch"].as<double>();

        m_state_time[0] = config_["SHIFT_TIME"].as<double>();
        m_state_time[1] = config_["LIFT_TIME"].as<double>();
        m_state_time[2] = config_["SWING_TIME"].as<double>();
        m_state_time[3] = config_["RETRACT_TIME"].as<double>();
        m_state_time[4] = config_["LANDING_TIME"].as<double>();
        m_state_time[5] = config_["FINISHED_TIME"].as<double>();

        kicking_param_.Torso_X       = config_["Torso_X"].as<double>();
        kicking_param_.Torso_Y       = config_["Torso_Y"].as<double>();
        kicking_param_.Torso_Z       = config_["Torso_Z"].as<double>();
        kicking_param_.L_Shift_X     = config_["L_Shift_X"].as<double>(); 
        kicking_param_.L_Shift_Y     = config_["L_Shift_Y"].as<double>();
        kicking_param_.L_Shift_Z     = config_["L_Shift_Z"].as<double>();
        kicking_param_.R_Shift_X     = config_["R_Shift_X"].as<double>();
        kicking_param_.R_Shift_Y     = config_["R_Shift_Y"].as<double>();
        kicking_param_.R_Shift_Z     = config_["R_Shift_Z"].as<double>();
        kicking_param_.L_Lift_X      = config_["L_Lift_X"].as<double>();
        kicking_param_.L_Lift_Y      = config_["L_Lift_Y"].as<double>();
        kicking_param_.L_Lift_Z      = config_["L_Lift_Z"].as<double>();
        kicking_param_.R_Lift_X      = config_["R_Lift_X"].as<double>();
        kicking_param_.R_Lift_Y      = config_["R_Lift_Y"].as<double>();
        kicking_param_.R_Lift_Z      = config_["R_Lift_Z"].as<double>();
        kicking_param_.L_Swing_X     = config_["L_Swing_X"].as<double>();
        kicking_param_.L_Swing_Y     = config_["L_Swing_Y"].as<double>();
        kicking_param_.L_Swing_Z     = config_["L_Swing_Z"].as<double>();
        kicking_param_.R_Swing_X     = config_["R_Swing_X"].as<double>();
        kicking_param_.R_Swing_Y     = config_["R_Swing_Y"].as<double>();
        kicking_param_.R_Swing_Z     = config_["R_Swing_Z"].as<double>();
        kicking_param_.L_Retract_X   = config_["L_Retract_X"].as<double>();
        kicking_param_.L_Retract_Y   = config_["L_Retract_Y"].as<double>();
        kicking_param_.L_Retract_Z   = config_["L_Retract_Z"].as<double>();
        kicking_param_.R_Retract_X   = config_["R_Retract_X"].as<double>();
        kicking_param_.R_Retract_Y   = config_["R_Retract_Y"].as<double>();
        kicking_param_.R_Retract_Z   = config_["R_Retract_Z"].as<double>();
        kicking_param_.Torso_Pitch   = config_["Torso_Pitch"].as<double>();
        kicking_param_.Shift_Roll    = config_["Shift_Roll"].as<double>();
        kicking_param_.Lift_Roll     = config_["Lift_Roll"].as<double>();
        kicking_param_.Lift_Pitch    = config_["Lift_Pitch"].as<double>(); 
        kicking_param_.Swing_Roll    = config_["Swing_Roll"].as<double>();
        kicking_param_.Swing_Pitch   = config_["Swing_Pitch"].as<double>(); 
        kicking_param_.Retract_Roll  = config_["Retract_Roll"].as<double>();
        kicking_param_.Retract_Pitch = config_["Retract_Pitch"].as<double>(); 
        kicking_param_.SHIFT_TIME    =  config_["SHIFT_TIME"].as<double>(); 
        kicking_param_.LIFT_TIME     = config_["LIFT_TIME"].as<double>();
        kicking_param_.SWING_TIME    = config_["SWING_TIME"].as<double>();
        kicking_param_.RETRACT_TIME  = config_["RETRACT_TIME"].as<double>();
        kicking_param_.LANDING_TIME  = config_["LANDING_TIME"].as<double>();
        kicking_param_.FINISHED_TIME = config_["FINISHED_TIME"].as<double>();





    }

    void Kicking::saveConfig(const std::string &path)
    {
        YAML::Node config_node;

        try
        {
            config_node = YAML::LoadFile(config_path_.c_str());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Fail to load yaml file.");
            return;
        }

        YAML::Node config_ = config_node["kicking_v10"];

        // config_["Body_Height"] = kicking_param_.Body_Height;
        // config_["Body_Tilt"] = kicking_param_.Body_Tilt;
        // config_["Leg_X"] = kicking_param_.Leg_X;
        // config_["Leg_Y"] = kicking_param_.Leg_Y;
        // config_["Leg_Z"] = kicking_param_.Leg_Z;
        // config_["X_Retract"] = kicking_param_.X_Retract;
        // config_["Z_Retract"] = kicking_param_.Z_Retract;

        // config_["Leg_R_X"] = kicking_param_.Leg_R_X;
        // config_["R_Shift"] = kicking_param_.R_Shift;
        // config_["R_Lift"] = kicking_param_.R_Lift;
        // config_["R_CoolDown"] = kicking_param_.R_CoolDown;
        // config_["R_Roll"] = kicking_param_.R_Roll;

        // config_["Leg_L_X"] = kicking_param_.Leg_L_X;
        // config_["L_Shift"] = kicking_param_.L_Shift;
        // config_["L_Lift"] = kicking_param_.L_Lift;
        // config_["L_CoolDown"] = kicking_param_.L_CoolDown;
        // config_["L_Roll"] = kicking_param_.L_Roll;

        // config_["Time_Shift"] = kicking_param_.Time_Shift;
        // config_["Time_Lift"] = kicking_param_.Time_Lift;
        // config_["Time_Retract"] = kicking_param_.Time_Retract;
        // config_["Time_Kick"] = kicking_param_.Time_Kick;
        // config_["Time_CoolDown"] = kicking_param_.Time_CoolDown;
        // config_["Time_BringDown"] = kicking_param_.Time_BringDown;

        // config_["Gain_Angle_Pitch"] = kicking_param_.Gain_Angle_Pitch;
        // config_["Gain_Velocity_Pitch"] = kicking_param_.Gain_Velocity_Pitch;
        // config_["Gain_Angle_Roll"] = kicking_param_.Gain_Angle_Roll;
        // config_["Gain_Velocity_Roll"] = kicking_param_.Gain_Velocity_Roll;
        // config_["Gain_Integral"] = kicking_param_.Gain_Integral;
        // output to file

        config_["Torso_X"]       = kicking_param_.Torso_X      ;
        config_["Torso_Y"]       = kicking_param_.Torso_Y      ;
        config_["Torso_Z"]       = kicking_param_.Torso_Z      ;
        config_["L_Shift_X"]     = kicking_param_.L_Shift_X    ; 
        config_["L_Shift_Y"]     = kicking_param_.L_Shift_Y    ;
        config_["L_Shift_Z"]     = kicking_param_.L_Shift_Z    ;
        config_["R_Shift_X"]     = kicking_param_.R_Shift_X    ;
        config_["R_Shift_Y"]     = kicking_param_.R_Shift_Y    ;
        config_["R_Shift_Z"]     = kicking_param_.R_Shift_Z    ;
        config_["L_Lift_X"]      = kicking_param_.L_Lift_X     ;
        config_["L_Lift_Y"]      = kicking_param_.L_Lift_Y     ;
        config_["L_Lift_Z"]      = kicking_param_.L_Lift_Z     ;
        config_["R_Lift_X"]      = kicking_param_.R_Lift_X     ;
        config_["R_Lift_Y"]      = kicking_param_.R_Lift_Y     ;
        config_["R_Lift_Z"]      = kicking_param_.R_Lift_Z     ;
        config_["L_Swing_X"]     = kicking_param_.L_Swing_X    ;
        config_["L_Swing_Y"]     = kicking_param_.L_Swing_Y    ;
        config_["L_Swing_Z"]     = kicking_param_.L_Swing_Z    ;
        config_["R_Swing_X"]     = kicking_param_.R_Swing_X    ;
        config_["R_Swing_Y"]     = kicking_param_.R_Swing_Y    ;
        config_["R_Swing_Z"]     = kicking_param_.R_Swing_Z    ;
        config_["L_Retract_X"]   = kicking_param_.L_Retract_X  ;
        config_["L_Retract_Y"]   = kicking_param_.L_Retract_Y  ;
        config_["L_Retract_Z"]   = kicking_param_.L_Retract_Z  ;
        config_["R_Retract_X"]   = kicking_param_.R_Retract_X  ;
        config_["R_Retract_Y"]   = kicking_param_.R_Retract_Y  ;
        config_["R_Retract_Z"]   = kicking_param_.R_Retract_Z  ;
        config_["Torso_Pitch"]   = kicking_param_.Torso_Pitch  ;
        config_["Shift_Roll"]    = kicking_param_.Shift_Roll   ;
        config_["Lift_Roll"]     = kicking_param_.Lift_Roll    ;
        config_["Lift_Pitch"]    = kicking_param_.Lift_Pitch   ; 
        config_["Swing_Roll"]    = kicking_param_.Swing_Roll   ;
        config_["Swing_Pitch"]   = kicking_param_.Swing_Pitch  ; 
        config_["Retract_Roll"]  = kicking_param_.Retract_Roll ;
        config_["Retract_Pitch"] = kicking_param_.Retract_Pitch; 
        config_["SHIFT_TIME"]    = kicking_param_.SHIFT_TIME   ; 
        config_["LIFT_TIME"]     = kicking_param_.LIFT_TIME    ;
        config_["SWING_TIME"]    = kicking_param_.SWING_TIME   ;
        config_["RETRACT_TIME"]  = kicking_param_.RETRACT_TIME ;
        config_["LANDING_TIME"]  = kicking_param_.LANDING_TIME ;
        config_["FINISHED_TIME"] = kicking_param_.FINISHED_TIME;

        std::ofstream fout(config_path_.c_str());
        fout << config_node;
    }

    void Kicking::onModuleEnable()
    {
        firstExc = true;
        kicking_state_ = KickingEnable;
        ROS_INFO("Kicking Enable");
    }

    void Kicking::onModuleDisable()
    {
        kicking_state_ = KickingDisable;
        ROS_INFO("Kicking Disable");
    }

    void Kicking::setValue()
    {
        double goal_position = 0.0;

        for (int idx = 0; idx < 18; idx++)
        {
            goal_position = (double)(init_position_.coeff(0, idx) + (getJoints(idx) * joint_axis_direction_(idx)));
            target_position_.coeffRef(0, idx) = goal_position;
        }

        // set result
        for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
             state_it != result_.end(); state_it++)
        {
            std::string joint_name = state_it->first;
            int joint_index = joint_table_[joint_name];

            result_[joint_name]->goal_position_ = target_position_.coeff(0, joint_index);
        }
    }

    void Kicking::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
    {
        if (enable_ == false)
            return;

        current_joint_pos = KDL::JntArray(12);

        joint_axis_direction_ << -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1;

        init_position_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        init_position_ *= DEGREE2RADIAN;

        // present angle
        for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
             state_iter != result_.end(); state_iter++)
        {
            std::string _joint_name = state_iter->first;
            int joint_index = joint_table_[_joint_name];

            robotis_framework::Dynamixel *dxl = NULL;
            std::map<std::string, robotis_framework::Dynamixel *>::iterator dxl_it = dxls.find(_joint_name);
            if (dxl_it != dxls.end())
                dxl = dxl_it->second;
            else
                continue;

            current_position_.coeffRef(0, joint_index) = dxl->dxl_state_->present_position_;
            uint data = dxl->dxl_state_->bulk_read_table_["hardware_error_status"];
            if (data != 0)
                ROS_ERROR("Device Error &s : &d", _joint_name, data);
            double valle = dxl->dxl_state_->present_position_;
        }

        for (int idx = 0; idx < 12; idx++)
        {
            current_joint_pos(idx) = joint_axis_direction_(idx) * (current_position_.coeffRef(0, idx) - offset[idx]);
        }

        forwardKinematic();
        estimateZMP();

        if (firstExc)
            kicking_state_ = KickingInitPose;

        if (kicking_state_ == KickingInitPose)
        {
            if (firstExc)
            {
                int max_ = 0;
                init_pose_count_ = 0;
                m_time_start = 0;

                Torso.head<3>() = Vector3d(kicking_param_.Torso_X, kicking_param_.Torso_Y, kicking_param_.Torso_Z);
                Torso.tail<3>() = Vector3d(0.0, kicking_param_.Torso_Pitch * PI / 180, 0.0);

                //Init Pose Kick

                L_Leg << 0, 0.065, 0, 0, 0, 0;
                R_Leg << 0, -0.065, 0, 0, 0, 0;

                getJoints.head<12>() = inverse_legs(L_Leg, R_Leg, Torso);

                for (int idx = 0; idx < 12; idx++)
                    max_ = std::max((int)fabs(current_joint_pos(idx) - getJoints(idx)), max_);

                double mov_time = (max_ / 30) < 1.5 ? 1.5 : (max_ / 30);
                double smp_time = control_cycle_msec_ * 0.001;
                int all_time_steps = int(mov_time / smp_time) + 1;

                calc_joint_tra_.resize(all_time_steps, 12);

                for (int idx = 0; idx < 12; idx++)
                    calc_joint_tra_.block(0, idx, all_time_steps, 1) = robotis_framework::calcMinimumJerkTra(current_joint_pos(idx), 0.0, 0.0,
                                                                                                             getJoints(idx), 0.0, 0.0,
                                                                                                             smp_time, mov_time);
            }
            firstExc = false;

            for (int idx = 1; idx < 12; idx++)
                getJoints(idx) = calc_joint_tra_(init_pose_count_, idx);

            init_pose_count_ += 1;

            if (init_pose_count_ >= calc_joint_tra_.rows())
            {
                kicking_state_ = KickingReady;
                firstTimeExc = ros::Time::now().toSec();
            }

            uTorso = Torso;
            uL_Leg = L_Leg;
            uR_Leg = R_Leg;
        }
        else if (kicking_state_ == KickingReady || kicking_state_ == KickingEnable)
        {
            timeExc = ros::Time::now().toSec() - firstTimeExc;

            if (m_active == true && timeExc > 2)
            {
                firstExc = false;
                kickStatus("kicking");

                // ROS_INFO("KICKING");

                if (m_start)
                {
                    m_time_start = ros::Time::now().toSec();
                    m_start = false;
                }

                m_time_now = ros::Time::now().toSec() - m_time_start;
                ROS_INFO("KICKING");
                
                if (phase >= 1)
                {
                    ROS_INFO("GANTI STATE");
                    m_time_start = ros::Time::now().toSec();
                    m_kick_state += 1;

                    phase = 0;

                    uTorso = Torso;
                    uL_Leg = L_Leg;
                    uR_Leg = R_Leg;
                }
                else
                {
                    if (m_kick_state != 6)
                        phase = m_time_now / m_state_time[m_kick_state];
                }

                ROS_INFO("%d", m_kick_state);

                switch (m_kick_state)
                {

                case Kick_Start_Shift:

                    ROS_INFO("SHIFT");

                    if (m_kick_mode == Right_Kick)
                    {
                        Torso.head<3>() = bezierCurve2(phase, uTorso.head<3>(), R_Shift_XYZ);
                        Torso.tail<3>() = bezierCurve2(phase, uTorso.tail<3>(), Vector3d(-kicking_param_.Shift_Roll * DEGREE2RADIAN, 0, 0));
                        R_Leg.tail<3>() = bezierCurve2(phase, uR_Leg.tail<3>(), Ankle_Shift_RPY); //harus nambah seberapa?
                    }
                    else
                    {
                        Torso.head<3>() = bezierCurve2(phase, uTorso.head<3>(), L_Shift_XYZ);
                        Torso.tail<3>() = bezierCurve2(phase, uTorso.tail<3>(), Vector3d(kicking_param_.Shift_Roll * DEGREE2RADIAN, 0, 0));
                        L_Leg.tail<3>() = bezierCurve2(phase, uL_Leg.tail<3>(), Ankle_Shift_RPY); //harus nambah seberapa?
                    }

                    break;

                case Kick_Lift:

                    ROS_INFO("LIFT");

                    if (m_kick_mode == Right_Kick)
                    {
                        Torso.tail<3>() = bezierCurve2(phase, uTorso.tail<3>(), R_Lift_RPY);
                        R_Leg.head<3>() = bezierCurve2(phase, uR_Leg.head<3>(), R_Lift_XYZ);
                        R_Leg.tail<3>() = bezierCurve2(phase, uR_Leg.tail<3>(), Vector3d(0,0*DEGREE2RADIAN, 0));
                    }
                    else
                    {
                        Torso.tail<3>() = bezierCurve2(phase, uTorso.tail<3>(), L_Lift_RPY);
                        L_Leg.head<3>() = bezierCurve2(phase, uL_Leg.head<3>(), L_Lift_XYZ);
                        L_Leg.tail<3>() = bezierCurve2(phase, uL_Leg.tail<3>(), Vector3d(0,0*DEGREE2RADIAN, 0));
                    }
                    break;

                case Kick_Swing:

                    ROS_INFO("SWING");

                    if (m_kick_mode == Right_Kick)
                    {
                        Torso.tail<3>() = bezierCurve2(phase, uTorso.tail<3>(), R_Swing_RPY);
                        R_Leg.head<3>() = bezierCurve3(phase, uR_Leg.head<3>(), Vector3d(0, -0.07, 0.09), R_Swing_XYZ);
                        R_Leg.tail<3>() = bezierCurve2(phase, uR_Leg.tail<3>(), Vector3d(0, 0, 0));
                    }
                    else
                    {
                        Torso.tail<3>() = bezierCurve2(phase, uTorso.tail<3>(), L_Swing_RPY);
                        L_Leg.head<3>() = bezierCurve3(phase, uL_Leg.head<3>(), Vector3d(0, 0.07, 0.09), L_Swing_XYZ);
                        L_Leg.tail<3>() = bezierCurve2(phase, uL_Leg.tail<3>(), Vector3d(0, 0, 0));
                    }
                    break;

                case Kick_Retract:

                    ROS_INFO("RETRACT");

                    if (m_kick_mode == Right_Kick)
                    {
                        Torso.tail<3>() = bezierCurve2(phase, uTorso.tail<3>(), R_Retract_RPY);
                        R_Leg.head<3>() = bezierCurve2(phase, uR_Leg.head<3>(), R_Retract_XYZ);
                        R_Leg.tail<3>() = bezierCurve2(phase, uR_Leg.tail<3>(), Ankle_Shift_RPY); //harus nambah seberapa?
                    }
                    else
                    {
                        Torso.tail<3>() = bezierCurve2(phase, uTorso.tail<3>(), L_Retract_RPY);
                        L_Leg.head<3>() = bezierCurve2(phase, uL_Leg.head<3>(), L_Retract_XYZ);
                        L_Leg.tail<3>() = bezierCurve2(phase, uL_Leg.tail<3>(), Ankle_Shift_RPY); //harus nambah seberapa?
                    }
                    break;

                case Kick_Landing:

                    ROS_INFO("LANDING");

                    Torso.head<3>() = bezierCurve2(phase, uTorso.head<3>(), Vector3d(kicking_param_.Torso_X, kicking_param_.Torso_Y,  kicking_param_.Torso_Z));
                    Torso.tail<3>() = bezierCurve2(phase, uTorso.tail<3>(), Vector3d(0.0, kicking_param_.Torso_Pitch * PI / 180, 0.0));

                    if (m_kick_mode == Right_Kick)
                    {
                        R_Leg.head<3>() = bezierCurve2(phase, uR_Leg.head<3>(), Vector3d(0, -0.07, 0));
                    }
                    else
                    {
                        L_Leg.head<3>() = bezierCurve2(phase, uL_Leg.head<3>(), Vector3d(0, 0.07, 0));
                    }
                    break;

                case Kick_Finish_Shift:

                    ROS_INFO("FINISHED");

                    Torso.head<3>() = bezierCurve2(phase, uTorso.head<3>(), Vector3d(kicking_param_.Torso_X, kicking_param_.Torso_Y,  kicking_param_.Torso_Z));
                    Torso.tail<3>() = bezierCurve2(phase, uTorso.tail<3>(), Vector3d(0.0, kicking_param_.Torso_Pitch * PI / 180, 0.0));

                    break;

                case Kick_Finished:

                    m_time_start = 0;
                    m_kick_state = 0;
                    kickStatus("finished_kick");
                    stop();
                    break;
                }
            }

            getJoints.head<12>() = inverse_legs(L_Leg, R_Leg, Torso);
            legStabilization();
            motion_arms();
        }
        setValue();
    }

    void Kicking::legStabilization()
    {
        // gyroFactor=0.175;
        // gyroFactor=0.225;
        gyroFactor=0.225;
        static double ankleImuParamRoll[4] = {0.6, 0.2 * gyroFactor, 0.0, 15 * DEGREE2RADIAN};
        static double ankleImuParamPitch[4] = {0.9, 0.2 * gyroFactor, 0.0, 15 * DEGREE2RADIAN};
        static double kneeImuParamPitch[4] = {0.6, 0.7 * gyroFactor, 0.0, 15 * DEGREE2RADIAN};
        static double hipImuParamPitch[4] = {0.9, 0.3 * gyroFactor, 0.0, 15 * DEGREE2RADIAN};

        ankleShift_roll = procFunc(m_imuGyr(0) * ankleImuParamRoll[1], ankleImuParamRoll[2], ankleImuParamRoll[3]);
        ankleShift_pitch = procFunc(m_imuGyr(1) * ankleImuParamPitch[1], ankleImuParamPitch[2], ankleImuParamPitch[3]);
        kneeShift_pitch = procFunc(m_imuGyr(1) * kneeImuParamPitch[1], kneeImuParamPitch[2], kneeImuParamPitch[3]);
        hipShift_pitch = procFunc(m_imuGyr(1) * hipImuParamPitch[1], hipImuParamPitch[2], hipImuParamPitch[3]);

        ankleShift[0] = ankleShift[0] + ankleImuParamRoll[0] * (ankleShift_roll - ankleShift[0]);
        ankleShift[1] = ankleShift[1] + ankleImuParamPitch[1] * (ankleShift_pitch - ankleShift[1]);
        kneeShift = kneeShift + kneeImuParamPitch[1] * (kneeShift_pitch - kneeShift);
        hipShift = hipShift + hipImuParamPitch[1] * (hipShift_pitch - hipShift);

        if(kneeShift >= 1)
            kneeShift = 0;

        if(hipShift >= 1)
            hipShift = 0;

        if(ankleShift[0] >= 1)
            ankleShift[0] = 0;

        if(ankleShift[1] >= 1)
            ankleShift[1] = 0;


        if (m_kick_mode == Left_Kick)
        {
            getJoints(3) += kneeShift;     //knee
            getJoints(4) += ankleShift[1]; //ankle pitch
        }
        else
        {
            getJoints(9) += kneeShift;      //knee
            getJoints(10) += ankleShift[1]; //ankle pitch
        }

        getJoints(2) += hipShift;
        getJoints(8) += hipShift;

        getJoints(5) += ankleShift[0];
        getJoints(11) += ankleShift[0];

        ROS_INFO("ANKLE ROLL [%f]", ankleShift[0] * RADIAN2DEGREE);
        ROS_INFO("ANKLE PITCH [%f]", ankleShift[1] * RADIAN2DEGREE);
        ROS_INFO("KNEE [%f]", kneeShift * RADIAN2DEGREE);
        ROS_INFO("HIP[%f]", hipShift * RADIAN2DEGREE);
    }

    double Kicking::procFunc(double value, double deadband, double maxValue)
    {
        double result;

        if (value > 0)
            result = std::fmin(std::fmax(0, std::fabs(value) - deadband), maxValue);
        else
            result = -1 * std::fmin(std::fmax(0, std::fabs(value) - deadband), maxValue);

        return result;
    }

    void Kicking::estimateZMP(){
        static const double Length = 0.355;
        static const double gravity = 9.8;
        static const double mass = 4.7;
        static const double c = 0.0005;
        static const double k = 375;


        double y_zmp = (std::pow(Length, 2)/ gravity * m_imuAcc.y()) + (c * std::pow(Length, 2) * m_imuGyr.y() / (mass * gravity)) + (k * std::pow(Length, 2) * m_imuOri.y() / (mass * gravity)) + (Length* m_imuOri.y());
        
        ROS_INFO("Y ZMP [%f]", y_zmp - 0.28);
    }

    void Kicking::motion_arms()
    
    {
        getJoints(12) = 0;
        getJoints(13) = 0;//-35 * DEGREE2RADIAN;
        getJoints(14) = -120; //-120 * DEGREE2RADIAN;
        getJoints(15) = 0;
        getJoints(16) = 0;//-35 * DEGREE2RADIAN;
        getJoints(17) = -120;//-120 * DEGREE2RADIAN;
    }

    void Kicking::forwardKinematic()
    {
        R_HIP.linear() = Eigen::Matrix3d::Identity();
        R_KNEE.linear() = Eigen::Matrix3d::Identity();
        R_ANKLE.linear() = Eigen::Matrix3d::Identity();
        L_HIP.linear() = Eigen::Matrix3d::Identity();
        L_KNEE.linear() = Eigen::Matrix3d::Identity();
        L_ANKLE.linear() = Eigen::Matrix3d::Identity();

        R_HIP.rotate(AngleAxisd(current_joint_pos(0), Vector3d(0, 0, 1) * joint_axis_direction_(0)) *
                     AngleAxisd(current_joint_pos(1), Vector3d(1, 0, 0) * -joint_axis_direction_(1)) *
                     AngleAxisd(current_joint_pos(2), Vector3d(0, 1, 0) * joint_axis_direction_(2)));
        R_KNEE.rotate(AngleAxisd(current_joint_pos(3), Vector3d(0, 1, 0) * joint_axis_direction_(3)));
        R_ANKLE.rotate(AngleAxisd(current_joint_pos(4), Vector3d(0, 1, 0) * joint_axis_direction_(4)) *
                       AngleAxisd(current_joint_pos(5), Vector3d(1, 0, 0) * joint_axis_direction_(5)));
        L_HIP.rotate(AngleAxisd(current_joint_pos(6), Vector3d(0, 0, 1) * joint_axis_direction_(6)) *
                     AngleAxisd(current_joint_pos(7), Vector3d(1, 0, 0) * -joint_axis_direction_(7)) *
                     AngleAxisd(current_joint_pos(8), Vector3d(0, 1, 0) * joint_axis_direction_(8)));
        L_KNEE.rotate(AngleAxisd(current_joint_pos(9), Vector3d(0, 1, 0) * joint_axis_direction_(9)));
        L_ANKLE.rotate(AngleAxisd(current_joint_pos(10), Vector3d(0, 1, 0) * joint_axis_direction_(10)) *
                       AngleAxisd(current_joint_pos(11), Vector3d(1, 0, 0) * joint_axis_direction_(11)));

        m_R_FOOT = BASE * R_HIP * R_KNEE * R_ANKLE * R_FOOT;
        m_L_FOOT = BASE * L_HIP * L_KNEE * L_ANKLE * L_FOOT;

        R_FOOT_WORLD = m_R_FOOT.translation();
        L_FOOT_WORLD = m_L_FOOT.translation();

        L_FOOT_WORLD(0) *= -1;

        R_ROT_WORLD = robotis_framework::convertRotationToRPY(m_R_FOOT.linear());
        L_ROT_WORLD = robotis_framework::convertRotationToRPY(m_L_FOOT.linear());

        // std::cout<<"ForwardKinematic:"<<std::endl;
        // std::cout<<"L_X: "<<L_FOOT_WORLD(0)<<"\tR_X: "<<R_FOOT_WORLD(0)<<"\t||\tL_Y: "<<L_FOOT_WORLD(1)<<"\tR_Y: "<<R_FOOT_WORLD(1)<<"\t||\tL_Z: "<<L_FOOT_WORLD(2)<<"\tR_Z: "<<R_FOOT_WORLD(2)<<std::endl;

        ////std::cout<<"L_Z: "<<std::fixed << std::setprecision(4)<<L_FOOT_WORLD(2)<<"\tR_Z = "<<std::fixed << std::setprecision(4)<<R_FOOT_WORLD(2)<<std::endl;
        if (L_FOOT_WORLD(2) > R_FOOT_WORLD(2) && fabs(L_FOOT_WORLD(2) - R_FOOT_WORLD(2)) > 0.01)
        {
            IndexSupport = 2; //1; //SSP Kanan
        }
        else if (L_FOOT_WORLD(2) < R_FOOT_WORLD(2) && fabs(L_FOOT_WORLD(2) - R_FOOT_WORLD(2)) > 0.01)
        {
            IndexSupport = 1; //2; //SSP Kiri
        }
        else if (fabs(L_FOOT_WORLD(2) - R_FOOT_WORLD(2)) <= 0.01)
        {
            IndexSupport = 0;
        }
    }

    Eigen::Vector3d Kicking::bezierCurve2(double phase, Eigen::Vector3d start, Eigen::Vector3d end)
    {
        Eigen::Vector3d result;

        result(0) = (1 - phase) * start(0) + phase * end(0);
        result(1) = (1 - phase) * start(1) + phase * end(1);
        result(2) = (1 - phase) * start(2) + phase * end(2);

        return result;
    }

    Eigen::Vector3d Kicking::bezierCurve3(double phase, Eigen::Vector3d start, Eigen::Vector3d inter, Eigen::Vector3d end)
    {
        Eigen::Vector3d result;

        result(0) = std::pow((1 - phase), 2) * start(0) + 2 * (1 - phase) * phase * inter(0) + std::pow(phase, 2) * end(0);
        result(1) = std::pow((1 - phase), 2) * start(1) + 2 * (1 - phase) * phase * inter(1) + std::pow(phase, 2) * end(1);
        result(2) = std::pow((1 - phase), 2) * start(2) + 2 * (1 - phase) * phase * inter(2) + std::pow(phase, 2) * end(2);

        return result;
    }

} // namespace robotis_op