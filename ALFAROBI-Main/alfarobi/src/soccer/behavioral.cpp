/**
 *
 * PAN
 *
 *                     |  1-----
 *            30       |        -30
 *                     |  2-----------
 *        45           |              -40
 *                     |  3 -------------
 *    50               |                   50
 *                     |  4------------------
 * 60                  |                        -60
 *                     |
 *                     |
 *                     |
 * --------------------|-----------------------
 *
 *
 *
 *
 *
 *
 *
 *
 *
 **/

#include "alfarobi/behavioral.h"
#include "alfarobi/alfarobi.h"

namespace robotis_op
{

    Behavioral::Behavioral()
        : nh_(ros::this_node::getName())
    {
        kickingDelay = false;
        lastKick = false;
        friend_status = false;
        firstSearch = true;
        firstIn = false;
        firstSearch_homeV2 = false;
        dribble_condition = false;
        lookStatus = true;
        swap_score = false;
        firstGetScore = true;
        ANGLE_LIMIT_TOLERANCE = false;
        counterSize = 0;
        buttonStatus = 0;
        //anyar
        kanan = true;
        firstWait = false;

        ownScore = 0;
        rivalScore = 0;

        std::string default_path = ros::package::getPath("op3_gui_demo") + "/config/demo_config.yaml";
        std::string motion_path = ros::package::getPath("op3_manager") + "/config/GlobalConfig.yaml";
        std::string behavioral_ = ros::package::getPath("config") + "/Main/behavioral.yaml";
        std::string offset_ = ros::package::getPath("op3_manager") + "/config/offset.yaml";

        std::string path = nh_.param<std::string>("demo_config", default_path);
        std::string motion = nh_.param<std::string>("main_program", motion_path);

        nh_.param<bool>("penalty", penalty, false);

        loadConfig(behavioral_, offset_);
        parseMotionIndex(motion);
        parseJointNameFromYaml(path);

        boost::thread queue_thread_ = boost::thread(boost::bind(&Behavioral::queueThread, this));
    }

    Behavioral::~Behavioral()
    {
    }

    void Behavioral::queueThread()
    {
        //publisher
        module_control_pub_ = nh_.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
        motion_index_pub_ = nh_.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
        angle_yaw_pub_ = nh_.advertise<std_msgs::Float32>("/angle_yaw", 0);
        cp_pub_ = nh_.advertise<std_msgs::Bool>("/alfarobi/quintic_walk/set_capture", 0);

        // set_walking_command_pub_ = nh_.advertise<std_msgs::String>("/robotis/walking/zmp_command", 10);
        // set_walking_param_pub_ = nh_.advertise<zmp_walking_module_msgs::ZMPWalkingParam>("/robotis/walking/zmp_set_params", 10);

        // get_walking_param_client_ = nh_.serviceClient<zmp_walking_module_msgs::GetZMPParam>(
        //             "/robotis/walking/zmp_get_params");

        set_kicking_command_pub_ = nh_.advertise<std_msgs::String>("/alfarobi/kicking/command", 10);

        ball_proj = nh_.advertise<std_msgs::Float32>("/alfarobi_proj", 10);

        set_walking_command_pub_ = nh_.advertise<std_msgs::String>("/robotis/quintic_walk/command", 10);
        set_walking_param_pub_ = nh_.advertise<quintic_walk_msgs::WalkingParam>("/robotis/quintic_walk/set_params", 10);

        get_walking_param_client_ = nh_.serviceClient<quintic_walk_msgs::GetWalkingParam>("/robotis/quintic_walk/get_params");

        kick_status_pub_ = nh_.advertise<std_msgs::String>("/alfarobi/kicking/status", 0);

        is_running_client_ = nh_.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");

        comm_status_pub_ = nh_.advertise<std_msgs::Int8>("/alfarobi/comm/status", 0);

        button_sub_ = nh_.subscribe("/arduino_controller/button", 1, &Behavioral::buttonHandlerCallback, this);
        imu_sub_ = nh_.subscribe("arduino_controller/imu", 1, &Behavioral::imuCallback, this);
        angle_sub_ = nh_.subscribe("/alfarobi/angle", 1, &Behavioral::angleCallback, this);
        odometry_sub_ = nh_.subscribe("/alfarobi/odometry", 1, &Behavioral::odometryCallback, this);
        walking_status_sub_ = nh_.subscribe("/robotis/walking/status", 1, &Behavioral::walkingCallback, this);
        kicking_status_sub_ = nh_.subscribe("/alfarobi/kicking/status", 1, &Behavioral::kickingCallback, this);
        friend_status_sub_ = nh_.subscribe("/alfarobi/kicking/friend_status", 1, &Behavioral::friendCallback, this);
        game_controller_sub_ = nh_.subscribe("/alfarobi/game_controller", 1, &Behavioral::gameControllerCallback, this);
        current_joint_states_sub_ = nh_.subscribe("/robotis/present_joint_states", 1, &Behavioral::currentJointStatesCallback, this);
        robot_status_sub_ = nh_.subscribe("/alfarobi/robot/status", 1, &Behavioral::robotStatusCallback, this);
        // line_pos_sub_ = nh.subscribe("/v11_cat2_detector/line_pos_",1,&Behavioral::line_pos_callback,this);
        // robot_state_sub_ = nh_.subscribe("/v9_localization_node/robot_state",1,&Behavioral::RobotStateCb,this);

        // controller_sub_ = nh_.subscribe("/alfarobi/controller_state", 1, &Behavioral::controllerCallback, this);

        ros::Rate rate(30);

        while (nh_.ok())
        {
            ros::spinOnce();

            rate.sleep();
        }
    }

    //KOMUNIKASI KRSBI REGIONAl 2022
    void Behavioral::robotStatusCallback(const std_msgs::Int8::ConstPtr& msg){
        m_robot_status = (short)msg->data;
    }

    void Behavioral::alfaStartPass(){
        std_msgs::Int8 msg;
        msg.data = 0;
        comm_status_pub_.publish(msg);
    }

    void Behavioral::abiWaiting(){
        std_msgs::Int8 msg;
        msg.data = 1;
        comm_status_pub_.publish(msg);
    }

    void Behavioral::resetFSM(){
        std_msgs::Int8 msg;
        msg.data = 2;
        comm_status_pub_.publish(msg);
    }



    void Behavioral::buttonHandlerCallback(const std_msgs::String::ConstPtr &msg)
    {
        std::string buttonCheck = msg->data;

        // if (buttonCheck == "L1")
        //     buttonStatus = Alfarobi::L1;
        // else if (buttonCheck == "L2")
        //     buttonStatus = Alfarobi::L2;
        // else if (buttonCheck == "L3")
        //     buttonStatus = Alfarobi::L3;
        // else if (buttonCheck == "L4")
        //     buttonStatus = Alfarobi::L4;
        // else if (buttonCheck == "R1")
        //     buttonStatus = Alfarobi::R1;f
        // else if (buttonCheck == "R2")
        //     buttonStatus = Alfarobi::R2;
        // else if (buttonCheck == "R3")
        //     buttonStatus = Alfarobi::R3;
        // else if (buttonCheck == "R4")
        //     buttonStatus = Alfarobi::R4;
        // else
        //     buttonStatus = Alfarobi::NoData;
        if(buttonCheck == "B1") {
            // activate_cp_.data = true;
            // cp_pub_.publish(activate_cp_);
            buttonStatus = Alfarobi::B1;
        }
        else if(buttonCheck == "B2") {
            // activate_cp_.data = false;
            // cp_pub_.publish(activate_cp_);
            buttonStatus = Alfarobi::B2;
        }
        else if(buttonCheck == "B3")
            buttonStatus = Alfarobi::B3;
        else if(buttonCheck == "B4")
            buttonStatus = Alfarobi::B4;
        else
            buttonStatus =  Alfarobi::NoData;
    }

    void Behavioral::controllerCallback(const std_msgs::UInt8 &msg)
    {
        char keyCheck = msg.data;

        if(keyCheck == 1)
            keyStatus = Alfarobi::W;
        else if(keyCheck == 2)
            keyStatus = Alfarobi::A;
        else if(keyCheck == 8)
            keyStatus = Alfarobi::S;
        else if(keyCheck == 3)
            keyStatus = Alfarobi::D;
        else if(keyCheck == 4)
            keyStatus = Alfarobi::Q;
        else if(keyCheck == 5)
            keyStatus = Alfarobi::E;
        else if(keyCheck == 6)
            keyStatus = Alfarobi::Z;
        else if(keyCheck == 7)
            keyStatus = Alfarobi::C;
        else
            keyStatus =  Alfarobi::NoData;
    }

    void Behavioral::angleCallback(const geometry_msgs::PointConstPtr &_msg)
    {
        angle_roll = (_msg->x) * 180 / PI;
        angle_pitch = (_msg->y) * 180 / PI;
        double current_pos = ((_msg->z) * 180 / PI) > 0 ? ((_msg->z) * 180 / PI) : 360 + ((_msg->z) * 180 / PI);
        ;

        if (current_pos >= 360)
            current_pos -= 360;
        else if (current_pos < 0)
            current_pos += 360;

        angle_yaw = current_pos <= 180 ? -current_pos : 360 - current_pos;
        if (penalty)
            angle_yaw = 0;

        m_yaw = angle_yaw;

        // ROS_INFO("YAW [%f]",angle_yaw);
    }

    void Behavioral::odometryCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        odometry_.x = (msg->pose.position.x * 100) + 700;
        // odometry_.y = (-1*msg->pose.position.y * 100) + 300;
        odometry_.y = (-1*msg->pose.position.y * 100);
        odometry_buffer_.push_back(odometry_);
    }

    void Behavioral::currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        double pan, tilt;
        int get_count = 0;

        for (int ix = 0; ix < msg->name.size(); ix++)
        {
            if (msg->name[ix] == "head_pan")
            {
                head_.x = (msg->position[ix] - offsetHead.x) * 180 / PI;
                get_count += 1;
            }
            else if (msg->name[ix] == "head_tilt")
            {
                head_.y = (msg->position[ix] - offsetHead.y) * 180 / PI;
                get_count += 1;
            }

            if (get_count == 2)
                break;
        }

        // head_buffer_.push_back(head_);
    }

    void Behavioral::kickingCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data == "kicking")
            kickingStatus = Kicking;
        else if (msg->data == "finished_kick")
        {
            // ROS_ERROR("<<==========FINISH KICK=========>>");
            finishedKick = true;
            kickingStatus = Finished_Kick;
            lastKick = false;
        }
        // else if(msg->data == "Not_Kicking")
        // {
        //     kickingStatus = Not_Kicking;
        // }
        // else if(msg->data == "kick woy")
        // {
        //     kicking = true;
        // }
    }

    void Behavioral::friendCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data == true)
            friend_status = true;
        else
            friend_status = false;
    }

    void Behavioral::walkingCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data == "enable")
            walkingStatus = true;
        else
            walkingStatus = false;
    }

    void Behavioral::gameControllerCallback(const game_controller::GameState::ConstPtr &msg)
    {
        comm_gameState = msg->gameState;
        comm_allowedToMove = msg->allowedToMove;
        comm_timeGame = msg->secondsRemaining;
        score_.x = msg->ownScore;
        score_.y = msg->rivalScore;

        score_buffer_.push_back(score_);
    }

    // void RobotStateCb(const geometry_msgs::PoseStampedConstPtr &msg)
    // {
    //     geometry_msgs::Pose2D robot_position;
    //     robot_position.x = msg->pose.position.x;
    //     // robot_position.pose.position.x = msg->pose.position.x;
    //     // robot_position.pose.position.y = msg->pose.position.y;
    //     // robot_position.theta = msg->pose.orientation.theta;
    // }    

    bool Behavioral::comm_scoreChanged()
    {
        Eigen::Vector2d score;
        bool idle = score_buffer_.size() == 0;
        bool scoreChanged;

        if (!idle)
        {
            score.x() = score_buffer_[0].x;
            score.y() = score_buffer_[0].y;
            score_buffer_.erase(score_buffer_.begin());
        }

        if ((ownScore != score.x() || rivalScore != score.y()) && swap_score == false && firstGetScore == false)
        {
            scoreChanged = true;
            swap_score = true;
        }
        else
            scoreChanged = false;

        firstGetScore = false;
        ownScore = score.x();
        rivalScore = score.y();

        return scoreChanged;
    }

    void Behavioral::parseJointNameFromYaml(const std::string &path)
    {
        YAML::Node doc;
        try
        {
            doc = YAML::LoadFile(path.c_str());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Fail to load id_joint table yaml.");
            return;
        }

        YAML::Node _id_sub_node = doc["id_joint"];
        for (YAML::iterator _it = _id_sub_node.begin(); _it != _id_sub_node.end(); ++_it)
        {
            int _id;
            std::string _joint_name;

            _id = _it->first.as<int>();
            _joint_name = _it->second.as<std::string>();

            id_joint_table_[_id] = _joint_name;
            joint_id_table_[_joint_name] = _id;
        }
    }

    void Behavioral::parseMotionIndex(const std::string &path)
    {
        YAML::Node doc;
        try
        {
            doc = YAML::LoadFile(path.c_str());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Fail to load id_joint table yaml.");
            return;
        }

        YAML::Node acc = doc["main_program"];

        Motion_Index.InitPose = acc["InitPose"].as<int>();
        Motion_Index.WalkingReady = acc["WalkingReady"].as<int>();
        Motion_Index.GetUpFront = acc["GetUpFront"].as<int>();
        Motion_Index.GetUpBack = acc["GetUpBack"].as<int>();
        Motion_Index.GetUpRight = acc["GetUpRight"].as<int>();
        Motion_Index.GetUpLeft = acc["GetUpLeft"].as<int>();
        Motion_Index.Ceremony = acc["Ceremony"].as<int>();
        Motion_Index.KeeperReady = acc["KeeperReady"].as<int>();
        Motion_Index.KeeperFront = acc["KeeperFront"].as<int>();
        Motion_Index.KeeperRight = acc["KeeperRight"].as<int>();
        Motion_Index.KeeperLeft = acc["KeeperLeft"].as<int>();
        Motion_Index.RightKick = acc["RightKick"].as<int>();
        Motion_Index.LeftKick = acc["LeftKick"].as<int>();
    }

    void Behavioral::loadConfig(const std::string &behavioral, const std::string &offset)
    {
        YAML::Node behavioral_section;
        YAML::Node offset_section;
        try
        {
            // load yaml
            behavioral_section = YAML::LoadFile(behavioral.c_str());
            offset_section = YAML::LoadFile(offset.c_str());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Fail to load yaml behavioral file.");
            return;
        }

        ROS_WARN("LOAD CONFIG BEHAVIORAL");

        //WALKING PARAM
        YAML::Node main_param = behavioral_section["main_program"];

        GAME_TIME = main_param["GAME_TIME"].as<int>();
        LOCALIZATION = main_param["LOCALIZATION"].as<bool>();
        COMMUNICATION = main_param["COMMUNICATION"].as<bool>();

        //WALKING PARAM
        YAML::Node walking_param = behavioral_section["walking"];

        MAX_FB_STEP = walking_param["MAX_FB_STEP"].as<double>();
        MIN_FB_STEP = walking_param["MIN_FB_STEP"].as<double>();
        MAX_BW_STEP = walking_param["MAX_BW_STEP"].as<double>();
        MIN_BW_STEP = walking_param["MIN_BW_STEP"].as<double>();
        MAX_RL_STEP = walking_param["MAX_RL_STEP"].as<double>();
        MIN_RL_STEP = walking_param["MIN_RL_STEP"].as<double>();
        MAX_RL_TURN = walking_param["MAX_RL_TURN"].as<double>();
        MIN_RL_TURN = walking_param["MIN_RL_TURN"].as<double>();
        REVO_CW_STEP = walking_param["REVO_CW_STEP"].as<double>();
        REVO_CW_TURN = walking_param["REVO_CW_TURN"].as<double>();
        REVO_CCW_STEP = walking_param["REVO_CCW_STEP"].as<double>();
        REVO_CCW_TURN = walking_param["REVO_CCW_TURN"].as<double>();
        UNIT_INCREMENT = walking_param["UNIT_INCREMENT"].as<double>();
        TRANTITION_BORDER = walking_param["TRANTITION_BORDER"].as<double>();

        //FALLEN LIMIT
        YAML::Node fallen_param = behavioral_section["fallen"];

        FRONT_FALL_LIMIT = fallen_param["FRONT_FALL_LIMIT"].as<double>();
        BEHIND_FALL_LIMIT = fallen_param["BEHIND_FALL_LIMIT"].as<double>();
        RIGHT_FALL_LIMIT = fallen_param["RIGHT_FALL_LIMIT"].as<double>();
        LEFT_FALL_LIMIT = fallen_param["LEFT_FALL_LIMIT"].as<double>();

        //FOLLOWING PARAM
        YAML::Node following_param = behavioral_section["following"];

        FORWARD_MIN_LIMIT = following_param["FORWARD_MIN_LIMIT"].as<double>();
        FORWARD_MIN_REVO = following_param["FORWARD_MIN_REVO"].as<double>();
        RL_MIN_LIMIT = following_param["RL_MIN_LIMIT"].as<double>();
        RL_MIN_REVO = following_param["RL_MIN_REVO"].as<double>();
        KICK_DELAY = following_param["KICK_DELAY"].as<double>();

        //KEEPER PARAM
        YAML::Node keeper_param = behavioral_section["keeper"];

        CENTER_MARGIN = keeper_param["CENTER_MARGIN"].as<double>();
        FALLEN_MARGIN = keeper_param["FALLEN_MARGIN"].as<double>();
        FOLLOWING_MARGIN = keeper_param["FOLLOWING_MARGIN"].as<double>();
        FRAME_NUMBER = keeper_param["FRAME_NUMBER"].as<double>();
        REGRESSION_NUMBER = keeper_param["REGRESSION_NUMBER"].as<double>();

        //KATEGORIPERTAMA PARAM
        YAML::Node kategoripertama_param = behavioral_section["kategoripertama"];

        X_MIN_LIMIT = kategoripertama_param["X_MIN_LIMIT"].as<double>();
        Y_RIGHT_LIMIT_FIRST = kategoripertama_param["Y_RIGHT_LIMIT_FIRST"].as<double>();
        Y_LEFT_LIMIT_FIRST = kategoripertama_param["Y_LEFT_LIMIT_FIRST"].as<double>();
        Y_RIGHT_LIMIT_SECOND = kategoripertama_param["Y_RIGHT_LIMIT_SECOND"].as<double>();
        Y_LEFT_LIMIT_SECOND = kategoripertama_param["Y_LEFT_LIMIT_SECOND"].as<double>();
        Y_RIGHT_LIMIT_THIRD = kategoripertama_param["Y_RIGHT_LIMIT_THIRD"].as<double>();
        Y_LEFT_LIMIT_THIRD = kategoripertama_param["Y_LEFT_LIMIT_THIRD"].as<double>();
        Y_RIGHT_LIMIT_FOURTH = kategoripertama_param["Y_RIGHT_LIMIT_FOURTH"].as<double>();
        Y_LEFT_LIMIT_FOURTH = kategoripertama_param["Y_LEFT_LIMIT_FOURTH"].as<double>();
        ANGLE_YAW_LIMIT = kategoripertama_param["ANGLE_YAW_LIMIT"].as<double>();

        //KEEPER PARAM
        YAML::Node angle_param = behavioral_section["angle"];

        KICKING_TOLERANCE = angle_param["KICKING_TOLERANCE"].as<double>();
        ANGLE_TOLERANCE = angle_param["ANGLE_TOLERANCE"].as<double>();
        BALL_TOLERANCE = angle_param["BALL_TOLERANCE"].as<double>();
        REVO_TOLERANCE = angle_param["REVO_TOLERANCE"].as<double>();
        BACKWARD_TOLERANCE = angle_param["BACKWARD_TOLERANCE"].as<double>();
        SHIFTING_TOLERANCE = angle_param["SHIFTING_TOLERANCE"].as<double>();

        YAML::Node prediction_param = behavioral_section["prediction_x"];
        p_pred_x[1] = prediction_param["p1"].as<double>();
        p_pred_x[2] = prediction_param["p2"].as<double>();
        p_pred_x[3] = prediction_param["p3"].as<double>();
        p_pred_x[4] = prediction_param["p4"].as<double>();
        p_pred_x[5] = prediction_param["p5"].as<double>();
        p_pred_x[6] = prediction_param["p6"].as<double>();

        YAML::Node avg_vel_param = behavioral_section["avgerage_velocity"];
        p_avg_vel[1] = avg_vel_param["a1"].as<double>();
        p_avg_vel[2] = avg_vel_param["a2"].as<double>();
        p_avg_vel[3] = avg_vel_param["a3"].as<double>();
        p_avg_vel[4] = avg_vel_param["a4"].as<double>();
        p_avg_vel[5] = avg_vel_param["a5"].as<double>();
        p_avg_vel[6] = avg_vel_param["a6"].as<double>();

        YAML::Node offset_param = offset_section["offset"];

        offsetHead.x = offset_param["head_pan"].as<double>();
        offsetHead.y = offset_param["head_tilt"].as<double>();

        ROS_WARN("LOAD CONFIG BEHAVIORAL DONE");
    }
    bool Behavioral::getWalkingParam()
    {
        quintic_walk_msgs::GetWalkingParam walking_param_msg;

        if (get_walking_param_client_.call(walking_param_msg))
        {
            current_walking_param_ = walking_param_msg.response.parameters;

            return true;
        }
        else
        {
            ROS_ERROR("Fail to get walking parameters.");

            return false;
        }
    }

    bool Behavioral::getJointNameFromID(const int &id, std::string &joint_name)
    {
        std::map<int, std::string>::iterator _iter;

        _iter = id_joint_table_.find(id);
        if (_iter == id_joint_table_.end())
            return false;

        joint_name = _iter->second;
        return true;
    }

    bool Behavioral::getIDFromJointName(const std::string &joint_name, int &id)
    {
        std::map<std::string, int>::iterator _iter;

        _iter = joint_id_table_.find(joint_name);
        if (_iter == joint_id_table_.end())
            return false;

        id = _iter->second;
        return true;
    }

    int Behavioral::getJointCount()
    {
        return joint_id_table_.size();
    }

    Eigen::Vector2d Behavioral::getOdometry()
    {
        Eigen::Vector2d odometry;
        bool idle = odometry_buffer_.size() == 0;

        if (!idle)
        {
            odometry.x() = odometry_buffer_[0].x * 100;
            odometry.y() = odometry_buffer_[0].y * 100;
            odometry_buffer_.erase(odometry_buffer_.begin());
        }

        return odometry;
    }

    Eigen::Vector2d Behavioral::getHead()
    {
        //     Eigen::Vector2d head;
        //     bool idle = head_buffer_.size() == 0;

        //     if(!idle){
        //         head.x() = (head_buffer_[0].x - offsetHead.x) * 180 / PI;
        //         head.y() = (head_buffer_[0].y - offsetHead.y) * 180 / PI;
            //         head_buffer_.erase(odometry_buffer_.begin());
            //     }

        //     return head;
    }


    void Behavioral::setModuleTo(const std::string &module_name)
    {
        robotis_controller_msgs::JointCtrlModule control_msg;
        std::map<int, std::string>::iterator joint_iter;

        for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
        {
            control_msg.joint_name.push_back(joint_iter->second);
            control_msg.module_name.push_back(module_name);
        }

        if (control_msg.joint_name.size() == 0)
            return;

        module_control_pub_.publish(control_msg);
    }

    void Behavioral::setBodyModuleTo(const std::string &body_module, bool with_head_control)
    {
        robotis_controller_msgs::JointCtrlModule control_msg;

        //ROS_WARN("Set Module to %s With Head Module %d", body_module.c_str(), with_head_control);
        std::string head_module = "head_control_module";
        std::map<int, std::string>::iterator joint_iter;

        for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
        {
            if (joint_iter->second.find("head") != std::string::npos)
            {
                if (with_head_control == true)
                {
                    ROS_WARN("Head is Enabled");
                    control_msg.joint_name.push_back(joint_iter->second);
                    control_msg.module_name.push_back(head_module);
                }
                else
                {
                    ROS_WARN("Head is Disabled");
                    continue;
                }
            }
            else
            {
                control_msg.joint_name.push_back(joint_iter->second);
                control_msg.module_name.push_back(body_module);
            }
        }

        if (control_msg.joint_name.size() == 0)
            return;

        module_control_pub_.publish(control_msg);
    }

    void Behavioral::setWalkingCommand(const std::string &command)
    {
        if (command == "start")
        {
            getWalkingParam();
            // setWalkingParam(0.43, 0.02, -0.785);
            // setWalkingParam(0.1, 0.05, -0.1); //romi
        }

        else if (command == "stop")
        {
            getWalkingParam();
            setWalkingParam(0,0,0); //kelas jul
        };

        std_msgs::String _command_msg;
        _command_msg.data = command;
        set_walking_command_pub_.publish(_command_msg);
    }

    void Behavioral::setWalkingParam(double x_move, double y_move, double rotation_angle)
    {
        // WalkZMP
        //     current_walking_param_.XMove = x_move/100;
        //     current_walking_param_.YMove = y_move/100;
        //     current_walking_param_.ZMove = rotation_angle;

        //Quintic
        current_walking_param_.XMove = x_move;
        current_walking_param_.YMove = y_move;
        current_walking_param_.ZMove = rotation_angle;

        // std::cout<<"FB : "<<x_move<<"\tRL : "<<y_move<<"\tTurn : "<<rotation_angle<<std::endl;
        if (!CALIBRATION)
            set_walking_param_pub_.publish(current_walking_param_);
    }

    void Behavioral::calcWalkingParam(double x_move, double y_move, double rotation_angle)
    {
        float delta_FB = fabs(FB_Step - x_move);
        float delta_RLS = fabs(RL_Step - y_move);
        float delta_RLT = fabs(RL_Turn - rotation_angle);
        float max_ = std::max(delta_RLS, delta_RLT);
        max_ = std::max(delta_FB, max_);
        float score = max_ / UNIT_INCREMENT;

        if (score != 0)
        {
            increment_fb = delta_FB / score;
            increment_rl = delta_RLS / score;
            increment_turn = delta_RLT / score;
        }
        else
            increment_fb = increment_rl = increment_turn = 0;

        //CALCULATION
        //FB MOVE
        if (x_move >= FB_Step)
        {
            if (x_move == 0)
                FB_Step = 0;
            else
                FB_Step += increment_fb * 2;
        }
        else if (x_move < FB_Step)
        {
            if (x_move == 0)
                FB_Step = 0;
            else
                FB_Step -= increment_fb * 2;
        }
        // FB_Step = x_move;
        //RL MOVE
        if (y_move >= RL_Step)
        {
            if (y_move == 0)
                RL_Step = 0;
            else
                RL_Step += increment_rl;
        }
        else if (y_move < RL_Step)
        {
            if (y_move == 0)
                RL_Step = 0;
            else
                RL_Step -= increment_rl;
        }
        // RL_Step = y_move;
        //RL TURN
        if (rotation_angle >= RL_Turn)
        {
            if (rotation_angle > RL_Turn)
                RL_Turn += increment_turn;
            else if (rotation_angle == 0)
                RL_Turn = 0;
        }
        else if (rotation_angle < RL_Turn)
        {
            if (rotation_angle < RL_Turn)
                RL_Turn -= increment_turn;
            else if (rotation_angle == 0)
                RL_Turn = 0;
        }
        // RL_Turn = rotation_angle;

        //CHECK VALUE
        //FB MOVE
        if (FB_Step > MAX_FB_STEP)
            FB_Step = MAX_FB_STEP;
        else if (FB_Step < MAX_BW_STEP)
            FB_Step = MAX_BW_STEP;
        // else if(x_move == 0)
        // FB_Step = 0;
        //RL MOVE
        if (RL_Step > MAX_RL_STEP)
            RL_Step = MAX_RL_STEP;
        else if (RL_Step < -MAX_RL_STEP)
            RL_Step = -MAX_RL_STEP;
        // else if(y_move == 0)
        // RL_Step = 0;

        //RL TURN
        if (RL_Turn > MAX_RL_TURN)
            RL_Turn = MAX_RL_TURN;
        else if (RL_Turn < -MAX_RL_TURN)
            RL_Turn = -MAX_RL_TURN;
        // else if(rotation_angle == 0)
        // RL_Turn = 0;

        setWalkingParam(FB_Step, RL_Step, RL_Turn);
    }


    void Behavioral::startFollowing()
    {
        //ROS_INFO("Start Ball following");
        if (!CALIBRATION)
            // startKicking("right_kick");
            // ros::Duration(5).sleep();
            setWalkingCommand("start");
    }

    void Behavioral::stopFollowing()
    {
        ROS_ERROR("Stop Ball following");

        setWalkingCommand("stop");
    }

    void Behavioral::startKicking(const std::string &position)
    {
        //ROS_INFO("Start Kicking");

        std_msgs::String _position;
        _position.data = position;

        if (!CALIBRATION)
            set_kicking_command_pub_.publish(_position);
    }

    void Behavioral::playMotion(int motion_index)
    {
        std_msgs::Int32 motion_msg;
        motion_msg.data = motion_index;

        motion_index_pub_.publish(motion_msg);
    }

    bool Behavioral::isHeadJoint(const int &id)
    {
        std::map<std::string, int>::iterator _iter;

        for (_iter = joint_id_table_.begin(); _iter != joint_id_table_.end(); ++_iter)
        {
            if (_iter->first.find("head") != std::string::npos)
                return true;
        }

        return false;
    }

    bool Behavioral::isActionRunning()
    {
        op3_action_module_msgs::IsRunning is_running_srv;

        if (is_running_client_.call(is_running_srv) == false)
        {
            //ROS_ERROR("Failed to get action status");
            return true;
        }
        else
        {
            if (is_running_srv.response.is_running == true)
                return true;
        }

        return false;
    }

    int Behavioral::checkFallen()
    {
        int fallenCondition = 999;

        if (angle_pitch < FRONT_FALL_LIMIT)
            fallenCondition = Fallen_Behind;
        else if (angle_roll > LEFT_FALL_LIMIT)
            fallenCondition = Fallen_Left;
        else if (angle_roll < RIGHT_FALL_LIMIT)
            fallenCondition = Fallen_Right;
        else if (angle_pitch > BEHIND_FALL_LIMIT)
            fallenCondition = Fallen_Forward;
        else
            fallenCondition = Nothing;
        // bool fallenCondition = false;

        // if(angle_pitch < FRONT_FALL_LIMIT || angle_roll > LEFT_FALL_LIMIT || angle_roll < RIGHT_FALL_LIMIT || angle_pitch > BEHIND_FALL_LIMIT)
        //     fallenCondition = true;
        
        return fallenCondition;
    }

    int Behavioral::robotZone(double robot_x, double robot_y)
    {
        int robot_zone = 999;

        if (robot_x >= 0 && robot_y < 0)
            robot_zone = Alfarobi::Zone1;
        else if (robot_x >= 0 && robot_y >= 0)
            robot_zone = Alfarobi::Zone2;
        else if (robot_x < 0 && robot_y >= 0)
            robot_zone = Alfarobi::Zone3;
        else if (robot_x < 0 && robot_y < 0)
            robot_zone = Alfarobi::Zone4;

        return robot_zone;
    }

    int Behavioral::destinationZone(int first_zone, int robot_zone)
    {
        switch (first_zone)
        {
        case Alfarobi::Zone1:
            switch (robot_zone)
            {
            case Alfarobi::Zone1:
                return Alfarobi::Zone2;
            case Alfarobi::Zone2:
                return Alfarobi::Zone4;
            case Alfarobi::Zone3:
                return Alfarobi::Zone1;
            case Alfarobi::Zone4:
                return Alfarobi::Zone3;
            }
            break;
        case Alfarobi::Zone2:
            switch (robot_zone)
            {
            case Alfarobi::Zone1:
                return Alfarobi::Zone3;
            case Alfarobi::Zone2:
                return Alfarobi::Zone1;
            case Alfarobi::Zone3:
                return Alfarobi::Zone4;
            case Alfarobi::Zone4:
                return Alfarobi::Zone2;
            }
            break;
        case Alfarobi::Zone3:
            switch (robot_zone)
            {
            case Alfarobi::Zone1:
                return Alfarobi::Zone3;
            case Alfarobi::Zone2:
                return Alfarobi::Zone1;
            case Alfarobi::Zone3:
                return Alfarobi::Zone4;
            case Alfarobi::Zone4:
                return Alfarobi::Zone2;
            }
            break;
        case Alfarobi::Zone4:
            switch (robot_zone)
            {
            case Alfarobi::Zone1:
                return Alfarobi::Zone2;
            case Alfarobi::Zone2:
                return Alfarobi::Zone4;
            case Alfarobi::Zone3:
                return Alfarobi::Zone1;
            case Alfarobi::Zone4:
                return Alfarobi::Zone3;
            }
            break;
        }
    }

    Eigen::Vector2d Behavioral::randomizedZone(int destination_zone, double border)
    {
        Eigen::Vector2d destination;

        int xZone = 450 - (2 * border);
        int yZone = 300 - (2 * border);

        std::default_random_engine gen;
        std::uniform_int_distribution<int> xRand(border, (xZone - border));
        std::uniform_int_distribution<int> yRand(border, (yZone - border));

        switch (destination_zone)
        {
        case Alfarobi::Zone1:
            destination.x() = 225; //xRand(gen);
            destination.y() = 150; //yRand(gen);
            break;
        case Alfarobi::Zone2:
            destination.x() = 225;  //xRand(gen);
            destination.y() = -150; //1 * (yRand(gen));
            break;
        case Alfarobi::Zone3:
            destination.x() = -225; //1 * (xRand(gen));
            destination.y() = -150; //1 * (yRand(gen));
            break;
        case Alfarobi::Zone4:
            destination.x() = -225; //1 * (xRand(gen));
            destination.y() = -150; //yRand(gen);
            break;
        }

        return destination;
    }

    Eigen::Vector2d Behavioral::calcRobotDiff(double robot_x, double robot_y, double robot_theta, double destination_x, double destination_y)
    {
        Eigen::Vector2d trajectory;

        double x_diff = destination_x - robot_x;
        double y_diff = destination_y - robot_y;

        trajectory.x() = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
        trajectory.y() = (atan2(y_diff, x_diff) * 180 / PI) + angle_yaw;

        return trajectory;
    }

    bool Behavioral::fallen(int fallen_status)
    {
        if (fallen_status == Stand || fallen_status == Nothing)
            return false;

        setBodyModuleTo("action_module", true);

        ros::Duration(0.1).sleep();

        switch (fallen_status)
        {
        case Fallen_Forward:
            // std::cout<<"^^^^^^^^^^^JATUH DEPANNNNNN^^^^^^^^^^^"<<std::endl;
            playMotion(Motion_Index.GetUpFront);
            break;

        case Fallen_Behind:
            // std::cout<<"___________JATUH BELAKANGGG___________"<<std::endl;
            playMotion(Motion_Index.GetUpBack);
            break;

        case Fallen_Right:
            // std::cout<<"JATUH KANAN>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;
            playMotion(Motion_Index.GetUpRight);
            break;

        case Fallen_Left:
            // std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<JATUH KIRI"<<std::endl;
            playMotion(Motion_Index.GetUpLeft);
            break;

        default:
            break;
        }

        return true;
    }

    void Behavioral::smartSearch(double robot_x, double robot_y, double robot_theta, double border)
    {
        double searchNow;
        bool positioning;
        static double dist_theta;

        if (firstSearch)
        {
            // orientation = robot_theta;
            firstSearch = false;
            orientation = angle_yaw;
            searchStart = ros::Time::now().toSec();
        }

        searchNow = ros::Time::now().toSec() - searchStart;

        switch (searchState)
        {
        case FirstZone:
            if (searchNow > 2.5)
                searchState = LookAround;
            break;
        case Destination:
            // std::cout<<"<<==========DESTINATION==========>>"<<std::endl;
            if (!firstSearch)
                robot_zone.x() = robotZone(robot_x, robot_y);
            firstSearch = true;
            lookStatus = true;
            robot_zone.y() = robotZone(robot_x, robot_y);
            robot_zone.z() = destinationZone(robot_zone.x(), robot_zone.y());

            zone_point = randomizedZone(robot_zone.z(), border);
            zone_dist = calcRobotDiff(robot_x, robot_y, angle_yaw, zone_point.x(), zone_point.y());
            dist_theta = zone_dist.y();

            mileage_xDist = 0;
            mileage_yDist = 0;

            searchState = AlignToZone;
            searchStart = ros::Time::now().toSec();
            break;
        case LookAround:
            // std::cout<<"<<==========LOOK AROUND==========>>"<<"\tANGLE YAW : "<<angle_yaw<<"\tORIENTATION : "<<orientation<<std::endl;
            if (lookStatus)
                orientation = angle_yaw;
            lookStatus = false;
            trackingMode = BallTracker::Nod;
            if (robotZone(robot_x, robot_y) == 1 || robotZone(robot_x, robot_y) == 4)
                setWalkingParam(0, 0, -MAX_RL_TURN);
            else
                setWalkingParam(0, 0, MAX_RL_TURN);

            if (searchNow > 3 && fabs(orientation - angle_yaw) < ANGLE_TOLERANCE)
                searchState = Destination;
            break;
        case AlignToZone:
            // std::cout<<"<<==========ALIGN TO ZONE==========>>"<<"\tANGLE YAW : "<<angle_yaw<<"\tORIENTATION : "<<orientation<<"\tANGLE : "<<zone_dist.y()<<std::endl;
            // std::cout<<"X ROB : "<<robot_x<<"\tY ROB : "<<robot_y<<"\tX DEST : "<<zone_point.x()<<"\tY DEST : "<<zone_point.y()<<"\tJARAK : "<<zone_dist.x()<<std::endl;
            trackingMode = BallTracker::Nod;
            if (robot_zone.y() == 1 || robot_zone.y() == 4)
                calcWalkingParam(0, 0, -MAX_RL_TURN);
            else
                calcWalkingParam(0, 0, MAX_RL_TURN);

            // std::cout<<"ANGLE YAW\t\t\t"<<angle_yaw<<"\t\t\tTHETA DIST\t\t\t"<<dist_theta<<std::endl;

            if (searchNow > 3 && fabs(angle_yaw - zone_dist.y()) < ANGLE_TOLERANCE){
            // if(searchNow > 3 && fabs(angle_yaw - dist_theta) < ANGLE_TOLERANCE)

                mileage_xDist = 0;
                mileage_yDist = 0;
                searchState = GoingToZone;
            }
            break;
        case GoingToZone:
            // std::cout<<"<<==========GO TO ZONE==========>>"<<"\tJARAK : "<<zone_dist.x()<<"\tMILEAGE :"<<mileage_xDist<<std::endl;
            robot_zone.y() = robotZone(robot_x, robot_y);
            calcWalkingParam(MAX_FB_STEP, 0, 0);
            if (robot_zone.y() == robot_zone.x())
                searchStart = ros::Time::now().toSec();
            if (searchNow > 6)
                searchState = Destination;
            break;
        }

        // std::cout<<"FIRST ZONE : "<<robot_zone.x()<<"\tRDEST ZONE : "<<robot_zone.z()<<"\tROBOT ZONE : "<<robot_zone.y()<<"\tREAL ZONE : "<<robotZone(robot_x, robot_y)<<std::endl;
    }

    void Behavioral::stupidSearch(double robot_x, double robot_y, double robot_theta, double border)
    {
        double searchNow;
        static int changeDirection;
        bool walk;

        if (firstSearch)
        {
            // orientation = robot_theta;
            firstSearch = false;
            searchState = Search_First;
            searchStart = ros::Time::now().toSec();
            // std::cout<<"<<==========STUPID SEARCH==========>>"<<std::endl;
        }

        // std::cout<<"ROBOT X : "<<robot_x<<"\t"<<"ROBOT Y : "<<robot_y<<std::endl;

        searchNow = ros::Time::now().toSec() - searchStart;

        switch (searchState)
        {
        case Search_First:
            // std::cout<<"<<==========FIRST==========>>"<<std::endl;
            trackingMode = BallTracker::Nod;

            mileage_yDist = 0;
            mileage_xDist = 0;

            if (fabs(robot_x) < (450 - border) && fabs(robot_y) < fabs(300 - border))
                searchState = Search_Forward;
            else
            {
                // searchStart = ros::Time::now().toSec();
                searchState = Search_Rotate;
                rotateState = Outer;
                orientation = angle_yaw;
            }
            break;

        case Search_Forward:
            // std::cout<<"<<==========FORWARD==========>>"<<std::endl;
            trackingMode = BallTracker::Sweep;
            calcWalkingParam(MAX_FB_STEP, 0, 0);
            // setWalkingParam(MAX_FB_STEP, 0, 0);
            walk = forward(200);

            if (walk == true && fabs(robot_x) < (450 - border) && fabs(robot_y) < (300 - border))
            {
                searchStart = ros::Time::now().toSec();
                searchState = Search_Rotate;
                rotateState = Inner;
            }
            else if (fabs(robot_x) >= (450 - border) || fabs(robot_y) >= (300 - border))
            {
                searchState = Search_Rotate;
                rotateState = Outer;
                orientation = angle_yaw;
            }
            break;

        case Search_Rotate:
            // std::cout<<"<<==========ROTATE==========Search>>"<<std::endl;
            trackingMode = BallTracker::Nod;
            mileage_xDist = 0;

            switch (rotateState)
            {
            case Inner:
                if (changeDirection % 2 == 0)
                    setWalkingParam(0, 0, MAX_RL_TURN);
                else
                    setWalkingParam(0, 0, -MAX_RL_TURN);

                if (searchNow > 30)
                {
                    searchStart = ros::Time::now().toSec();
                    searchState = Search_Forward;
                    changeDirection += 1;
                }
                break;
            case Outer:
                if (orientation >= 0)
                {
                    setWalkingParam(0, 0, MAX_RL_TURN);
                    if (searchNow > 4 && fabs(orientation) + fabs(angle_yaw) > 150)
                        searchState = Search_Forward;
                }
                else
                {
                    setWalkingParam(0, 0, -MAX_RL_TURN);
                    if (searchNow > 4 && fabs(orientation) + fabs(angle_yaw) < 150)
                        searchState = Search_Forward;
                }

                break;
            }
            break;
        }
    }

    void Behavioral::searchPillar(double robot_x, double robot_y, double robot_theta, double border)
    {
        double searchNow;
        static int changeDirectionP;

        if (firstSearch)
        {
            firstSearch = false;
            searchState = Search_Pillar;
            searchStart = ros::Time::now().toSec();
        }

        searchNow = ros::Time::now().toSec() - searchStart;
        switch (searchState)
        {
        case Search_Pillar:
            trackingMode = PillarTracker::Sweep;

            if (searchNow > 30)
            {
                if (kanan)
                {
                    for (int i = 0; i <= changeDirectionP; i++)
                        setWalkingParam(0, 0, -MAX_RL_TURN);
                    searchStart = ros::Time::now().toSec();
                    kanan = false;
                    searchState = Search_Pillar;
                    changeDirectionP += 1;
                }
                else
                {
                    for (int i = 0; i <= changeDirection; i++)
                        setWalkingParam(0, 0, MAX_RL_TURN);
                    searchStart = ros::Time::now().toSec();
                    kanan = true;
                    searchState = Search_Pillar;
                    changeDirectionP += 1;
                }
            }

            break;
        }
    }

    void Behavioral::searchHome(double robot_x, double robot_y, double robot_theta, double border)
    {
        double searchNow;
        static int changeDirectionH;

        if (firstSearch)
        {
            firstSearch = false;
            searchState = Search_Home;
            searchStart = ros::Time::now().toSec();
        }

        searchNow = ros::Time::now().toSec() - searchStart;
        switch (searchState)
        {
        case Search_Home:
            // trackingMode::HomeTracker::Sweep;

            if (searchNow > 30)
            {
                if (kanan)
                {
                    for (int i = 0; i <= changeDirectionH; i++)
                    {
                        setWalkingParam(0, 0, -MAX_RL_TURN);
                    }
                    searchStart = ros::Time::now().toSec();
                    kanan = false;
                    searchState = Search_Home;
                    changeDirectionH += 1;
                }
                else
                {
                    for (int i = 0; i <= changeDirectionH; i++)
                    {
                        setWalkingParam(0, 0, MAX_RL_TURN);
                    }
                    searchStart = ros::Time::now().toSec();
                    kanan = true;
                    searchState = Search_Home;
                    changeDirectionH += 1;
                }
            }

            break;
        }
    }

    void Behavioral::fallenKeeper(int keeper_fallen)
    {
        setModuleTo("action_module");

        ros::Duration(0.1).sleep();

        switch (keeper_fallen)
        {
        case Keeper_Right:
            // std::cout << "Keeper Right >>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
            playMotion(Motion_Index.KeeperRight);
            break;
        case Keeper_Left:
            // std::cout << "<<<<<<<<<<<<<<<<<<<<<<<< Keeper Left" << std::endl;
            playMotion(Motion_Index.KeeperLeft);
            break;
        case Keeper_Front:
            // std::cout << "^^^^^^^^^^^ Keeper Front ^^^^^^^^^^^" << std::endl;
            playMotion(Motion_Index.KeeperFront);
            break;
        }
    }

    Eigen::Vector2d Behavioral::checkFollowing(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        Eigen::Vector2d State;
        double RB_THETA = atan2(ball_y, ball_x) * 180 / PI; //Robot to Ball Theta

        // if( (fabs(robot_theta-ball_theta) < ANGLE_TOLERANCE && fabs(RB_THETA) < BALL_TOLERANCE && ball_x < FORWARD_MIN_REVO) ||
        //     (fabs(robot_theta-ball_theta) < ANGLE_TOLERANCE && ball_x > FORWARD_MIN_REVO) )
        // {
        //     State.x() = Face_Up_Enemy;
        // }
        // else
        // {
        State.x() = Face_Up_Home;

        if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO && fabs(robot_theta - ball_theta) > ANGLE_TOLERANCE)
            State.y() = Following_Revo;
        else
        {
            if (ball_x < 0)
                followState = Following_Backward;
            else if (fabs(ball_y) > RL_MIN_LIMIT && ball_x < FORWARD_MIN_REVO)
                followState = Following_Shift;
            else if (fabs(RB_THETA) > BALL_TOLERANCE)
                followState = Following_Rotate;
            else
            {
                if (FORWARD_MIN_REVO > ball_x && ball_x > FORWARD_MIN_LIMIT)
                    followState = Following_ForwardMin;
                else
                    followState = Following_Forward;
            }
        }
        // }

        return State;
    }

    void Behavioral::following(int following_state, double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        static bool kicking;
        firstSearch = true;

        switch (following_state)
        {
        case Face_Up_Enemy:
            if (!kickingDelay)
                kicking = faceUpEnemy(robot_theta, ball_x, ball_y, ball_theta);
            break;
        case Face_Up_Home:
            if (!kickingDelay)
                kicking = faceUpHome(robot_theta, ball_x, ball_y, ball_theta);
            break;
        }

        kickTimeNow = ros::Time::now().toSec() - kickTimeStart;

        if (kicking == true && kickingStatus == Not_Kicking)
        {
            stopFollowing();
            kickingDelay = true;

            if (kickTimeNow >= 3)
            {
                if (!CALIBRATION)
                {
                    ROS_WARN("SSSSEEEEEETTTTTT BBBBBOOOOOODDDDDDYYYYYYY MMMMMMMOOOOOOOOODDDUUUUULLLLEEEEE");
                    setBodyModuleTo("kicking", false);
                }

                if (head_.x >= 0)
                {
                    // std::cout<<"<<<<<<<===== KICKKKKKK LEFTTTTTTTTTT"<<head_.x<<std::endl;
                    startKicking("left_kick");
                }
                else
                {
                    // std::cout<<"KICKKKKKK RIGHTTTTTTTTT =====>>>>>>>"<<std::endl;
                    startKicking("right_kick");
                }
                kicking = false;
                lastKick = false;
                kickTimeStart = ros::Time::now().toSec();
            }
        }
        else
            kickingDelay = false;
    }

    bool Behavioral::faceUpEnemy(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        double fb_move = 0.0, rl_move = 0.0, turn_angle = 0.0;
        double RB_THETA = atan2(fabs(ball_x), fabs(ball_y)) * 180 / PI; //Robot to Ball Theta
        double THETA_LIMIT = 30;
        double LIMIT_Y = 30;
        trackingMode = BallTracker::Sweep;

        if ((ball_x > FORWARD_MIN_LIMIT) || (fabs(ball_y) > RL_MIN_LIMIT))
        {
            if (RB_THETA < THETA_LIMIT || ANGLE_LIMIT_TOLERANCE)
            {
                if (RB_THETA < 75)
                    ANGLE_LIMIT_TOLERANCE = true;
                else
                    ANGLE_LIMIT_TOLERANCE = false;

                if (fabs(ball_y) > LIMIT_Y) //Kondisi 1
                {
                    if (ball_y >= 0)
                        turn_angle = MAX_RL_TURN;
                    else
                        turn_angle = -MAX_RL_TURN;
                }
                else
                { //Kondisi 2
                    fb_move = MAX_BW_STEP;
                    rl_move = -MAX_RL_STEP; //-1 * (((fabs(ball_y)-RL_MIN_LIMIT)/(RL_MAX_LIMIT-RL_MIN_LIMIT)*(MAX_RL_STEP-MIN_RL_STEP))+MIN_RL_STEP);
                }
            }
            else //Kondisi 3
            {
                fb_move = MAX_FB_STEP; //(((ball_x-FORWARD_MIN_LIMIT)/(FORWARD_MAX_LIMIT-FORWARD_MIN_LIMIT)*(MAX_FB_STEP-MIN_FB_STEP))+MIN_FB_STEP);

                if (ball_y >= 0)
                {                              //kiri
                    rl_move = MAX_RL_STEP;     //(((fabs(ball_y)-RL_MIN_LIMIT)/(RL_MAX_LIMIT-RL_MIN_LIMIT)*(MAX_RL_STEP-MIN_RL_STEP))+MIN_RL_STEP);
                    turn_angle = -MAX_RL_TURN; //-1 *(((fabs(ball_theta)-TURN_MIN_LIMIT)/(TURN_MAX_LIMIT-TURN_MIN_LIMIT)*(MAX_RL_TURN-MIN_RL_TURN))+MIN_RL_TURN);
                }
                else
                {                             //kanan
                    rl_move = -MAX_RL_STEP;   //-1 * (((fabs(ball_y)-RL_MIN_LIMIT)/(RL_MAX_LIMIT-RL_MIN_LIMIT)*(MAX_RL_STEP-MIN_RL_STEP))+MIN_RL_STEP);
                    turn_angle = MAX_RL_TURN; //(((fabs(ball_theta)-TURN_MIN_LIMIT)/(TURN_MAX_LIMIT-TURN_MIN_LIMIT)*(MAX_RL_TURN-MIN_RL_TURN))+MIN_RL_TURN);
                }
            }

            calcWalkingParam(fb_move, rl_move, turn_angle);
            return false;
        }
        else
            return true;
    }

    bool Behavioral::faceUpHome(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        static double BALL_THETA, walkStart;
        double TOTAL_THETA = angle_yaw - BALL_THETA;
        double RB_THETA = atan2(ball_y, ball_x) * 180 / PI; //Robot to Ball Theta
        double walkNow;
        static bool walkDelay;

        // followState = Following_ForwardMin;

        // std::cout<<"Ball X\t"<<ball_x<<"\tBall Y\t"<<ball_y<<std::endl;
        // std::cout<<"RB_THETA\t"<<RB_THETA<<"\tROBOT THETA\t"<<robot_theta/*<<"\tBALL THETA\t"<<ball_theta*/<<std::endl;

        kickTimeStart = ros::Time::now().toSec();
        if (followState == Following_Forward || followState == Following_Rotate)
            BALL_THETA = ball_theta;

        if (ball_x < FORWARD_MIN_LIMIT && fabs(ball_y) < RL_MIN_LIMIT && fabs(RB_THETA) < BALL_TOLERANCE && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            return true;

        if (followState != Following_OnTheSpot)
            walkDelay = true;

        // if(followState == lastFollowState && followState != Following_Rotate){
        //     lastFollowState = followState;
        //     walkDelay = true;
        switch (followState)
        {
        case Following_Forward:
            // std::cout<<"<<=======FORWARD========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(MAX_FB_STEP, 0, 0);
            setWalkingParam(MAX_FB_STEP, 0.05, -0.06); // 0.3, 0, 0

            if (fabs(RB_THETA) > BALL_TOLERANCE)
            {
                // followState = Following_OnTheSpot;
                // BALL_THETA = ball_theta;
                followState = Following_Rotate;
            }
            else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                followState = Following_Revo;
            else if (ball_x < FORWARD_MIN_REVO)
                followState = Following_ForwardMin;
            break;

        case Following_Rotate:
            // std::cout<<"<<========ROTATE========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, 0, (RB_THETA < 0) ? -MAX_RL_TURN : MAX_RL_TURN);
            setWalkingParam(0, 0, ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? MAX_RL_TURN : -MAX_RL_TURN);

            if (fabs(RB_THETA) < ANGLE_TOLERANCE)
                followState = Following_Forward;
            else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                followState = Following_Revo;
            break;

        case Following_Revo:
            // std::cout<<"<<=========REVO=========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, ((robot_theta - ball_theta) < 0) ? REVO_CW_STEP : REVO_CCW_STEP, ((robot_theta - ball_theta) < 0) ? REVO_CW_TURN : REVO_CCW_TURN)

            if (TOTAL_THETA >= 0)
                setWalkingParam(0, REVO_CCW_STEP, REVO_CCW_TURN);
            else
                setWalkingParam(0, REVO_CW_STEP, REVO_CW_TURN);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else
            {
                if (fabs(TOTAL_THETA) < ANGLE_TOLERANCE)
                {
                    if (ball_x < 0)
                        followState = Following_Backward;
                    else if (fabs(ball_y) > RL_MIN_LIMIT)
                        followState = Following_Shift;
                    else
                        followState = Following_ForwardMin;
                }
                else if (fabs(RB_THETA) > BACKWARD_TOLERANCE)
                    followState = Following_Backward;
            }
            break;

        case Following_Shift:
            // std::cout<<"<<========SHIFT========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);
            setWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (ball_x > 0 && fabs(ball_y) < RL_MIN_LIMIT)
            {
                if (ball_x > FORWARD_MIN_LIMIT)
                    followState = Following_ForwardMin;
                else if (fabs(TOTAL_THETA) > ANGLE_TOLERANCE)
                    followState = Following_Revo;
            }
            else if (fabs(RB_THETA) > BACKWARD_TOLERANCE || ball_x < 0)
                followState = Following_Backward;
            break;

        case Following_ForwardMin:
            // std::cout<<"<<=====FORWARD MIN=====>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            calcWalkingParam(MIN_FB_STEP, 0, 0);
            // setWalkingParam(MIN_FB_STEP,0,0);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (fabs(RB_THETA) > BACKWARD_TOLERANCE)
                followState = Following_Shift;
            break;

        case Following_Backward:
            // std::cout<<"<<=======BACKWARD======>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            calcWalkingParam(MAX_BW_STEP, 0, 0);
            // setWalkingParam(MAX_BW_STEP,0,0);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (fabs(RB_THETA) < BACKWARD_TOLERANCE)
                followState = Following_Shift;
            else if (fabs(TOTAL_THETA) > ANGLE_TOLERANCE)
                followState = Following_Revo;
            break;

        case Following_OnTheSpot:
            // std::cout<<"-------ON THE SPOT------"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            if (walkDelay)
            {
                walkStart = ros::Time::now().toSec();
                walkDelay = false;
            }
            setWalkingParam(0, 0, 0);
            walkNow = ros::Time::now().toSec() - walkStart;

            if (walkNow > 2)
                followState = Following_Rotate;
            break;

        case Following_Trantition:
            calcWalkingParam(MIN_FB_STEP, 0, 0);
            // setWalkingParam(MIN_FB_STEP, 0, 0);

            if (FB_Step <= TRANTITION_BORDER)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                {
                    // followState = Following_OnTheSpot;
                    // BALL_THETA = ball_theta;
                    followState = Following_Rotate;
                }
                else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                    followState = Following_Revo;
            }
            break;
        }
        // }
        // else
        // {
        //    // ROS_WARN("-------------ON THE SPOT-------------");
        //     std::cout<<"<<==========ON THE SPOT==========>>"<<std::endl;
        //     if(walkDelay){
        //         walkStart = ros::Time::now().toSec();
        //         walkDelay = false;
        //     }
        //     setWalkingParam(0,0,0);
        //     walkNow = ros::Time::now().toSec() - walkStart;

        //     if(walkNow > 2.5)
        //     {
        //         switch(followState)
        //         {
        //             case Following_Forward:
        //                 followState = Following_Forward;
        //                 break;
        //             case Following_Rotate:
        //                 followState = Following_Rotate;
        //                 break;
        //             case Following_Revo:
        //                 followState = Following_Revo;
        //                 break;
        //             case Following_Shift:
        //                 followState = Following_Shift;
        //                 break;
        //             case Following_ForwardMin:
        //                 followState = Following_ForwardMin;
        //                 break;
        //             case Following_Backward:
        //                 followState = Following_Backward;
        //                 break;
        //         }
        //         lastFollowState = followState;
        //     }
        // }
        return false;
    }

    bool Behavioral::faceUpHomeV2(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        static double BALL_THETA, walkStart;
        double TOTAL_THETA = angle_yaw - BALL_THETA;
        double RB_THETA = atan2(ball_y, ball_x) * 180 / PI; //Robot to Ball Theta
        double walkNow;
        static bool walkDelay;
        stopFollowing();

        //AAANNNNYARRRR
        if (!firstSearch_homeV2)
        {
            std::cout << "======================FIRSTSEARCHV2===================" << std::endl;
            if (fabs(RB_THETA) > 14 && ball_x < 40)
                followState = Following_Revo;
            else
            {
                if (ball_x < 0)
                    followState = Following_Backward;
                // else if(fabs(ball_y) > RL_MIN_LIMIT && ball_x < FORWARD_MIN_REVO)
                //     followState = Following_Shift;
                else if (fabs(RB_THETA) > 15 && ball_x > 40)
                    followState = Following_Rotate;
                else
                {
                    if (FORWARD_MIN_REVO > ball_x && ball_x > FORWARD_MIN_LIMIT)
                        followState = Following_ForwardMin;
                    else
                        followState = Following_Forward;
                }
            }
            std::cout << "FOLLOWSTATE===================     " << followState << std::endl;
            firstSearch_homeV2 = true;
        }
        // std::cout<<"Ball X\t"<<ball_x<<"\tBall Y\t"<<ball_y<<std::endl;
        // std::cout<<"RB_THETA\t"<<RB_THETA<<"\tROBOT THETA\t"<<robot_theta/*<<"\tBALL THETA\t"<<ball_theta*/<<std::endl;

        kickTimeStart = ros::Time::now().toSec();
        if (followState == Following_Forward || followState == Following_Rotate)
            BALL_THETA = ball_theta;

        if (ball_x < 15 && fabs(ball_y) < 10 && fabs(RB_THETA) < 15)
        {
            ROS_ERROR("jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj");
            firstSearch_homeV2 = false;
            dribble_condition = true;
            return true;
        }
        if (followState != Following_OnTheSpot)
            walkDelay = true;

        // if(followState == lastFollowState && followState != Following_Rotate){
        //     lastFollowState = followState;
        //     walkDelay = true;
        // std::cout<<"======================firstSearch==================="<<firstSearch<<std::endl;
        std::cout << "======================searchState===================" << searchState << std::endl;

        switch (followState)
        {
        case Following_Forward:

            std::cout << "\t\t\t\tROBOT : " << robot_theta << "\tYAW : " << angle_yaw << "\t\t\tTHETA : " << TOTAL_THETA << std::endl;
            // calcWalkingParam(MAX_FB_STEP, 0, 0);
            setWalkingParam(MAX_FB_STEP, 0, 0);
            std::cout << "<<=======FORWARD========>>"
                      << "\tBall X : " << ball_x << "\tBall Y : " << ball_y << "\tRB T : " << RB_THETA << "\tBALL : " << ball_theta << std::endl;

            if (fabs(RB_THETA) > 15)
            {
                // followState = Following_OnTheSpot;
                // BALL_THETA = ball_theta;
                followState = Following_Rotate;
            }
            else if (fabs(RB_THETA) > 15 && ball_x < 40)
                followState = Following_Revo;
            else if (ball_x < FORWARD_MIN_REVO)
                followState = Following_ForwardMin;
            break;

        case Following_Rotate:

            std::cout << "\t\t\t\tROBOT : " << robot_theta << "\tYAW : " << angle_yaw << "\t\t\tTHETA : " << TOTAL_THETA << std::endl;
            // calcWalkingParam(0, 0, (RB_THETA < 0) ? -MAX_RL_TURN : MAX_RL_TURN);
            if (RB_THETA > 0)
                setWalkingParam(0.08, -0.18, 1);
            else
                setWalkingParam(0.1, 0.15, -1);
            std::cout << "<<========ROTATE========>>"
                      << "\tBall X : " << ball_x << "\tBall Y : " << ball_y << "\tRB T : " << RB_THETA << "\tBALL : " << ball_theta << std::endl;

            if (fabs(RB_THETA) < 15)
                followState = Following_Forward;
            else if (fabs(RB_THETA) > 15 && ball_x < 40)
                followState = Following_Revo;
            break;

        case Following_Revo:
            std::cout << "<<=========REVO=========>>"
                      << "\tBall X : " << ball_x << "\tBall Y : " << ball_y << "\tRB T : " << RB_THETA << "\tBALL : " << ball_theta << std::endl;
            std::cout << "\t\t\t\tROBOT : " << robot_theta << "\tYAW : " << angle_yaw << "\t\t\tTHETA : " << TOTAL_THETA << std::endl;
            // calcWalkingParam(0, ((robot_theta - ball_theta) < 0) ? REVO_CW_STEP : REVO_CCW_STEP, ((robot_theta - ball_theta) < 0) ? REVO_CW_TURN : REVO_CCW_TURN)

            if (RB_THETA >= 0)
                setWalkingParam(0, 0, 1);
            else
                setWalkingParam(0, 0, -1);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > 15 && ball_x > 40)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else
            {
                if (fabs(TOTAL_THETA) < ANGLE_TOLERANCE)
                {
                    if (ball_x < 0)
                        followState = Following_Backward;
                    else if (fabs(ball_y) > 10)
                        followState = Following_Shift;
                    else
                        followState = Following_ForwardMin;
                }
                else if (fabs(RB_THETA) > BACKWARD_TOLERANCE)
                    followState = Following_Backward;
            }
            break;

        case Following_Shift:
            std::cout << "<<========SHIFT========>>"
                      << "\tBall X : " << ball_x << "\tBall Y : " << ball_y << "\tRB T : " << RB_THETA << "\tBALL : " << ball_theta << std::endl;
            std::cout << "\t\t\t\tROBOT : " << robot_theta << "\tYAW : " << angle_yaw << "\t\t\tTHETA : " << TOTAL_THETA << std::endl;
            // calcWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);
            setWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (ball_x > 0 && fabs(ball_y) < RL_MIN_LIMIT)
            {
                if (ball_x > FORWARD_MIN_LIMIT)
                    followState = Following_ForwardMin;
                else if (fabs(TOTAL_THETA) > 15 && ball_x < 40)
                    followState = Following_Revo;
            }
            else if (fabs(RB_THETA) > BACKWARD_TOLERANCE || ball_x < 0)
                followState = Following_Backward;
            break;

        case Following_ForwardMin:
            std::cout << "<<=====FORWARD MIN=====>>"
                      << "\tBall X : " << ball_x << "\tBall Y : " << ball_y << "\tRB T : " << RB_THETA << "\tBALL : " << ball_theta << std::endl;
            std::cout << "\t\t\t\tROBOT : " << robot_theta << "\tYAW : " << angle_yaw << "\t\t\tTHETA : " << TOTAL_THETA << std::endl;
            calcWalkingParam(MAX_FB_STEP, 0, 0);
            // setWalkingParam(MIN_FB_STEP,0,0);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            // else if(fabs(RB_THETA) > BACKWARD_TOLERANCE)
            //     followState = Following_Shift;
            break;

        case Following_Backward:
            std::cout << "<<=======BACKWARD======>>"
                      << "\tBall X : " << ball_x << "\tBall Y : " << ball_y << "\tRB T : " << RB_THETA << "\tBALL : " << ball_theta << std::endl;
            std::cout << "\t\t\t\tROBOT : " << robot_theta << "\tYAW : " << angle_yaw << "\t\t\tTHETA : " << TOTAL_THETA << std::endl;
            calcWalkingParam(MAX_BW_STEP, 0, 0);
            // setWalkingParam(MAX_BW_STEP,0,0);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (fabs(RB_THETA) < BACKWARD_TOLERANCE)
                followState = Following_Shift;
            else if (fabs(TOTAL_THETA) > 15 && ball_x < 40)
                followState = Following_Revo;
            break;

        case Following_OnTheSpot:
            std::cout << "-------ON THE SPOT------"
                      << "\tBall X : " << ball_x << "\tBall Y : " << ball_y << "\tRB T : " << RB_THETA << "\tBALL : " << ball_theta << std::endl;
            std::cout << "\t\t\t\tROBOT : " << robot_theta << "\tYAW : " << angle_yaw << "\t\t\tTHETA : " << TOTAL_THETA << std::endl;
            if (walkDelay)
            {
                walkStart = ros::Time::now().toSec();
                walkDelay = false;
            }
            setWalkingParam(0, 0, 0);
            walkNow = ros::Time::now().toSec() - walkStart;

            if (walkNow > 2)
                followState = Following_Rotate;
            break;

        case Following_Trantition:
            calcWalkingParam(MIN_FB_STEP, 0, 0);
            // setWalkingParam(MIN_FB_STEP, 0, 0);

            if (FB_Step <= TRANTITION_BORDER)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                {
                    // followState = Following_OnTheSpot;
                    // BALL_THETA = ball_theta;
                    followState = Following_Rotate;
                }
                //     else if(fabs(RB_THETA) < REVO_TOLERANCE  && ball_x < FORWARD_MIN_REVO)
                //         followState = Following_Revo;
            }
            break;
        }
        return false;
    }

    bool Behavioral::positionCheck(double ball_x, double ball_y, double ball_theta)
    {
        double RB_THETA = atan2(ball_y, ball_x);
        double TOTAL_THETA = angle_yaw - ball_theta;
        if (ball_x < 15 && fabs(ball_y) < 10 && fabs(ball_theta) < 15)
            return true;
        else
            return false;
    }

    bool Behavioral::alignToPillar()
    {
        double searchNow;
        static bool firstSearch_align;
        if (!firstSearch_align)
        {
            std::cout << "==========BEHAVIORAL____ALIGGNNNNNNN_STARTTT  " << std::endl;
            firstSearch_align = true;
            // searchStart = ros::Time::now().toSec();
        }

        if (angle_yaw > 10 && angle_yaw < 50)
        {
            ROS_ERROR("iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii");
            return true;
        }

        if (angle_yaw > 50)
        {
            setWalkingParam(0, REVO_CCW_STEP, REVO_CCW_TURN);
            std::cout << "==========BEHAVIORAL____ALIGGNNNNNNN____CCCCCCCWWWWWWWWWWWWW  " << std::endl;
            if (angle_yaw < 50)
                return true;
        }
        else if (angle_yaw < 10)
        {
            setWalkingParam(0, REVO_CW_STEP, REVO_CW_TURN);
            std::cout << "==========BEHAVIORAL____ALIGGNNNNNNN____CWWWWWWWWWWWWWWWWWWWW  " << std::endl;
            if (angle_yaw > 10)
                return true;
        }
    }

    bool Behavioral::alignToHome(double ball_x_to_home, double ball_y_to_home, double home_x, double home_y)
    {
        // double BH_THETA = atan2(ball_y_to_home, ball_x_to_home);
        // double RH_THETA = atan2(home_y, home_x);

        // double difference = BH_THETA - RH_THETA;
        double difference = 4;

        // bool inPosition = positionCheck();
        // if(inPosition)
        // {
        if (fabs(difference) < 2) //blm tau parameter sudut uda aman deket tu brp 12345
            return true;

        if (difference > 0)
            setWalkingParam(0, REVO_CW_STEP, REVO_CW_TURN);
        else
            setWalkingParam(0, REVO_CCW_STEP, REVO_CCW_TURN);

        //harus dicoba
        // if (difference > 0)
        // {
        //     setWalkingParam(0, geser_y_berapa, geser_z_brp);
        //     if (fabs(difference) < 2)
        //         setWalkingParam(0, 0, 0);
        // }
        // else
        // {
        //     setWalkingParam(0, geser_y_brp, geser_x_brp);
        //     if(fabs(difference) < 2)
        //         setWalkingParam(0, 0, 0);
        // }
        // }

        return false;
    }

    bool Behavioral::dribble(double distance)
    {
        double searchNow;
        static bool firstSearch_dribbler;
        if (!firstSearch_dribbler)
        {
            searchStartt = ros::Time::now().toSec();
        }
        searchNow = ros::Time::now().toSec() - searchStartt;
        std::cout << "==========BEHAVIORAL__SEARCH_STARTTTTTTTTTTTTTT  " << searchStart << std::endl;
        std::cout << "==========BEHAVIORAL__SEARCH_NOWWWWWWWWWWWWWWWWWW  " << searchNow << std::endl;
        setWalkingParam(MAX_FB_STEP, 0, 0);
        if (searchNow > 10)
        {
            // searchStart = ros::Time::now().toSec();
            return true;
        }
        else
            return false;
    }

    bool Behavioral::back_dribble(double distance)
    {
        if (dribble_condition)
        {
            calcWalkingParam(MAX_BW_STEP, 0, 0);
            trackingMode = BallTracker::Nod;
            mileage_xDist += getOdometry().x();
            remain_xDist = distance - mileage_xDist;

            if (fabs(distance) > 15)
                calcWalkingParam(MAX_BW_STEP, 0, 0);
            else
                calcWalkingParam(MIN_BW_STEP, 0, 0);

            std::cout << "======================BW___DONE===================" << std::endl;

            if (remain_xDist > 0)
                return false;
            else
            {
                dribble_condition = false;
                return true;
            }
        }
    }

    bool Behavioral::RevoPillar(double pillar_x)
    {
        bool walk;
        double searchNow;
        static bool firstSearch_revoV2;

        if (!firstSearch_revoV2)
        {
            firstSearch_revoV2 = true;
            std::cout << "=============BEHAVIORAL FIRST SEARCHHH REVOOOO 2222================="
                      << "       " << firstSearch_revoV2 << std::endl;
            setWalkingParam(MAX_FB_STEP, 0, 0);
            std::cout << "=============== BEHAVIORAL MAJU REVO PILLAR=================" << std::endl;
            searchStart = ros::Time::now().toSec();
            m_revo_state = first;
            m_revo_status = firstRevo;
            std::cout << "=============BEHAVIORAL MLEBU REVO KONTTTLLLLOOOOOO====================="
                      << "       " << firstSearch_revoV2 << std::endl;
        }

        // if (walkfinal)
        // {
        //     std::cout<<"=============SUDA FINALLLLLLLLLLLLLL============="<<std::endl;
        //     walkfinal = false;
        //     return true;
        // }

        switch (m_revo_status)
        {
        case firstRevo:

            switch (m_revo_state)
            {
            case first:
                searchNow = ros::Time::now().toSec() - searchStart;
                std::cout << "======================FIRST_REVO===================" << std::endl;
                std::cout << "<<====================>>"
                          << "\tANGLE YAW : " << angle_yaw << "\tORIENTATION : " << orientation << "\tANGLE : " << zone_dist.y() << std::endl;
                if (searchNow > 2)
                {
                    setWalkingParam(0, REVO_CW_STEP, REVO_CW_TURN);
                    if (searchNow > 10)
                    {
                        firstSearch_revoV2 = false;
                        m_revo_state = second;
                        m_revo_status = firstRevo;
                        return false;
                    }
                }

                break;

            case second:
                searchNow = ros::Time::now().toSec() - searchStart;
                std::cout << "======================SECOND_REVO===================" << std::endl;
                if (searchNow > 2)
                {
                    setWalkingParam(0, REVO_CW_STEP, REVO_CW_TURN);
                    if (searchNow > 10)
                    {
                        // stopFollowing();
                        return true;
                    }
                }
                break;
            }
            break;
        }
    }

    bool Behavioral::goToLocation(double x_dist, double y_dist, double robot_theta)
    {
        bool xDist, yDist, angle;

        angle = rotate(robot_theta);


        if (angle == true)
        {
            xDist = forward(x_dist);
            if (xDist == true)
                return true;
            else
                return false;
        }
        else
            return false;
    }

    // void Behavioral::waiting(Eigen::Vector2d ball_pos, Eigen::Vector3d robot_pos)
    // {
    // }

    // bool Behavioral::waiting(double ball_x, double robot_x)
    // {
    //     if(ball_x )
    // }

    bool Behavioral::alignToBall(Eigen::Vector2d ball_pos, Eigen::Vector3d robot_pos)
    {
        return false;
    }

    bool Behavioral::reposition()
    {
    }

    // bool Behavioral::forward(double distance)
    // {
    //     trackingMode = BallTracker::Sweep;
    //     // mileage_xDist += getOdometry().x();
    //     mileage_xDist += odometry_.x;
    //     remain_xDist = distance - mileage_xDist;
    //     std::cout<<"REMAIN X DISTANCE"<<remain_xDist<<"\n";

    //     if (distance > 15)
    //         setWalkingParam(MAX_FB_STEP, 0.05, -0.06); // 0.3, 0, 0

    //         // calcWalkingParam(MAX_FB_STEP, 0, 0);
    //     // setWalkingParam(MAX_FB_STEP, 0, 0);
    //     else
    //         calcWalkingParam(MIN_FB_STEP,0.14, -0.06);
    //         // calcWalkingParam(MIN_FB_STEP, 0, 0);
    //     // setWalkingParam(MIN_FB_STEP, 0, 0);

    //     if (remain_xDist > 0)
    //         return true;
    //     else
    //         return false;
    // }

    bool Behavioral::forward(double moveTime)
    {
        static double timeNow, firstTime;
        std::cout<<"TIME NOW ["<<timeNow<<"]\n";
        // if(!firstIn)
        // {
        //     timeNow = ros::Time::now().toSec();
        //     firstIn = true;
        // }

         if (!firstIn){
            // m_pass_state = Pass_Forward;
            firstTime = ros::Time::now().toSec();
            firstIn = true;
        }

        timeNow = ros::Time::now().toSec() - firstTime;

        if(timeNow > moveTime){
            // firstIn = false;
            return true;
        }
        // else
        // {
        ROS_WARN("==================FORWARD========================>\n");
        calcWalkingParam(/*MAX_FB_STEP*/ 0.3, -0.1, 0.5);
        // calcWalkingParam(/*MAX_FB_STEP*/ 0.12, 0, 0);
        return false;


        // }    
    }

    bool Behavioral::shift(double distance)
    {
        trackingMode = BallTracker::Sweep;
        mileage_yDist += getOdometry().y();
        remain_yDist = distance - fabs(mileage_yDist);

        if (distance >= 0)
            calcWalkingParam(0, MAX_RL_STEP, 0);
        else
            calcWalkingParam(0, -MAX_RL_STEP, 0);

        if (remain_yDist > 0)
            return false;
        else
            return true;
    }

    bool Behavioral::rotate(double angle)
    {
        // trackingMode = BallTracker::Nod;
        std::cout<<"==================ROTATE========================>\n";
        double s_angle_yaw;
        // if (angle_yaw > 0)
        //     s_angle_yaw = resetOffset + angle_yaw;
        // else
        //     s_angle_yaw = angle_yaw - resetOffset;
        s_angle_yaw = angle_yaw - resetOffset;

        std::cout << "==========================================> " << fabs(s_angle_yaw - angle) << std::endl;

        // if (s_angle_yaw >=  angle )
        //     setWalkingParam(0, 0, 1);
        // else
        //     setWalkingParam(0, -0.3, -1);
        setWalkingParam((s_angle_yaw >= angle) ? 0.01 : 0.01,
                 (s_angle_yaw >= angle) ? -0.08 : 0.1,
                  (s_angle_yaw >= angle) ? MAX_RL_TURN : -MAX_RL_TURN);

        if (fabs(s_angle_yaw - angle) < 10)
            return true;
        else
            return false;
    }

    bool Behavioral::revo(double angle)
    {
        trackingMode = BallTracker::Nod;
        if (angle >= 0)
            calcWalkingParam(0, REVO_CCW_STEP, REVO_CCW_TURN);
        else
            calcWalkingParam(0, REVO_CW_STEP, REVO_CW_TURN);

        if (fabs(angle) < ANGLE_TOLERANCE)
            return true;
        else
            return false;
    }

    double Behavioral::getVelocity(std::vector<cv::Point2f> _vp)
    {
        double _pindahX = _vp.at(_vp.size() - 1).x - _vp.at(_vp.size() - 2).x;

        for (int s = 1; s < _vp.size(); s++)
        {
            if (abs(_vp.at(s).x - _vp.at(s - 1).x) > _pindahX)
                _pindahX = _vp.at(s).x - _vp.at(s - 1).x;
        }
        return _pindahX;
    }

    double Behavioral::x_y_score(std::vector<cv::Point2f> _vp, int x_score)
    {
        cv::Mat A = cv::Mat::ones(_vp.size(), 2, CV_32FC1);
        cv::Mat y = cv::Mat(_vp.size(), 1, CV_32FC1);

        for (int b = 0; b < _vp.size(); b++)
        {
            A.at<float>(b, 1) = _vp.at(b).x;
            y.at<float>(b) = _vp.at(b).y;
        }

        cv::Mat AtA = A.t() * A;
        cv::Mat cm = AtA.inv() * A.t() * y;

        cv::Mat x = (cv::Mat_<float>(1, 2) << 1.0, float(x_score));

        cv::Mat mY_X_ = x * cm;
        double Y_X_ = double(mY_X_.at<float>(0));

        return Y_X_;
    }

    int Behavioral::predictionSudiro(double pos_x, double pos_y, double ball_x, double ball_y)
    {
        int keeperCondition;
        double jari2 = 8;

        double prediksiY = 0.0;
        double kecepatanX = 0.0;

        if (ball_x <= 150)
        {
            if (vp.size() < 5)
            {
                vp.push_back(cv::Point2f(ball_x, ball_y));
                if (vp.size() >= 2)
                {
                    kecepatanX = getVelocity(vp);

                    ROS_WARN("perpindahan: %lf\n", kecepatanX);
                    if (abs(kecepatanX) <= jari2 * 0.75)
                        counterSize++;

                    if (counterSize == 3) // jarak kurang 1000, kondisi diam
                    {
                        vp.clear();
                        counterSize = 0;

                        if ((ball_y >= -4 * jari2) && (ball_y <= 4 * jari2))
                        {
                            ROS_INFO("DIAM");

                            keeperCondition = Do_Nothing;
                        }
                        else if (ball_y < -4 * jari2)
                        {
                            ROS_INFO("GESER KANAN");

                            keeperCondition = Shift_Right;
                        }
                        else if (ball_y > 4 * jari2)
                        {
                            ROS_INFO("GESER KIRI");

                            keeperCondition = Shift_Left;
                        }
                    }
                }
            }

            else if (vp.size() == 5)
            {
                counterSize = 0;

                kecepatanX = getVelocity(vp);
                if (kecepatanX < -jari2 * 0.75) // jarak kurang dari 1000, mendekati gawang
                {
                    prediksiY = x_y_score(vp, 0);

                    if (abs(prediksiY) <= 30)
                    {
                        ROS_INFO("JATUH DEPAN");

                        keeperCondition = Keeper_Front;
                    }
                    else if (prediksiY > 30)
                    {
                        ROS_INFO("JATUH KIRI");

                        keeperCondition = Keeper_Left;
                    }
                    else if (prediksiY < -30)
                    {
                        ROS_INFO("JATUH KANAN");

                        keeperCondition = Keeper_Right;
                    }
                }
                else if (kecepatanX > jari2 * 0.75) // jarak kurang dari 1000, menjauhi gawang
                {
                    if (abs(ball_y) <= 4 * jari2)
                    {
                        ROS_INFO("DIAM");

                        keeperCondition = Do_Nothing;
                    }
                    else if (ball_y > 4 * jari2)
                    {
                        ROS_INFO("GESER KIRI");

                        keeperCondition = Shift_Left;
                    }
                    else if (ball_y < -4 * jari2)
                    {
                        ROS_INFO("GESER KANAN");

                        keeperCondition = Shift_Right;
                    }
                }
                vp.clear();
            }
        }
        else // jarak lebih dari 1000, apapun kondisi nya
        {
            if (abs(ball_y) <= 4 * jari2)
            {
                ROS_INFO("DIAM");

                keeperCondition = Do_Nothing;
            }
            else if (ball_y > 4 * jari2)
            {
                ROS_INFO("GESER KIRI");

                keeperCondition = Shift_Left;
            }
            else if (ball_y < -4 * jari2)
            {
                ROS_INFO("GESER KANAN");

                keeperCondition = Shift_Right;
            }

            if (vp.size() != 0)
                vp.clear();
        }

        return keeperCondition;
    }

    int Behavioral::shifting(double ball_x, double ball_y, double hold_time)
    {
        bool kicking;
        // std::cout<<"BALL X: "<<ball_x<<"\tBALL Y: "<<ball_y<<std::endl;

        if ((FORWARD_MIN_LIMIT < ball_x && ball_x < 25 && fabs(ball_y) > RL_MIN_LIMIT) || (200 > ball_x > 100 && fabs(ball_y) > 30))
        {
            if (ball_y > 0)
                return keeperState = Shift_Left;
            else
                return keeperState = Shift_Right;
        }
        else if (ball_x < FORWARD_MIN_LIMIT && fabs(ball_y) < RL_MIN_LIMIT)
        {
            if (kickingDelay == Not_Kicking)
            {
                // std::cout<<"KICK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                // if (ball_y > 0)
                // startKicking("left_kick");
                // else
                // startKicking("right_kick");
            }
            return keeperState = Kick;
        }
        else
            return keeperState = Do_Nothing;

        return kicking;
    }

    int Behavioral::prediction(double ball_x, double ball_y, double hold_time)
    {
        for (int i = 0; i < REGRESSION_NUMBER - 1; i++)
        {
            x[i] = x[i + 1];
            y[i] = y[i + 1];
        }
        x[3] = ball_x;
        y[3] = ball_y;

        double current_vel_x = x[3] - x[2];
        double current_vel_y = y[3] - y[2];
        double avg_vel_x = (current_vel_x + vel_x[0] + vel_x[1]) / 3;
        double avg_vel_y = (current_vel_y + vel_y[0] + vel_y[1]) / 3;
        double avg_vel = sqrt(pow(avg_vel_x, 2) + pow(avg_vel_y, 2));

        double pred_x = x[3];
        double pred_y = y[3];

        vel_x[0] = vel_x[1];
        vel_x[1] = current_vel_x;
        vel_y[0] = vel_y[1];
        vel_y[1] = current_vel_y;

        // std::cout << "Ball X : " << ball_x << "\tBall Y : " << ball_y << std::endl;
        // std::cout << "Orientasi : " << orientation << "\tCurrent Vel : " << avg_vel << "\tTime : " << hold_time << std::endl;

        for (int i = 0; i < FRAME_NUMBER; i++)
        {
            pred_x = pred_x + avg_vel_x;
            pred_y = pred_y + avg_vel_y;
        }

        if (hold_time <= 0.6 || (ball_x == 0 && ball_y == 0))
            return Prediction;

        if ((pred_x < p_pred_x[1] && avg_vel > p_avg_vel[1]) ||
            (pred_x < p_pred_x[2] && avg_vel > p_avg_vel[2]) ||
            (pred_x < p_pred_x[3] && avg_vel > p_avg_vel[3]) ||
            (pred_x < p_pred_x[4] && avg_vel > p_avg_vel[4]) ||
            (pred_x < p_pred_x[5] && avg_vel > p_avg_vel[5]) ||
            (pred_x < p_pred_x[6] && avg_vel > p_avg_vel[6]))
        {
            if (FALLEN_MARGIN > pred_y && pred_y > CENTER_MARGIN)
                return Keeper_Left;
            else if (pred_y > -CENTER_MARGIN)
                return Keeper_Front;
            else if (pred_y > -FALLEN_MARGIN)
                return Keeper_Right;
        }

        return Prediction;
    }

    bool Behavioral::gabShifting(double robot_x, double robot_y, double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        bool kicking = false;
        double RB_THETA = atan2(ball_y, ball_x) * 180 / PI; //Robot to Ball Theta
        double searchNow = ros::Time::now().toSec() - searchStart;

        switch (keeperState)
        {
        case Do_Nothing:
            stopFollowing();
            if (fabs(RB_THETA) > SHIFTING_TOLERANCE && fabs(robot_y) < FOLLOWING_MARGIN)
                keeperState = Keeper_Shifting;
            if (ball_y < FOLLOWING_MARGIN)
            {
                mileage_xDist = 0;
                mileage_yDist = 0;
                keeperState = Following;
            }
            break;
        case Keeper_Shifting:
            ROS_WARN("---------SHIFT---------");
            ROS_WARN("---------SHIFT---------");
            ROS_WARN("---------SHIFT---------");
            ROS_WARN("---------SHIFT---------");
            setWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);
            if (fabs(RB_THETA) < SHIFTING_TOLERANCE || fabs(robot_y) < FOLLOWING_MARGIN)
                keeperState = Do_Nothing;
            break;
        case Keeper_Rotate:
            ROS_WARN("-------- ROTATE--------");
            ROS_WARN("-------- ROTATE--------");
            ROS_WARN("-------- ROTATE--------");
            ROS_WARN("-------- ROTATE--------");
            setWalkingParam(0, 0, (angle_yaw > 0) ? MAX_RL_TURN : -MAX_RL_TURN);
            if (fabs(angle_yaw) < ANGLE_TOLERANCE)
                keeperState = Do_Nothing;
            break;
        case Following:
            // ROS_WARN("-------------FOLLOWING-------------");
            kicking = faceUpHome(angle_yaw, ball_x, ball_y, ball_theta);
            if (kicking)
            {
                zone_dist = calcRobotDiff(robot_x, robot_y, angle_yaw, -440, 0);
                searchStart = ros::Time::now().toSec();
                keeperState = Align_Pose;
            }
            break;
        case Align_Pose:
            ROS_WARN("-----ALIGN POSE-----");
            ROS_WARN("-----ALIGN POSE-----");
            ROS_WARN("-----ALIGN POSE-----");
            ROS_WARN("-----ALIGN POSE-----");
            if (robot_zone.y() == 1 || robot_zone.y() == 4)
                setWalkingParam(0, 0, -MAX_RL_TURN);
            else
                setWalkingParam(0, 0, MAX_RL_TURN);

            if (searchNow > 3 && fabs(angle_yaw - zone_dist.y()) < ANGLE_TOLERANCE)
                keeperState = Positioning;
            break;
        case Positioning:
            ROS_WARN("====POSITIONING ====");
            ROS_WARN("====POSITIONING ====");
            ROS_WARN("====POSITIONING ====");
            ROS_WARN("====POSITIONING ====");
            setWalkingParam(MAX_FB_STEP, 0, 0);
            if (robot_x < -440)
                keeperState = Keeper_Rotate;
            break;
        }

        return kicking;
    }

    bool Behavioral::gabPrediction(double hold_time, double robot_x, double robot_y, double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        bool kicking = false;
        double RB_THETA = atan2(ball_y, ball_x) * 180 / PI; //Robot to Ball Theta
        double ball_world_y = ball_y + robot_y;
        double searchNow = ros::Time::now().toSec() - searchStart;

        switch (keeperState)
        {
        case Do_Nothing:
            stopFollowing();
            // std::cout<<"HOLD TIME : "<<hold_time<<"\tBALL X : "<<ball_x<<"\tBALL Y : "<<ball_y<<"\tROBOT X : "<<robot_x<<"\tROBOT Y : "<<robot_y<<std::endl;
            keeperState = prediction(ball_x, ball_y, hold_time);
            if (keeperState == Prediction)
                keeperState = Do_Nothing;

            if (fabs(angle_yaw) > ANGLE_TOLERANCE)
                keeperState = Keeper_Rotate;
            else if ((fabs(ball_world_y) > FALLEN_MARGIN && ball_x > FOLLOWING_MARGIN) && (fabs(RB_THETA) > SHIFTING_TOLERANCE) && fabs(robot_y) < FOLLOWING_MARGIN)
                keeperState = Keeper_Shifting;

            if (ball_x < FOLLOWING_MARGIN && keeperState == Do_Nothing)
                keeperState = Following;
            else if ((fabs(RB_THETA) < SHIFTING_TOLERANCE || ball_x > FOLLOWING_MARGIN) && keeperState == Do_Nothing)
                keeperState = Do_Nothing;

            if (keeperState == Keeper_Shifting || keeperState == Keeper_Rotate || keeperState == Following)
                startFollowing();
            break;
        case Keeper_Shifting:
            ROS_WARN("---------SHIFT---------");
            ROS_WARN("---------SHIFT---------");
            ROS_WARN("---------SHIFT---------");
            ROS_WARN("---------SHIFT---------");
            setWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);
            if (fabs(RB_THETA) < SHIFTING_TOLERANCE || fabs(robot_y) < FOLLOWING_MARGIN)
                keeperState = Do_Nothing;
            // case Keeper_Rotate:
            ROS_WARN("-------- ROTATE--------");
            ROS_WARN("-------- ROTATE--------");
            ROS_WARN("-------- ROTATE--------");
            ROS_WARN("-------- ROTATE--------");
            if (!CALIBRATION)
                setWalkingParam(0, 0, (angle_yaw > 0) ? MAX_RL_TURN : -MAX_RL_TURN);
            if (fabs(angle_yaw) < ANGLE_TOLERANCE)
                keeperState = Do_Nothing;
        case Keeper_Left:
            ROS_WARN("<<<<<<<<<<<<<<<< KIRI");
            ROS_WARN("<<<<<<<<<<<<<<<< KIRI");
            ROS_WARN("<<<<<<<<<<<<<<<< KIRI");
            ROS_WARN("<<<<<<<<<<<<<<<< KIRI");
            if (!CALIBRATION)
            {
                fallenKeeper(Keeper_Left);
                while (isActionRunning() == true)
                    ros::Duration(0.05).sleep();
                setBodyModuleTo("quintic_walk", true);
            }
            keeperState = Do_Nothing;
            break;
        case Keeper_Right:
            ROS_WARN("KANAN >>>>>>>>>>>>>>>");
            ROS_WARN("KANAN >>>>>>>>>>>>>>>");
            ROS_WARN("KANAN >>>>>>>>>>>>>>>");
            ROS_WARN("KANAN >>>>>>>>>>>>>>>");
            if (!CALIBRATION)
            {
                fallenKeeper(Keeper_Right);
                while (isActionRunning() == true)
                    ros::Duration(0.05).sleep();
                setBodyModuleTo("quintic_walk", true);
            }
            keeperState = Do_Nothing;
            break;
        case Keeper_Front:
            ROS_WARN("^^^^^^^ DEPAN^^^^^^^");
            ROS_WARN("^^^^^^^ DEPAN^^^^^^^");
            ROS_WARN("^^^^^^^ DEPAN^^^^^^^");
            ROS_WARN("^^^^^^^ DEPAN^^^^^^^");
            ROS_WARN("^^^^^^^ DEPAN^^^^^^^");
            if (!CALIBRATION)
            {
                fallenKeeper(Keeper_Front);
                while (isActionRunning() == true)
                    ros::Duration(0.05).sleep();
                setBodyModuleTo("quintic_walk", true);
            }
            keeperState = Do_Nothing;
            break;
        case Following:
            // ROS_WARN("-------------FOLLOWING-------------");
            kicking = faceUpHome(angle_yaw, ball_x, ball_y, ball_theta);
            if (kicking)
            {
                zone_dist = calcRobotDiff(robot_x, robot_y, angle_yaw, -440, 0);
                searchStart = ros::Time::now().toSec();
                keeperState = Align_Pose;
            }
            break;
        case Align_Pose:
            ROS_WARN("-----ALIGN POSE-----");
            ROS_WARN("-----ALIGN POSE-----");
            ROS_WARN("-----ALIGN POSE-----");
            ROS_WARN("-----ALIGN POSE-----");

            if (robot_zone.y() == 1 || robot_zone.y() == 4)
                setWalkingParam(0, 0, -MAX_RL_TURN);
            else
                setWalkingParam(0, 0, MAX_RL_TURN);

            if (searchNow > 3 && fabs(angle_yaw - zone_dist.y()) < ANGLE_TOLERANCE)
                keeperState = Positioning;
            break;
        case Positioning:
            ROS_WARN("====POSITIONING ====");
            ROS_WARN("====POSITIONING ====");
            ROS_WARN("====POSITIONING ====");
            ROS_WARN("====POSITIONING ====");
            setWalkingParam(MAX_FB_STEP, 0, 0);
            if (robot_x < -440)
                keeperState = Keeper_Rotate;
            break;
        }

        return kicking;
    }

    void Behavioral::gabKeeper(int keeper_mode, double hold_time, double robot_x, double robot_y, double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        static bool kicking;

        switch (keeper_mode)
        {
        case Fallen_Mode:
            if (!kickingDelay)
                kicking = gabPrediction(hold_time, robot_x, robot_y, robot_theta, ball_x, ball_y, ball_theta);
            break;
        case Shifting_Mode:
            if (!kickingDelay)
                kicking = gabShifting(robot_x, robot_y, robot_theta, ball_x, ball_y, ball_theta);
            break;
        }

        kickTimeNow = ros::Time::now().toSec() - kickTimeStart;

        if (kicking == true && kickingStatus == Not_Kicking)
        {
            setWalkingParam(0, 0, 0);
            kickingDelay = true;

            if (kickTimeNow >= KICK_DELAY)
            {
                if (!CALIBRATION)
                    setBodyModuleTo("kicking", true);

                if (head_.x >= 0)
                {
                    // std::cout<<"<<<<<<<===== KICKKKKKK LEFTTTTTTTTTT"<<head_.x<<std::endl;
                    startKicking("left_kick");
                }
                else
                {
                    // std::cout<<"KICKKKKKK RIGHTTTTTTTTT =====>>>>>>>"<<std::endl;
                    startKicking("right_kick");
                }
                kickingDelay = false;
                kicking = false;
                kickTimeStart = ros::Time::now().toSec();
            }
        }
        else
            kickingDelay = false;
    }

    int Behavioral::predictionMode(double robot_theta, double ball_x, double ball_y, double hold_time, double ball_theta)
    {
        if (fabs(orientation) > ANGLE_TOLERANCE)
            keeperState = Keeper_Rotate;

        switch (keeperState)
        {
        case Keeper_Rotate:
            ROS_INFO("--ROTATE--");
            startFollowing();
            calcWalkingParam(0, 0, (orientation > 0) ? MAX_RL_TURN : -MAX_RL_TURN);
            if (fabs(orientation) < ANGLE_TOLERANCE)
                keeperState = Prediction;
            break;

        case Prediction:
            setWalkingCommand("stop");
            keeperState = prediction(ball_x, ball_y, hold_time);
            break;

        case Keeper_Left:
            ROS_WARN("<<<<<<<<<<KIRI");
            stopFollowing();
            fallenKeeper(Keeper_Left);
            while (isActionRunning() == true)
                ros::Duration(0.05).sleep();
            setBodyModuleTo("walk_zmp", true);
            keeperState = Prediction;
            break;

        case Keeper_Right:
            ROS_WARN("KANAN>>>>>>>>>>");
            stopFollowing();
            fallenKeeper(Keeper_Right);
            while (isActionRunning() == true)
                ros::Duration(0.05).sleep();
            setBodyModuleTo("walk_zmp", true);
            keeperState = Prediction;
            break;

        case Keeper_Front:
            ROS_WARN("^^^^^DEPAN^^^^^");
            stopFollowing();
            fallenKeeper(Keeper_Front);
            while (isActionRunning() == true)
                ros::Duration(0.05).sleep();
            setBodyModuleTo("walk_zmp", true);
            keeperState = Prediction;
            break;

        case Do_Nothing:
            ROS_INFO("--NOTHING--");
            keeperState = Prediction;
            break;
        }
    }

    int Behavioral::shiftingMode(double robot_theta, double ball_x, double ball_y, double hold_time, double ball_theta)
    {
        if (fabs(orientation) > ANGLE_TOLERANCE)
            keeperState = Keeper_Rotate;

        switch (keeperState)
        {
        case Keeper_Rotate:
            ROS_WARN("- - - R O T A T E - - -");
            if (!m_following_status)
            {
                startFollowing();
                m_following_status = true;
            }
            calcWalkingParam(0, 0, (orientation > 0) ? MAX_RL_TURN : -MAX_RL_TURN);
            if (fabs(orientation) < ANGLE_TOLERANCE)
                keeperState = Keeper_Shifting;
            break;

        case Keeper_Shifting:
            keeperState = shifting(ball_x, ball_y, hold_time);
            break;

        case Shift_Left:
            ROS_WARN("<<<<<<<<<<SHIFT_KIRI");
            if (!m_following_status)
            {
                startFollowing();
                m_following_status = true;
            }
            shift(10);
            keeperState = Keeper_Shifting;
            break;

        case Shift_Right:
            ROS_WARN("SHIFT_KANAN>>>>>>>>>>");
            if (!m_following_status)
            {
                startFollowing();
                m_following_status = true;
            }
            shift(10);
            keeperState = Keeper_Shifting;
            break;

        case Kick:
            ROS_WARN("- - - K I C K I N G - - -");
            m_following_status = false;
            keeperState = Keeper_Shifting;

        case Do_Nothing:
            ROS_INFO("- - - N O T H I N G - - -");
            stopFollowing();
            m_following_status = false;
            keeperState = Keeper_Shifting;
            break;
        }
    }

    void Behavioral::keeper(int keeper_mode, double robot_theta, double ball_x, double ball_y, double hold_time, double ball_theta)
    {
        bool kicking = false;

        switch (keeper_mode)
        {
        case Fallen_Mode:
            // ROS_INFO("FALLEN MODE");
            kicking = predictionMode(robot_theta, ball_x, ball_y, hold_time, ball_theta);
            break;
        case Shifting_Mode:
            // ROS_INFO("SHIFTING MODE");
            kicking = shiftingMode(robot_theta, ball_x, ball_y, hold_time, ball_theta);
            break;
        }

        // if(kicking)
        // {
        //     ROS_INFO("KKKKKKKIIIIIIIIIIIICCCCCCCCCCCCKKKKKKKKKKKK");
        //     setBodyModuleTo("kicking", false);
        //     kickingDelay = true;

        //     kickTimeNow = ros::Time::now().toSec() - kickTimeStart;

        //     if(kickTimeNow >= KICK_DELAY)
        //     {
        //         ROS_INFO("KKKKKKKIIIIIIIIIIIICCCCCCCCCCCCKKKKKKKKKKKK  AAAAAAAAAAAAAAAAAAAAA");
        //         if(ball_y > 0)
        //             startKicking("left_kick");
        //         else
        //             startKicking("right_kick");

        //         // keeperState = Align_Pose;
        //         keeperState = Do_Nothing;
        //     }
        // }
        // else
        //     kickingDelay = false;
    }

    void Behavioral::send(){
        std_msgs::String msg;
        msg.data = "send";

        kick_status_pub_.publish(msg);
    }

    void Behavioral::resetComm(){
        std_msgs::String msg;
        msg.data = "reset";

        kick_status_pub_.publish(msg);
    }

    void Behavioral::passerSearch(bool role)
    {
        std_msgs::String data;
        data.data = "Not_Kicking";

        kick_status_pub_.publish(data);
        // trackingMode = BallTracker::Nod;

        setWalkingParam(-0.2, 0, 0);
    }

    void Behavioral::passerCheckFollowing(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        double RB_THETA = atan2(ball_y, ball_x) * 180 / PI; //Robot to Ball Theta
        std::cout<<"passercheckfollowing"<<std::endl;
        // if(ball_x > 40)
        // {
        //     followState = Following_Forward;
        // }
        // --Mei 2024--
        if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO && fabs(robot_theta - ball_theta) > ANGLE_TOLERANCE)
            followState = Following_Revo;
        // if (fabs(getOdometry().y() > 60)) {

        //     followState = Following_Revo;
        // }
        else
        {
           // if (ball_x < 0)
            //     followState = Following_Backward;
            if (fabs(RB_THETA) > BALL_TOLERANCE)
                followState = Following_Rotate;
            else if (fabs(ball_y) > RL_MIN_LIMIT && ball_x < FORWARD_MIN_REVO)
                followState = Following_Shift;
            
            else
            {
                if (FORWARD_MIN_REVO > ball_x && ball_x > FORWARD_MIN_LIMIT)
                    followState = Following_ForwardMin;
                else
                    followState = Following_Forward;
            }
        }
    }

    void Behavioral::kicking()
    {
        ROS_ERROR("++++++++++++++++++++++++++++++++++++++++++++++++");
        
        if (firstKick)
            setBodyModuleTo("kicking", false);

        std_msgs::String msg;

        msg.data = "right_kick";

        set_kicking_command_pub_.publish(msg);
    }

    // bool Behavioral::kickingHarusKanan()
    // {
    //     ROS_WARN("Menghindai left-kick");
    //     if(head_.x < 0)
    //         return true;
    //     else
    //     {
    //         setWalkingParam(-0.05, MAX_RL_STEP, 0.7);
    //     }
    // }

    void Behavioral::dribblerFollowing(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {   
        // ROS_WARN("PASSER FOLLLLlllll");

        std_msgs::String data;
        data.data = "Not_Kicking";

        kick_status_pub_.publish(data);

        static bool kicking;
        static bool kickingKanan;
        // static bool jalanDiTempat;
        firstSearch = true;
        std::cout<<"lastKick : "<<lastKick<<"\n";

        if (!lastKick)
            kicking = GoingToBall(robot_theta, ball_x, ball_y, ball_theta);
            // kicking = myBalls(robot_theta, ball_x, ball_y, ball_theta);

        kickTimeNow = ros::Time::now().toSec() - kickTimeStart;

        ROS_INFO("KICK TIME NOW [%f]", kickTimeNow);
        ROS_ERROR("Head x [%f]", head_.x);
        std::cout<<"kicking : "<<kicking<<"\n";

        if (kicking == true /*&& kickingStatus == Not_Kicking*/)
        {
            // stopFollowing();
            // if(stableWalk())
            // {
            //     kickingDelay = true;

            //     if (kickTimeNow >= KICK_DELAY)
            //     {
            //         if (!CALIBRATION)
            //             setBodyModuleTo("kicking", false);

            //         if (head_.x >= 0)
            //             startKicking("left_kick");

            //         else
            //             startKicking("right_kick");
            //         kicking = false;
            //         kickTimeStart = ros::Time::now().toSec();
            //     }
            // }
            // else
            //     ROS_WARN("LAGI JALAN DI TEMPAT");
            // kickingDelay = true;

        //     if (kickTimeNow >= KICK_DELAY)
        //     {
        //         // if (!CALIBRATION)
        //         //     setBodyModuleTo("kicking", false);

        //         // if (head_.x >= 0)
        //         //     startKicking("left_kick");

        //         // else
        //         //     startKicking("right_kick");
        //         // kicking = false;
        //         // kickTimeStart = ros::Time::now().toSec();
        //     }
        // }
        // else
        //     kickingDelay = false;
            lastKick = true;
            if(forward(2)) {
                finishedKick = true;
                kicking = false;
            }
        }   
    }
    void Behavioral::passerFollowing(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {   
        
        ROS_WARN("PASSER FOLLLLlllll");
       
        std_msgs::String data;
        data.data = "Not_Kicking";

        kick_status_pub_.publish(data);

        static bool kicking;
        static bool maju;
        // static bool jalanDiTempat;
        firstSearch = true;
        
        std::cout<<"lastKick : "<<lastKick<<"\n";
        
        // if(fabs(robot_theta > 90))
        // {
        //     maju = true;
        //     startFollowing();
        //     return;
        // }
        // else
        // {
        //     if(ball_x < 150)
        //     {
        //         maju = true;
        //     }
        // }

        
        if (!lastKick)
        {
            kicking = GoingToBall(robot_theta, ball_x, ball_y, ball_theta);
            // kicking = myBalls(robot_theta, ball_x, ball_y, ball_theta);
            // kicking = yourBalls_2(robot_theta, ball_x, ball_y, ball_theta);
            // maju = false;
        }
            

        kickTimeNow = ros::Time::now().toSec() - kickTimeStart;
     
        ROS_INFO("KICK TIME NOW [%f]", kickTimeNow);
        ROS_ERROR("Head x [%f]", head_.x);
        std::cout<<"kicking : "<<kicking<<"\n";
       
        if (kicking == true /*&& kickingStatus == Not_Kicking*/)
        {
            stopFollowing();
            lastKick = true;
            // if(stableWalk())
            // {
            //     kickingDelay = true;

            //     if (kickTimeNow >= KICK_DELAY)
            //     {
            //         if (!CALIBRATION)
            //             setBodyModuleTo("kicking", false);

            //         if (head_.x >= 0)
            //             startKicking("left_kick");

            //         else
            //             startKicking("right_kick");
            //         kicking = false;
            //         kickTimeStart = ros::Time::now().toSec();
            //     }
            // }
            // else
            //     ROS_WARN("LAGI JALAN DI TEMPAT");
            kickingDelay = true;

            if (kickTimeNow >= KICK_DELAY)
            {
                if (!CALIBRATION)
                    setBodyModuleTo("kicking", false);

                if (head_.x >= 0)
                    startKicking("left_kick");

                else
                    startKicking("right_kick");
                kicking = false;
                kickTimeStart = ros::Time::now().toSec();
                lastKick = false;
            }
        }
        else
            kickingDelay = false;
    }

    bool Behavioral::positioning(double goal_x, double goal_y)
    {
        static int m_pass_state = Pass_Forward;
        static bool first = false;
        static double firstTime, timeNow;

        if (!first)
            firstTime = ros::Time::now().toSec();

        first = true;
        timeNow = ros::Time::now().toSec() - firstTime;

        if (goal_x < 200)
            return true;

        ROS_INFO("TIME NOW [%f]", timeNow);

        switch (m_pass_state)
        {
        case Pass_Forward:
            ROS_INFO("FORWARD");
            setWalkingParam(MAX_FB_STEP, 0, 0);
            if (angle_yaw > 10)
                m_pass_state = Pass_Rotate;
            break;

        case Pass_Rotate:
            ROS_INFO("Rotate");
            setWalkingParam(0, 0, (angle_yaw > 0) ? 1 : -1);
            if (angle_yaw < 10)
                m_pass_state = Pass_Forward;
            break;
        }

        return false;
    }

    bool Behavioral::timePositioning()
    {
        static int m_pass_state;
        static double firstTime, timeNow;

        std_msgs::String data;
        data.data = "Not_Kicking";

        kick_status_pub_.publish(data);

        double s_angle_yaw;

        s_angle_yaw = angle_yaw - resetOffset;

        std::cout << "=================== Angle Yaw ===============" << angle_yaw << std::endl;
        std::cout << "S Angle Yaw : " << s_angle_yaw << std::endl;
        std::cout << "=================== RESET OFFSET ===============" << resetOffset << std::endl;

        if (!firstIn){
            m_pass_state = Pass_Forward;
            firstTime = ros::Time::now().toSec();
            firstIn = true;
        }

        timeNow = ros::Time::now().toSec() - firstTime;
        //std::cout<<"TIme Now : "<<timeNow<<std::endl;
        // if (timeNow > 16)//16
        // {
        //     // firstIn = false;
        //     m_pass_state = Pass_Diam;
        //     //return true;
        // }
        //std::cout<<"time now : "<<timeNow<<std::endl;
        


        ROS_INFO("TIME NOW [%f]", timeNow);

        switch (m_pass_state)
        {
        case Pass_Forward:
            ROS_INFO("FFFFFFFFFOOOOOOOOOORWARDDDDDDDDDDDDDDDDDDDDDDDDDD");
            calcWalkingParam(/*MAX_FB_STEP*/ 0.35, -0.03, 0.02);
            if (fabs(s_angle_yaw) > 7/*25 > 3*/)
                m_pass_state = Pass_Rotate;
            break;

        case Pass_Rotate:
            ROS_INFO("RRRRRRRRRRROOOOOOOOOOOOOOTTTTTTTTTTTTTTAAAAAAAAAATTTTTTTTTEEEEEEEEE");
            setWalkingParam((s_angle_yaw > 0) ? 0.01 : 0.01,
                 (s_angle_yaw > 0) ? -0.08 : 0.1,
                  (s_angle_yaw > 0) ? MAX_RL_TURN : -MAX_RL_TURN);
            
            if (fabs(s_angle_yaw) < 7)
                m_pass_state = Pass_Forward;
            break;
        
        case Pass_Diam:
            ROS_INFO("DIAAAMMMMMMMMMMMMM");
            setWalkingParam(0, 0, 0);
            break;

        }

        if (timeNow > 30)//15
        {   
            // timeNow = 0;
            m_pass_state = Pass_Diam;
            firstIn = false;
            return true;
            
        }

        return false;
    }

    bool Behavioral::stableWalk()
    {
        static double firstTime, timeNow;
        ROS_WARN("=========STABLE WALK==========");
        ROS_INFO("TIME NOW [%f]", timeNow);
        ROS_INFO("TIME START [%f]", firstTime);

        std_msgs::String data;
        data.data = "Not_Kicking";

        kick_status_pub_.publish(data);

        std::cout<<"FIRST IN : "<<firstIn<<std::endl;
        if (!firstIn) {
            firstTime = ros::Time::now().toSec();
            ROS_INFO("MASUK SINI");
        }
        firstIn = true;
        timeNow = ros::Time::now().toSec() - firstTime;

        if (timeNow > 2) {
            ROS_ERROR("MASUK TRUE");
            return true;
        }

        // calcWalkingParam(0.12, 0, 0);
        setWalkingParam(0.1, 0.05, -0.1);

        // ROS_INFO("TIME NOW [%f]", timeNow);
        // ROS_INFO("TIME START [%f]", firstTime);

        return false;
    }

    bool Behavioral::WaitingBall()
    {
        

        static double firstTime, timeNow;

        if (!firstWait)
            firstTime = ros::Time::now().toSec();

        firstWait = true;
        timeNow = ros::Time::now().toSec() - firstTime;

        if (timeNow > 2)
            return true;
        // ROS_INFO("TOTAL THETA[%f]", TOTAL_THETA);
        ROS_INFO("TIME NOW [%f]", timeNow);

        return false;
    }

    bool Behavioral::dribblePasser()
    {
        static int m_pass_state = Pass_Forward;
        static double firstTime, timeNow;

        if (!firstIn)
            firstTime = ros::Time::now().toSec();

        firstIn = true;
        timeNow = ros::Time::now().toSec() - firstTime;

        if (timeNow > 15)
            return true;

        ROS_INFO("TIME NOW [%f]", timeNow);

        switch (m_pass_state)
        {
        case Pass_Forward:
            ROS_INFO("FORWARD");
            setWalkingParam(MAX_FB_STEP, 0, 0);
            if (angle_yaw > 15)
                m_pass_state = Pass_Rotate;
            break;

        case Pass_Rotate:
            ROS_INFO("Rotate");
            setWalkingParam(0, 0, (angle_yaw > 0) ? 1 : -1);
            if (angle_yaw < 15)
                m_pass_state = Pass_Forward;
            break;
        }

        return false;
    }

    void Behavioral::RizCheckBalls(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        double RB_THETA = atan2(ball_y, ball_x) * 180 / PI; //Robot to Ball Theta
        std::cout<<"passercheckfollowing"<<std::endl;
        // if(ball_x > 40)
        // {
        //     followState = Following_Forward;
        // }
        if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO && fabs(robot_theta - ball_theta) > ANGLE_TOLERANCE)
            followState = Following_Revo;
        else
        {
           // if (ball_x < 0)
            //     followState = Following_Backward;
            if (fabs(ball_y) > RL_MIN_LIMIT && ball_x < FORWARD_MIN_REVO)
                followState = Following_Shift;
            else if (fabs(RB_THETA) > BALL_TOLERANCE)
                followState = Following_Rotate;
            else
            {
                if (FORWARD_MIN_REVO > ball_x && ball_x > FORWARD_MIN_LIMIT)
                    followState = Following_ForwardMin;
                else
                    followState = Following_Forward;
            }
        }
    }

    bool Behavioral::FarizsBalls(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {   
        
        ROS_INFO("GO TO BALL");
        
        std_msgs::String data;
        data.data = "Not_Kicking";
        static bool rotate = false;
        
        kick_status_pub_.publish(data);

        double s_angle_yaw;
        // if(angle_yaw > 0)
        s_angle_yaw = angle_yaw - resetOffset;
        // else
        // s_angle_yaw = angle_yaw + resetOffset;
        
        std::cout << "Angle Yaw       : " << angle_yaw << std::endl;
        std::cout << "S Angle Yaw       : " << s_angle_yaw << std::endl;
        std::cout << "RESSSEEEEEEEEETTTTTT Yaw       : " << resetOffset << std::endl;
     
        
        std_msgs::Float32 msg;

        msg.data = jarakX;
        ball_proj.publish(msg);

        
        
        double TOTAL_THETA = s_angle_yaw - ball_theta;
        double RB_THETA = atan2((ball_y), fabs(ball_x)) * 180 / PI; //Robot to Ball Theta
      
        // std::cout<<"ball x ["<<ball_x<<"]\n";
        // std::cout<<"ball y ["<<ball_y<<"]\n";
        // std::cout<<"total theta ["<<TOTAL_THETA<<"]\n";
        // std::cout<<"RB_THETA ["<<RB_THETA<<"]\n";
   
        if (ball_x <= FORWARD_MIN_LIMIT && /*((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) */ fabs(ball_y) == RL_MIN_LIMIT && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
        {
            ROS_INFO("APA KAU");
            
            lastKick = true;
       
            ROS_ERROR("KKKKKKKKKKIIIIIIIIIIIIIIIIIIIIICCCCCCCCCCCCCCCCKKKKKKKKKKKKK");
         
                
            return true;
        }
    
        ROS_INFO("BALL X [%f], BALL Y[%f], RB_THETA[%f], TOTAL THETA[%f]", ball_x, ball_y, RB_THETA, TOTAL_THETA);
    
        

        switch (followState)
        {
        case Following_Forward:

            calcWalkingParam(0.3, -0.05, 0.370);

            if(ball_x < 60) {
                if(ball_y > RL_MIN_LIMIT) {
                    followState = Following_Shift;
                }
                else if(fabs(RB_THETA) > REVO_TOLERANCE && ball_x > FORWARD_MIN_REVO) {
                    followState = Following_Revo;
                }
                else if(fabs(RB_THETA) > KICKING_TOLERANCE) {
                    followState = Following_Rotate;
                }
                else {
                    followState = Following_ForwardMin;
                }
            }
        
            break;

        case Following_Rotate:
            
            if((RB_THETA > 0 && RB_THETA <= 90) || RB_THETA < -90) {
                //parame ccw
            }
            else if(RB_THETA < 0 && RB_THETA >= -90) {
                //parame cw

            }
            if (fabs(RB_THETA) <= KICKING_TOLERANCE) {
                if(ball_y > RL_MIN_LIMIT) {
                    followState = Following_Shift;
                }
                else if(fabs(RB_THETA) > REVO_TOLERANCE && ball_x > FORWARD_MIN_REVO) {
                    followState = Following_Revo;
                }
                else if((ball_x > FORWARD_MIN_LIMIT)){
                    followState = Following_ForwardMin;
                }
            }
            break;
            
            

        case Following_Revo:
         
            ROS_INFO("REVO");
            if(TOTAL_THETA > 0) {
                //parame cw
            } else if(TOTAL_THETA <= 0) {
                //parame ccw
            }
            if(fabs(TOTAL_THETA) <= REVO_TOLERANCE) {
                if(ball_y > RL_MIN_LIMIT) {
                    followState = Following_Shift;
                }
                else if(fabs(RB_THETA) > KICKING_TOLERANCE) {
                    followState = Following_Rotate;
                }
                else if((ball_x > FORWARD_MIN_LIMIT)) {
                    followState = Following_ForwardMin;
                }
            }
            
            
                
            break;

        case Following_Shift:
          
            ROS_INFO("SHIFT");
            if(ball_y > 0) {
                //geser kanan
                //parame kanan
            }
            else if(ball_y <=0) {
                //geser kiri
                //parame kiri
            }
            if(fabs(ball_y) <= RL_MIN_LIMIT) {
                if(fabs(RB_THETA) > REVO_TOLERANCE && ball_x > FORWARD_MIN_REVO) {
                    followState = Following_Revo;
                }
                else if(fabs(RB_THETA) > KICKING_TOLERANCE) {
                    followState = Following_Rotate;
                }
                else if(ball_x > FORWARD_MIN_LIMIT){
                    followState = Following_ForwardMin;
                }
            }
          
            
            break;

        case Following_ForwardMin:
         
            ROS_INFO("FORWARD MIN");
            //parame

            if(ball_x <= FORWARD_MIN_LIMIT) {
                if(ball_y > RL_MIN_LIMIT) {
                    followState = Following_Shift;
                }
                else if(fabs(RB_THETA) > REVO_TOLERANCE && ball_x > FORWARD_MIN_REVO) {
                    followState = Following_Revo;
                }
                else if(fabs(RB_THETA) > KICKING_TOLERANCE) {
                    followState = Following_Rotate;
                }
            }

         
            

        case Following_Backward:
           
            ROS_INFO("BACKWARD");
        
            break;

        case Following_Stop:
           
            ROS_INFO("READY TO KICK");
           
            lastKick = true;
            kickTimeStart = ros::Time::now().toSec();
           
            ROS_ERROR("KKKKKKKKKKIIIIIIIIIIIIIIIIIIIIICCCCCCCCCCCCCCCCKKKKKKKKKKKKK");
           
            //setWalkingParam(0, 0, 0);
            return true;   
        default:
           
            ROS_INFO("GAMASUK STATE WOYYYY");
           
            followState = Following_ForwardMin;
            break;  
        }

        return false;
    }

    bool Behavioral::yourBalls_3(double robot_theta, double ball_x, double ball_y, double ball_theta) 
    {
        ROS_INFO("GO TO BALL");
        std_msgs::String data;
        data.data = "Not_Kicking";
        static bool rotate = false;
        kick_status_pub_.publish(data);

        static std::vector<int> ball_x_ma,ball_y_ma;


        int size_buff = 15;
        double ball_x_filt,ball_y_filt;
 
        ball_x_ma.insert(ball_x_ma.end(), ball_x);
        ball_y_ma.insert(ball_y_ma.end(), ball_y);

        double s_angle_yaw;
        if(resetOffset > 0){
            if(angle_yaw > 0)
            {
                s_angle_yaw = angle_yaw - resetOffset;
            }     
            else{
                s_angle_yaw = angle_yaw + resetOffset;
            }
        }
        else {
            if(angle_yaw > 0)
            {
                s_angle_yaw = angle_yaw + resetOffset;
            }     
            else{
                s_angle_yaw = angle_yaw - resetOffset;
            }
        }
        
        
        
        std::cout << "Angle Yaw       : " << angle_yaw << std::endl;
        std::cout << "S Angle Yaw       : " << s_angle_yaw << std::endl;
        std::cout << "RESSSEEEEEEEEETTTTTT Yaw       : " << resetOffset << std::endl;
     
        if(ball_x_ma.size() < size_buff){
            ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / ball_x_ma.size());
            ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / ball_y_ma.size());
        }else{
            ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / size_buff);
            ball_x_ma.erase(ball_x_ma.begin());
            ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / size_buff);
            ball_y_ma.erase(ball_y_ma.begin());
        }

        std_msgs::Float32 msg;

        msg.data = jarakX;
        ball_proj.publish(msg);

        // ball_x_filt = fabs(ball_x_filt);
        ball_x = ball_x_filt - 9.6; //-10 //arya ngurangin 10
        ball_y = ball_y_filt;
        
        double TOTAL_THETA = s_angle_yaw - ball_theta;
        double RB_THETA = atan2((ball_y), fabs(ball_x)) * 180 / PI; //Robot to Ball Theta

        double gain_theta = 0.865;
   
        if (ball_x <= FORWARD_MIN_LIMIT && /*((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) */ fabs(ball_y) == RL_MIN_LIMIT && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
        {
            ROS_INFO("APA KAU");
            lastKick = true;
       
            ROS_ERROR("KKKKKKKKKKIIIIIIIIIIIIIIIIIIIIICCCCCCCCCCCCCCCCKKKKKKKKKKKKK");
            return true;
        }
    
        ROS_INFO("BALL X [%f], BALL Y[%f], RB_THETA[%f], TOTAL THETA[%f]", ball_x, ball_y, RB_THETA, TOTAL_THETA);
        
        static int counterAction = 0; 

        // if(followState == lastFollowState && followState != Following_Rotate){
        //     lastFollowState = followState;
        //     walkDelay = true;

        switch (followState)
        {
        case Following_Forward:
        
            ROS_INFO("FORWARD");
            
            // std::cout<<"<<=======FORWARD========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(MAX_FB_STEP, 0, 0);
            counterAction++;
            ROS_INFO("Cnt Action: %d",counterAction );
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            } 
            if(ball_x > 40){ //40 //60
            // reset counterAction
            counterAction = 0;
                // calcWalkingParam(/*MAX_FB_STEP*/ 0.3, -0.1, 0.00);//-0.55
                // calcWalkingParam(0.3, 0.1 + (gain_theta*s_angle_yaw*0.4), -0.3 + (-gain_theta*s_angle_yaw));
                // calcWalkingParam(0.3, -0.05, 0.37); //mei 2024
                // setWalkingParam(/*MAX_FB_STEP*/ 0.35, 0, 0);

                
              
                std::cout << "romi babiiiiiiiiiiiiii" << std::endl;
                setWalkingParam(0.1, 0.04, -0.24);


                if(fabs(RB_THETA) > 1.5*BALL_TOLERANCE) {
                    followState = Following_Rotate;
                    break;
                }
         
            } else if(counterAction>20){ //baru nammbah 23 Mei jam 6.34 pagi
                // else {
                //     calcWalkingParam(0.25, -0.02, 0.1);
                //     std::cout << "romi nggak babiiiiii" << std::endl;
                // }
                // else
                // {
                //     if (fabs(RB_THETA) > BALL_TOLERANCE)
                //     {
                //         // followState = Following_OnTheSpot;
                //         // BALL_THETA = ball_theta;
                //         followState = Following_Rotate;
                //     }
                //     else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                //         followState = Following_Revo;
                //     else if (ball_x < FORWARD_MIN_REVO)
                //         followState = Following_ForwardMin;
                // }

                // else if (fabs(RB_THETA) > BALL_TOLERANCE)
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                {
                    // followState = Following_OnTheSpot;
                    // BALL_THETA = ball_theta;
                    followState = Following_Rotate;
                }
                else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                    followState = Following_Revo;
                else if (ball_x < FORWARD_MIN_REVO)
                    followState = Following_ForwardMin;
                else {
                    // calcWalkingParam(0.15, 0.18, 0.1);
                    // calcWalkingParam(MIN_FB_STEP,0.14, -0.06);
                    // calcWalkingParam(0.15, 0.18, 0.1
                    // calcWalkingParam(0.3, 0.1 + (gain_theta*s_angle_yaw*0.4), -0.3 + (-gain_theta*s_angle_yaw));
                    // calcWalkingParam(0.3, -0.05, 0.37); mei 2024
                    // setWalkingParam(0.3, 0.04, -0.24);
                    setWalkingParam(0.1, 0.04, -0.24);

                    std::cout << "romi nggak babiiiiii" << std::endl;
                  
                }
                // else if(/*ball_x <= 60*/counterAction > 5){
                    
                //     followState = Following_Stop;
                // }
                // else{
                //     ROS_ERROR("BALL NOT FOUND");
                // }
            
                
            }

            break;
        

        case Following_Rotate:
            
            // std::cout<<"<<========ROTATE========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, 0, (RB_THETA < 0) ? -MAX_RL_TURN : MAX_RL_TURN);
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=20 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -20)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            }  

            if(/*ball_x < 40 ||*/ (rotate)){
              
                ROS_INFO("ROTATE_ANJAY");

                if(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90)) {
                    ROS_INFO("ROTATE CCW");
                } else {
                    ROS_INFO("ROTATE CW");
                }
             
                // if(fabs(ball_y) < 12 && ball_x < 20)
                //     rotate = false;
                // else if(fabs(ball_y) < 50 && ball_x > 100)
                //     rotate = false;
                if(ball_x > 70 && fabs(RB_THETA) <= BALL_TOLERANCE) //30 //60
                    rotate = false;
                setWalkingParam(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 0.0: -0.002,
                 ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -0.1:0.2,
                  ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 1.00 : -1.0); // CCW : CW
                // setWalkingParam(((TOTAL_THETA > 0 && TOTAL_THETA < 90) || TOTAL_THETA <= -90) ? 0.1 : 0.12, //nilai RB_THETA nya gajelas kemarin
                //  ((TOTAL_THETA > 0 && TOTAL_THETA < 90) || TOTAL_THETA <= -90) ? -0.15 : 0.07,
                //   ((TOTAL_THETA > 0 && RB_THETA < 90) || TOTAL_THETA <= -90) ? MAX_RL_TURN : -MAX_RL_TURN);
            
            }
            // else if(ball_x < 20)
            //     followState = Following_ForwardMin;
            else if(ball_x > 160){ // TAQY NAMBAH Y
               
                ROS_INFO("ROTATE_FORWARD_JAUUHHHHH");
              
                if(fabs(ball_y) > 125)
                    rotate = true;
                setWalkingParam(/*MAX_FB_STEP*/ 0.3,
                 ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 0.15 : -0.15/*0.05*/,
                 ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -1 : 1);
            }
            else {
             
                
                if(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90)) {
                    ROS_INFO("ROTATE_FORWARD CCW");
                } else {
                    ROS_INFO("ROTATE_FORWARD CW");
                }
           
                if(fabs(ball_y) > 30)
                    followState = Following_Shift;
                else if(fabs(RB_THETA) > BALL_TOLERANCE)
                    rotate = true;
                else if(ball_x < 60 && ball_x < 60)
                    followState = Following_ForwardMin;
                else if (ball_x <= 14 && fabs(ball_y)>=4)
                    followState = Following_Shift;
                else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                    followState = Following_Revo;
                setWalkingParam(0.25,
                 ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -0.2/*0.05*/ : 0.2 ,  //nilai RB_THETA nya gajelas kemarin
                ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 1.0 : -1);
                // if(fabs(RB_THETA) > BALL_TOLERANCE) {
                //     rotate = true;
                // }
                // if(fabs(ball_y) >= 20) {
                //     followState = Following_Shift;
                // }
                // else if(ball_x > 25) {
                //     followState = Following_Forward;
                // }
                // else {
                //     followState = Following_Stop;
                // }
            }



            if (fabs(RB_THETA) < BALL_TOLERANCE)
                followState = false;
            // else if (ball_x <= 14 && fabs(ball_y)>=4)
            //     followState = Following_Shift;
            // else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
            //     followState = Following_Revo;
            break;

        case Following_Revo:
         
            ROS_INFO("REVO");
            
            // std::cout<<"<<=========REVO=========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, ((robot_theta - ball_theta) < 0) ? REVO_CW_STEP : REVO_CCW_STEP, ((robot_theta - ball_theta) < 0) ? REVO_CW_TURN : REVO_CCW_TURN)
            // repo_tt = true;
            // if (TOTAL_THETA > 0)
            //     setWalkingParam(0.0, -0.3, 0.8); //CCW
            // else if(TOTAL_THETA == 0)
            //     repo_tt = false; 
            // else 
            //      //CW
            //     setWalkingParam(0.05, 0.3, -0.6);            
            //     // followState = Following_Shift;

            if (TOTAL_THETA > 0){
                ROS_INFO("============ CCW ============");
                setWalkingParam(0.15, -0.3, 0.5);
            } 
            else if(TOTAL_THETA < 0){
                ROS_INFO("============ CW ============");
                setWalkingParam(0.1, 0.2, -0.5);
            }
                // repo_tt = false; 
            // else 
                 //CW
                // followState = Following_Shift;

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_ForwardMin;
            }
            else
            {
                if (fabs(TOTAL_THETA) < ANGLE_TOLERANCE)
                {
                    // if (ball_x < 5 && fabs(ball_y)< 11)
                    //     followState = Following_Backward;
                    if (fabs(ball_y) > RL_MIN_LIMIT)
                        followState = Following_Shift;
                    else
                        followState = Following_ForwardMin;
                }
                // else if (ball_x < 1 && fabs(ball_y)< 6)
                //     followState = Following_Backward;
            }
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            }
                
            break;

        case Following_Shift:
          
            ROS_INFO("SHIFT");
          
            // std::cout<<"<<========SHIFT========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);
            // setWalkingParam(-0.05, (ball_y > 1) ? MAX_RL_STEP : -MAX_RL_STEP, (ball_y > 1) ? 0.4 : -0.1);

            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            }
            if(ball_y < -1) //-5
            {   
              
                ROS_ERROR("GESER KANAN");
            
                // setWalkingParam(0.1 , -0.1, -0.65);
                setWalkingParam(-0.1, -0.23, -0.3);
                if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
                {
                    // walkNow = true;
                    followState = Following_Stop;
                    break;
                }
            }
            
            else if(ball_y > 1) //5
            {   
              
                ROS_ERROR("GESER KIRI");
              
                // setWalkingParam(0.1, /*MAX_RL_STEP*/ 0.1, 0.4);
                setWalkingParam(-0.06, 0.2, 0.450);
                if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
                {
                    // walkNow = true;
                    followState = Following_Stop;
                    break;
                }

            }

            if(fabs(ball_y) <= RL_MIN_LIMIT && ball_x > FORWARD_MIN_REVO) {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_ForwardMin;
            }
            // if (ball_x > FORWARD_MIN_REVO)
            // {
            //     if (fabs(RB_THETA) > BALL_TOLERANCE)
            //         followState = Following_Rotate;
            //     else
            //         followState = Following_ForwardMin;
            // }
            else if (ball_x > 0 && fabs(ball_y) < RL_MIN_LIMIT)
            {
                if (ball_x > FORWARD_MIN_LIMIT)
                    followState = Following_ForwardMin;
                else if (fabs(TOTAL_THETA) > ANGLE_TOLERANCE)
                    followState = Following_Revo;
            }
            // else if (ball_x < 0)
            //     followState = Following_Backward;
            break;

        case Following_ForwardMin:
         
            ROS_INFO("FORWARD MIN");
         
            // std::cout<<"<<=====FORWARD MIN=====>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(MIN_FB_STEP,0, 0);
            // calcWalkingParam(0.15, 0.18, 0.1
            // calcWalkingParam(0.2, 0.1 + (gain_theta*s_angle_yaw*0.4), (-gain_theta*s_angle_yaw));
            // calcWalkingParam(0.2, 0.01, -0.5); //mwi 2024
            setWalkingParam(0.15, 0.04, -0.24);
            //coba ganti nambahin arya
            if (ball_x <= FORWARD_MIN_LIMIT && ((ball_y<=13 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -13)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)//dibesarin
            {
                // walkNow = true;
                
                followState = Following_Stop;
                break;
            }

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (fabs(ball_y) >= (RL_MIN_LIMIT + 4) && ball_x <= (FORWARD_MIN_LIMIT + 1))
                followState = Following_Shift;
            break;

        case Following_Backward:
           
            ROS_INFO("BACKWARD");
        
            // std::cout<<"<<=======BACKWARD======>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(MAX_BW_STEP, 0, 0);
            setWalkingParam(MAX_BW_STEP,0,0.3);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_ForwardMin;
            }
            else if (fabs(ball_x) <= FORWARD_MIN_REVO && fabs(ball_y) >= 5)
                followState = Following_Shift;
            else if (fabs(ball_x <= FORWARD_MIN_REVO))
                followState = Following_Revo;
            break;

        case Following_Stop:
           
            ROS_INFO("READY TO KICK");
           
            lastKick = true;
            kickTimeStart = ros::Time::now().toSec();
           
            // ROS_ERROR("KKKKKKKKKKIIIIIIIIIIIIIIIIIIIIICCCCCCCCCCCCCCCCKKKKKKKKKKKKK");
           
            //setWalkingParam(0, 0, 0);
            return true;   
        default:
           
            ROS_INFO("GAMASUK STATE WOYYYY");
           
            followState = Following_ForwardMin;
            break;  
        }
        return false;
    }

    bool Behavioral::yourBalls_2(double robot_theta, double ball_x, double ball_y, double ball_theta) 
    {
        ROS_INFO("GO TO BALL");
        // ball_ = ball_y - 5;
        std_msgs::String data;
        data.data = "Not_Kicking";
        static bool rotate = false;
        // double walkDelay, walkStart;
        // static bool walkNow = false;
        kick_status_pub_.publish(data);

        static std::vector<int> ball_x_ma,ball_y_ma;


        int size_buff = 15;
        double ball_x_filt,ball_y_filt;
 
        ball_x_ma.insert(ball_x_ma.end(), ball_x);
        ball_y_ma.insert(ball_y_ma.end(), ball_y);

        double s_angle_yaw;
        if(resetOffset > 0){
            if(angle_yaw > 0)
            {
                s_angle_yaw = angle_yaw - resetOffset;
            }     
            else{
                s_angle_yaw = angle_yaw + resetOffset;
            }
        }
        else {
            if(angle_yaw > 0)
            {
                s_angle_yaw = angle_yaw + resetOffset;
            }     
            else{
                s_angle_yaw = angle_yaw - resetOffset;
            }
        }
        
        
        
        std::cout << "Angle Yaw       : " << angle_yaw << std::endl;
        std::cout << "S Angle Yaw       : " << s_angle_yaw << std::endl;
        std::cout << "RESSSEEEEEEEEETTTTTT Yaw       : " << resetOffset << std::endl;
     
        if(ball_x_ma.size() < size_buff){
            ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / ball_x_ma.size());
            ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / ball_y_ma.size());
        }else{
            ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / size_buff);
            ball_x_ma.erase(ball_x_ma.begin());
            ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / size_buff);
            ball_y_ma.erase(ball_y_ma.begin());
        }

        std_msgs::Float32 msg;

        msg.data = jarakX;
        ball_proj.publish(msg);

        // ball_x_filt = fabs(ball_x_filt);
        ball_x = ball_x_filt - 9.6; //-10 //arya ngurangin 10
        ball_y = ball_y_filt;
        
        double TOTAL_THETA = s_angle_yaw - ball_theta;
        double RB_THETA = atan2((ball_y), fabs(ball_x)) * 180 / PI; //Robot to Ball Theta

        double gain_theta = 0.865;

        // std::cout<<"ball x ["<<ball_x<<"]\n";
        // std::cout<<"ball y ["<<ball_y<<"]\n";
        // std::cout<<"total theta ["<<TOTAL_THETA<<"]\n";
        // std::cout<<"RB_THETA ["<<RB_THETA<<"]\n";
   
        if (ball_x <= FORWARD_MIN_LIMIT-8 && /*((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) */ fabs(ball_y) == RL_MIN_LIMIT && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
        {
            ROS_INFO("APA KAU");
            // walkNow = true;
            // followState = Following_Stop;
            lastKick = true;

            ROS_ERROR("KKKKKKKKKKIIIIIIIIIIIIIIIIIIIIICCCCCCCCCCCCCCCCKKKKKKKKKKKKK");
         
                //setWalkingParam(0, 0, 0);
            return true;
        }
    
        ROS_INFO("BALL X [%f], BALL Y[%f], RB_THETA[%f], TOTAL THETA[%f]", ball_x, ball_y, RB_THETA, TOTAL_THETA);
        
        static int counterAction = 0; 

        // if(followState == lastFollowState && followState != Following_Rotate){
        //     lastFollowState = followState;
        //     walkDelay = true;

        switch (followState)
        {
        case Following_Forward:
        
            ROS_INFO("FORWARD");
            
            // std::cout<<"<<=======FORWARD========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(MAX_FB_STEP, 0, 0);
            counterAction++;
            ROS_INFO("Cnt Action: %d",counterAction );
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            } 
            if(ball_x > 40){ //40 //60
            // reset counterAction
            counterAction = 0;
                // calcWalkingParam(/*MAX_FB_STEP*/ 0.3, -0.1, 0.00);//-0.55
                // calcWalkingParam(0.3, 0.1 + (gain_theta*s_angle_yaw*0.4), -0.3 + (-gain_theta*s_angle_yaw));
                // calcWalkingParam(0.3, -0.05, 0.37); //mei 2024
                // setWalkingParam(/*MAX_FB_STEP*/ 0.35, 0, 0);

                
              
                std::cout << "romi babiiiiiiiiiiiiii" << std::endl;
                setWalkingParam(0.25, 0.0, 0.0 );


                if(fabs(RB_THETA) > 1.5*BALL_TOLERANCE) {
                    followState = Following_Rotate;
                    break;
                }
         
            } else if(counterAction>20){ //baru nammbah 23 Mei jam 6.34 pagi
                // else {
                //     calcWalkingParam(0.25, -0.02, 0.1);
                //     std::cout << "romi nggak babiiiiii" << std::endl;
                // }
                // else
                // {
                //     if (fabs(RB_THETA) > BALL_TOLERANCE)
                //     {
                //         // followState = Following_OnTheSpot;
                //         // BALL_THETA = ball_theta;
                //         followState = Following_Rotate;
                //     }
                //     else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                //         followState = Following_Revo;
                //     else if (ball_x < FORWARD_MIN_REVO)
                //         followState = Following_ForwardMin;
                // }

                // else if (fabs(RB_THETA) > BALL_TOLERANCE)
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                {
                    // followState = Following_OnTheSpot;
                    // BALL_THETA = ball_theta;
                    followState = Following_Rotate;
                }
                else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                    followState = Following_Revo;
                else if (ball_x < FORWARD_MIN_REVO)
                    followState = Following_ForwardMin;
                else {
                    // calcWalkingParam(0.15, 0.18, 0.1);
                    // calcWalkingParam(MIN_FB_STEP,0.14, -0.06);
                    // calcWalkingParam(0.15, 0.18, 0.1
                    // calcWalkingParam(0.3, 0.1 + (gain_theta*s_angle_yaw*0.4), -0.3 + (-gain_theta*s_angle_yaw));
                    // calcWalkingParam(0.3, -0.05, 0.37); mei 2024
                    // setWalkingParam(0.3, 0.04, -0.24);
                    setWalkingParam(0.1, 0.04, -0.24);

                    std::cout << "romi nggak babiiiiii" << std::endl;
                  
                }
                // else if(/*ball_x <= 60*/counterAction > 5){
                    
                //     followState = Following_Stop;
                // }
                // else{
                //     ROS_ERROR("BALL NOT FOUND");
                // }
            
                
            }

            break;
        

        case Following_Rotate:
            
            // std::cout<<"<<========ROTATE========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, 0, (RB_THETA < 0) ? -MAX_RL_TURN : MAX_RL_TURN);
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=20 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -20)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            }  

            if(/*ball_x < 40 ||*/ (rotate)){
              
                ROS_INFO("ROTATE_ANJAY");

                if(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90)) {
                    ROS_INFO("ROTATE CCW");
                    setWalkingParam(0.01, -0.050, 0.6);
                } else {
                    ROS_INFO("ROTATE CW");
                    setWalkingParam(-0.03, 0.050, -0.6);
                }
             
                // if(fabs(ball_y) < 12 && ball_x < 20)
                //     rotate = false;
                // else if(fabs(ball_y) < 50 && ball_x > 100)
                //     rotate = false;
                if(ball_x > 70 && fabs(RB_THETA) <= BALL_TOLERANCE) //30 //60
                    rotate = false;
                // setWalkingParam(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 0.0: -0.002,
                //  ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -0.1:0.2,
                //   ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 1.00 : -1.0); // CCW : CW
                // setWalkingParam(((TOTAL_THETA > 0 && TOTAL_THETA < 90) || TOTAL_THETA <= -90) ? 0.1 : 0.12, //nilai RB_THETA nya gajelas kemarin
                //  ((TOTAL_THETA > 0 && TOTAL_THETA < 90) || TOTAL_THETA <= -90) ? -0.15 : 0.07,
                //   ((TOTAL_THETA > 0 && RB_THETA < 90) || TOTAL_THETA <= -90) ? MAX_RL_TURN : -MAX_RL_TURN);
            
            }
            // else if(ball_x < 20)
            //     followState = Following_ForwardMin;
            // else if(ball_x > 160){ // TAQY NAMBAH Y
               
            //     ROS_INFO("ROTATE_FORWARD_JAUUHHHHH");
              
            //     if(fabs(ball_y) > 125)
            //         rotate = true;
            //     setWalkingParam(/*MAX_FB_STEP*/ 0.3,
            //      ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 0.15 : -0.15/*0.05*/,
            //      ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -1 : 1);
            // }
            else {
             
                
                if(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90)) {
                    ROS_INFO("ROTATE_FORWARD CCW");
                } else {
                    ROS_INFO("ROTATE_FORWARD CW");
                }
           
                if(fabs(ball_y) > 30)
                    followState = Following_Shift;
                else if(fabs(RB_THETA) > BALL_TOLERANCE)
                    rotate = true;
                else if(ball_x < 60 && ball_x < 60)
                    followState = Following_ForwardMin;
                else if (ball_x <= 14 && fabs(ball_y)>=4)
                    followState = Following_Shift;
                else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                    followState = Following_Revo;
                // setWalkingParam(0.25,
                //  ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -0.2/*0.05*/ : 0.2 ,  //nilai RB_THETA nya gajelas kemarin
                // ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 1.0 : -1);
            }



            if (fabs(RB_THETA) < BALL_TOLERANCE)
                followState = false;
            break;

        case Following_Revo:
         
            ROS_INFO("REVO");
            
            // std::cout<<"<<=========REVO=========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, ((robot_theta - ball_theta) < 0) ? REVO_CW_STEP : REVO_CCW_STEP, ((robot_theta - ball_theta) < 0) ? REVO_CW_TURN : REVO_CCW_TURN)
            // repo_tt = true;
            // if (TOTAL_THETA > 0)
            //     setWalkingParam(0.0, -0.3, 0.8); //CCW
            // else if(TOTAL_THETA == 0)
            //     repo_tt = false; 
            // else 
            //      //CW
            //     setWalkingParam(0.05, 0.3, -0.6);            
            //     // followState = Following_Shift;

            if (TOTAL_THETA > 0){
                ROS_INFO("============ CCW ============");
                setWalkingParam(0.0, -0.180, 0.300);
            } 
            else if(TOTAL_THETA < 0){
                ROS_INFO("============ CW ============");
                setWalkingParam(0.0, 0.1, 0.0);
            }

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_ForwardMin;
            }
            else
            {
                if (fabs(TOTAL_THETA) < ANGLE_TOLERANCE)
                {
                    if (fabs(ball_y) > RL_MIN_LIMIT)
                        followState = Following_Shift;
                    else
                        followState = Following_ForwardMin;
                }
            }
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            }
                
            break;

        case Following_Shift:
          
            ROS_INFO("SHIFT");
          
            // std::cout<<"<<========SHIFT========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);
            // setWalkingParam(-0.05, (ball_y > 1) ? MAX_RL_STEP : -MAX_RL_STEP, (ball_y > 1) ? 0.4 : -0.1);

            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                followState = Following_Stop;
                break;
            }
            if(ball_y < -1) //-5
            {   
              
                ROS_ERROR("GESER KANAN");
            
                setWalkingParam(-0.050, -0.200, -0.500);
                if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
                {
                    // walkNow = true;
                    followState = Following_Stop;
                    break;
                }
            }
            
            else if(ball_y > 1) //5
            {   
                ROS_ERROR("GESER KIRI");
                setWalkingParam(0.0, 0.160, 0.7);
                if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
                {
                    // walkNow = true;
                    followState = Following_Stop;
                    break;
                }

            }

            if(fabs(ball_y) <= RL_MIN_LIMIT && ball_x > FORWARD_MIN_REVO) {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_ForwardMin;
            }
            else if (ball_x > 0 && fabs(ball_y) < RL_MIN_LIMIT)
            {
                if (ball_x > FORWARD_MIN_LIMIT)
                    followState = Following_ForwardMin;
                else if (fabs(TOTAL_THETA) > ANGLE_TOLERANCE)
                    followState = Following_Revo;
            }
            break;

        case Following_ForwardMin:
         
            ROS_INFO("FORWARD MIN");
         
            // std::cout<<"<<=====FORWARD MIN=====>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(MIN_FB_STEP,0, 0);
            // calcWalkingParam(0.15, 0.18, 0.1
            // calcWalkingParam(0.2, 0.1 + (gain_theta*s_angle_yaw*0.4), (-gain_theta*s_angle_yaw));
            // calcWalkingParam(0.2, 0.01, -0.5); //mwi 2024
            setWalkingParam(0.065, 0.0, 0.0 );
            //coba ganti nambahin arya
            if (ball_x <= FORWARD_MIN_LIMIT && ((ball_y<=13 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -13)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)//dibesarin
            {
                // walkNow = true;
                
                followState = Following_Stop;
                break;
            }

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (fabs(ball_y) >= (RL_MIN_LIMIT + 4) && ball_x <= (FORWARD_MIN_LIMIT + 1))
                followState = Following_Shift;
            break;
        case Following_Stop:
           
            ROS_INFO("READY TO KICK");
           
            lastKick = true;
            kickTimeStart = ros::Time::now().toSec();
           
            // ROS_ERROR("KKKKKKKKKKIIIIIIIIIIIIIIIIIIIIICCCCCCCCCCCCCCCCKKKKKKKKKKKKK");
           
            //setWalkingParam(0, 0, 0);
            return true;   
        default:
           
            ROS_INFO("GAMASUK STATE WOYYYY");
           
            followState = Following_ForwardMin;
            break;  
        }

        return false;
    }

    void Behavioral::yourBalls(double robot_theta, double ball_x, double ball_y, double ball_theta) {
        // A modified version of GoingToBall(). This function used for ball dribble and after some specified time, kick the ball.
        ROS_INFO("GO TO BALL");
        std_msgs::String data;
        data.data = "Not_Kicking";
        static bool rotate = false;
        double s_angle_yaw;
        double gain_theta = 0.865;
        // resetOffset = m_yaw;
        kick_status_pub_.publish(data);

            ball_x = tracker_.ballPos_x;
            ball_y = tracker_.ballPos_y;
            std::cout << "ball_x: " << ball_x << std::endl;
            std::cout << "ball_y: " << ball_y << std::endl;

            static std::vector<int> ball_x_ma, ball_y_ma;

            int size_buff = 15;
            double ball_x_filt,ball_y_filt;
    
            ball_x_ma.insert(ball_x_ma.end(), ball_x);
            ball_y_ma.insert(ball_y_ma.end(), ball_y);

            
            if(resetOffset > 0){
                if(angle_yaw > 0)
                {
                    s_angle_yaw = angle_yaw - resetOffset;
                }     
                else{
                    s_angle_yaw = angle_yaw + resetOffset;
                }
            }
            else {
                if(angle_yaw > 0)
                {
                    s_angle_yaw = angle_yaw + resetOffset;
                }     
                else{
                    s_angle_yaw = angle_yaw - resetOffset;
                }
            }
            
            std::cout << "Angle Yaw       : " << angle_yaw << std::endl;
            std::cout << "S Angle Yaw     : " << s_angle_yaw << std::endl;
            std::cout << "Reset Yaw       : " << resetOffset << std::endl;
        
            if(ball_x_ma.size() < size_buff){
                ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / ball_x_ma.size());
                ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / ball_y_ma.size());
            } else{
                ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / size_buff);
                ball_x_ma.erase(ball_x_ma.begin());
                ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / size_buff);
                ball_y_ma.erase(ball_y_ma.begin());
            }

            std_msgs::Float32 msg;

            msg.data = jarakX;
            ball_proj.publish(msg);

            // Ball Position ------------------------------------------------------
            ball_x = ball_x_filt - 9.6;
            ball_y = ball_y_filt;
            double TOTAL_THETA = s_angle_yaw - ball_theta;
            double RB_THETA = atan2((ball_y), fabs(ball_x)) * 180 / PI;

        while (ball_x >= FORWARD_MIN_LIMIT && fabs(ball_y) != RL_MIN_LIMIT && fabs(TOTAL_THETA) > KICKING_TOLERANCE) {
            


            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else {
                    followState = Following_Forward;
                }
                    
            }
            else if (fabs(ball_y) >= (RL_MIN_LIMIT + 4) && ball_x <= (FORWARD_MIN_LIMIT + 1))
                followState = Following_Shift;

            // Following Case --------------------------------------------------------------------------------------------s
            if (followState == Following_Rotate) 
            {
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=20 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -20)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                followState = Following_Stop;
                break;
            }


            if(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90)) {
                ROS_INFO("ROTATE CCW");
            } else {
                ROS_INFO("ROTATE CW");
            }

            if(ball_x > 70 && fabs(RB_THETA) <= BALL_TOLERANCE)
                setWalkingParam(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 0.0: -0.002,
                 ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -0.1:0.2,
                  ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 1.00 : -1.0); // CCW : CW

            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE) {
                followState = Following_Forward;
            }
            else if(ball_y > RL_MIN_LIMIT || ball_x <= 14 && fabs(ball_y) >= 4) {
                followState = Following_Shift;
            }

            if(fabs(ball_y) > 30)
                followState = Following_Shift;
            else if(fabs(RB_THETA) > BALL_TOLERANCE)
                followState = Following_Rotate;
            else if(ball_x < 60 && ball_x < 60)
                followState = Following_Forward;
            else if (ball_x <= 14 && fabs(ball_y)>=4)
                followState = Following_Shift;
            break;
            }

            // FORWARD ------------------------------------------------------------------------
            else if (followState == Following_Forward)
            {
            ROS_INFO("Forward");
            setWalkingParam(0.15, 0.05, -0.1);
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                followState = Following_Stop;
                break;
            }
            // ros::Duration(1).sleep();
            if(fabs(ball_y) >= (RL_MIN_LIMIT + 4) && ball_x <= (FORWARD_MIN_LIMIT + 1)) {
                followState = Following_Shift;
            }
            else if(fabs(RB_THETA) > BALL_TOLERANCE) {
                followState = Following_Rotate;
            }
            break;
            }
            // SHIFT ------------------------------------------------------
            else if(followState == Following_Shift)
            {
                if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
                {
                    followState = Following_Stop;
                    break;
                }

                if(ball_y < -1)
                    {   
                        ROS_INFO("GESER KANAN");
                        // setWalkingParam(0.1 , -0.1, -0.65);
                        setWalkingParam(-0.08, -0.1, -0.3);
                    }
                else if(ball_y > 1)
                    {   
                        ROS_INFO("GESER KIRI");
                        // setWalkingParam(0.1, /*MAX_RL_STEP*/ 0.1, 0.4);
                        setWalkingParam(-0.05, 0.1, 0.4);
                    }

                if(fabs(ball_y) <= RL_MIN_LIMIT && ball_x > FORWARD_MIN_REVO) {
                    if (fabs(RB_THETA) > BALL_TOLERANCE)
                        followState = Following_Rotate;
                    else
                        followState = Following_Forward;
                }
                else if (ball_x > 0 && fabs(ball_y) < RL_MIN_LIMIT)
                {
                    if (ball_x > FORWARD_MIN_LIMIT)
                        followState = Following_Forward;
                }
            }
            else if(followState == Following_Stop) {

                ROS_INFO("Jalan di Tempat");
            
                lastKick = true;
                kickTimeStart = ros::Time::now().toSec();
                
                setWalkingParam(0, 0, 0);
                break;
            }
        }   

    }

    bool Behavioral::myBalls(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {

        // A modified version of GoingToBall(). This function used for ball dribble and after some specified time, kick the ball.
        // Integrated with passerA. Non-blocking function.
        // The condition of the followState switch case is copied from GoingToBall()

        ROS_INFO("GO TO BALL");
        std_msgs::String data;
        data.data = "Not_Kicking";
        static bool rotate = false;
        double s_angle_yaw;
        double gain_theta = 0.865;
        kick_status_pub_.publish(data);
        
        // std::cout << "ball_x: " << ball_x << std::endl;
        // std::cout << "ball_y: " << ball_y << std::endl;

        static std::vector<int> ball_x_ma, ball_y_ma;

        int size_buff = 15;
        double ball_x_filt,ball_y_filt;

        ball_x_ma.insert(ball_x_ma.end(), ball_x);
        ball_y_ma.insert(ball_y_ma.end(), ball_y);

        
        if(resetOffset > 0){
            if(angle_yaw > 0)
            {
                s_angle_yaw = angle_yaw - resetOffset;
            }     
            else{
                s_angle_yaw = angle_yaw + resetOffset;
            }
        }
        else {
            if(angle_yaw > 0)
            {
                s_angle_yaw = angle_yaw + resetOffset;
            }     
            else{
                s_angle_yaw = angle_yaw - resetOffset;
            }
        }
        
        std::cout << "Angle Yaw       : " << angle_yaw << std::endl;
        std::cout << "S Angle Yaw     : " << s_angle_yaw << std::endl;
        std::cout << "Reset Yaw       : " << resetOffset << std::endl;

        std::cout << "Odometry Y: "<< getOdometry().y() << std::endl;
    
        if (ball_x_ma.size() < size_buff) {
            ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / ball_x_ma.size());
            ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / ball_y_ma.size());
        } else{
            ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / size_buff);
            ball_x_ma.erase(ball_x_ma.begin());
            ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / size_buff);
            ball_y_ma.erase(ball_y_ma.begin());
        }

        std_msgs::Float32 msg;

        msg.data = jarakX;
        ball_proj.publish(msg);

        // Ball Position ------------------------------------------------------
        ball_x = ball_x_filt - 9.6;
        ball_y = ball_y_filt;
        double TOTAL_THETA = s_angle_yaw - ball_theta;
        double RB_THETA = atan2((ball_y), fabs(ball_x)) * 180 / PI;

        ROS_INFO("BALL X [%f], BALL Y[%f], RB_THETA[%f], TOTAL THETA[%f]", ball_x, ball_y, RB_THETA, TOTAL_THETA);

        int deltaTime = ros::Time::now().toSec() - timeAwal;

        // Used as Timer
        int duration = 10;
        if (deltaTime < duration && ball_x <= FORWARD_MIN_LIMIT && fabs(ball_y) == RL_MIN_LIMIT && fabs(TOTAL_THETA) < KICKING_TOLERANCE) {
            // Kick after the duration elapsed. Return True and passed to PasserFollowing(). Kicking condition on that function.
            return true;
        }

        switch (followState)
        {
        case Following_Forward:
            ROS_INFO("Forward");
            counterAction++;
            // if (deltaTime < 5) {
                // setWalkingParam(0.43, 0.02, 0.785);
                // setWalkingParam(0.065, 0.0, 0.0);
            // }
            // else if (counterAction < 20) {
            setWalkingParam(0.25, 0.0, 0.0);
            // }
            // else {
                if(fabs(RB_THETA) > BALL_TOLERANCE) {
                    followState = Following_Rotate;
                }
                else if(fabs(ball_y) >= (RL_MIN_LIMIT + 4)) {
                    followState = Following_Shift;
                }
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                {
                    followState = Following_Rotate;
                }
                else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                    followState = Following_Revo;
                else {
                    setWalkingParam(0.25, 0.0, 0.0);
                }
            // }
            break;

        case Following_Rotate:
            ROS_INFO("Rotate");

            if (ball_x < 70 && fabs(RB_THETA) >= BALL_TOLERANCE) {
                if(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90)) {
                    ROS_INFO("ROTATE CCW");
                    // setWalkingParam(-0.002, 0.2, -1.0);
                    setWalkingParam(0.01, -0.050, 0.6);
                }
                } else {
                    ROS_INFO("ROTATE CW");
                    // setWalkingParam(0.0, -0.1, 1);
                    setWalkingParam(-0.03, 0.050, -0.6);
                }

                // setWalkingParam(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 0.0: -0.002,
                //  ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -0.1:0.2,
                //   ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 1.00 : -1.0); // CCW : CW
                

            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE) {
                followState = Following_Forward;
            }
            else if(ball_y > RL_MIN_LIMIT) {
                followState = Following_Shift;
            }
            else if(fabs(RB_THETA) > BALL_TOLERANCE)
                followState = Following_Rotate;
            else if(ball_x < 60 && ball_x < 60)
                followState = Following_Forward;
            break;

        case Following_Shift:
            ROS_INFO("Shift");

            if(ball_y < -1)
                {   
                    ROS_INFO("GESER KANAN");
                    // setWalkingParam(-0.1, -0.23, -0.3);
                    setWalkingParam(-0.050, -0.200, -0.500);
                }
            else if(ball_y > 1)
                {   
                    ROS_INFO("GESER KIRI");
                    // setWalkingParam(-0.06, 0.2, 0.450);
                    setWalkingParam(0.0, 0.160, 0.7);
                }

            if(fabs(ball_y) <= RL_MIN_LIMIT && ball_x > FORWARD_MIN_REVO) {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (ball_x > 0 && fabs(ball_y) < RL_MIN_LIMIT)
            {
                if (ball_x > FORWARD_MIN_LIMIT)
                    followState = Following_Forward;
                else if (fabs(TOTAL_THETA) > ANGLE_TOLERANCE)
                    followState = Following_Revo;
            }
            break;

        case Following_Revo:
            ROS_INFO("REVO");
            if (TOTAL_THETA > 0){
                ROS_INFO("============ CCW ============");
                // setWalkingParam(0.15, -0.3, 0.5);
                setWalkingParam(0.0, 0.1, 0.0);
                
            } 
            else if(TOTAL_THETA < 0){
                ROS_INFO("============ CW ============");
                // setWalkingParam(0.1, 0.2, -0.5);
                setWalkingParam(0.0, -0.180, 0.300);
            }
            
            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else {
                if (fabs(TOTAL_THETA) < ANGLE_TOLERANCE) {
                    if (fabs(ball_y) > RL_MIN_LIMIT)
                        followState = Following_Shift;
                    else
                        followState = Following_Forward;
                }
            }

        // case Following_Stop:
        //     ROS_INFO("Jalan di Tempat");
           
        //     lastKick = true;
        //     kickTimeStart = ros::Time::now().toSec();
            
        //     setWalkingParam(0, 0, 0);
        //     break;

        default:
            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            
            else if (fabs(ball_y) >= (RL_MIN_LIMIT + 4) /*&& ball_x <= (FORWARD_MIN_LIMIT + 1)*/)
                followState = Following_Shift;
            // else if (ball_x <= FORWARD_MIN_LIMIT && ((ball_y<=13 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -13)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            // {
            //     followState = Following_Stop;
            // }
            else 
            {
                followState = Following_Forward;
            }
            break;
            
        }
        
        return false;
    }

    bool Behavioral::GoingToBall(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {   
        
        ROS_INFO("GO TO BALL");
        // ball_ = ball_y - 5;
        std_msgs::String data;
        data.data = "Not_Kicking";
        static bool rotate = false;
        // double walkDelay, walkStart;
        // static bool walkNow = false;
        kick_status_pub_.publish(data);

        static std::vector<int> ball_x_ma,ball_y_ma;


        int size_buff = 15;
        double ball_x_filt,ball_y_filt;
 
        ball_x_ma.insert(ball_x_ma.end(), ball_x);
        ball_y_ma.insert(ball_y_ma.end(), ball_y);

        double s_angle_yaw;
        if(resetOffset > 0){
            if(angle_yaw > 0)
            {
                s_angle_yaw = angle_yaw - resetOffset;
            }     
            else{
                s_angle_yaw = angle_yaw + resetOffset;
            }
        }
        else {
            if(angle_yaw > 0)
            {
                s_angle_yaw = angle_yaw + resetOffset;
            }     
            else{
                s_angle_yaw = angle_yaw - resetOffset;
            }
        }
        
        
        
        std::cout << "Angle Yaw       : " << angle_yaw << std::endl;
        std::cout << "S Angle Yaw       : " << s_angle_yaw << std::endl;
        std::cout << "RESSSEEEEEEEEETTTTTT Yaw       : " << resetOffset << std::endl;
     
        if(ball_x_ma.size() < size_buff){
            ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / ball_x_ma.size());
            ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / ball_y_ma.size());
        }else{
            ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / size_buff);
            ball_x_ma.erase(ball_x_ma.begin());
            ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / size_buff);
            ball_y_ma.erase(ball_y_ma.begin());
        }

        std_msgs::Float32 msg;

        msg.data = jarakX;
        ball_proj.publish(msg);

        // ball_x_filt = fabs(ball_x_filt);
        ball_x = ball_x_filt - 9.6; //-10 //arya ngurangin 10
        ball_y = ball_y_filt;
        
        double TOTAL_THETA = s_angle_yaw - ball_theta;
        double RB_THETA = atan2((ball_y), fabs(ball_x)) * 180 / PI; //Robot to Ball Theta

        double gain_theta = 0.865;

        // std::cout<<"ball x ["<<ball_x<<"]\n";
        // std::cout<<"ball y ["<<ball_y<<"]\n";
        // std::cout<<"total theta ["<<TOTAL_THETA<<"]\n";
        // std::cout<<"RB_THETA ["<<RB_THETA<<"]\n";
   
        if (ball_x <= FORWARD_MIN_LIMIT && /*((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) */ fabs(ball_y) == RL_MIN_LIMIT && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
        {
            ROS_INFO("APA KAU");
            // walkNow = true;
            // followState = Following_Stop;
            lastKick = true;
       
            ROS_ERROR("KKKKKKKKKKIIIIIIIIIIIIIIIIIIIIICCCCCCCCCCCCCCCCKKKKKKKKKKKKK");
         
                //setWalkingParam(0, 0, 0);
            return true;
        }
    
        ROS_INFO("BALL X [%f], BALL Y[%f], RB_THETA[%f], TOTAL THETA[%f]", ball_x, ball_y, RB_THETA, TOTAL_THETA);
        
        static int counterAction = 0; 

        // if(followState == lastFollowState && followState != Following_Rotate){
        //     lastFollowState = followState;
        //     walkDelay = true;

        switch (followState)
        {
        case Following_Forward:
        
            ROS_INFO("FORWARD");
            
            // std::cout<<"<<=======FORWARD========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(MAX_FB_STEP, 0, 0);
            counterAction++;
            ROS_INFO("Cnt Action: %d",counterAction );
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            } 
            if(ball_x > 40){ //40 //60
            // reset counterAction
            counterAction = 0;
                // calcWalkingParam(/*MAX_FB_STEP*/ 0.3, -0.1, 0.00);//-0.55
                // calcWalkingParam(0.3, 0.1 + (gain_theta*s_angle_yaw*0.4), -0.3 + (-gain_theta*s_angle_yaw));
                // calcWalkingParam(0.3, -0.05, 0.37); //mei 2024
                // setWalkingParam(/*MAX_FB_STEP*/ 0.35, 0, 0);

                
              
                std::cout << "romi babiiiiiiiiiiiiii" << std::endl;
                setWalkingParam(0.1, 0.04, -0.24);


                if(fabs(RB_THETA) > 1.5*BALL_TOLERANCE) {
                    followState = Following_Rotate;
                    break;
                }
         
            } else if(counterAction>20){ //baru nammbah 23 Mei jam 6.34 pagi
                // else {
                //     calcWalkingParam(0.25, -0.02, 0.1);
                //     std::cout << "romi nggak babiiiiii" << std::endl;
                // }
                // else
                // {
                //     if (fabs(RB_THETA) > BALL_TOLERANCE)
                //     {
                //         // followState = Following_OnTheSpot;
                //         // BALL_THETA = ball_theta;
                //         followState = Following_Rotate;
                //     }
                //     else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                //         followState = Following_Revo;
                //     else if (ball_x < FORWARD_MIN_REVO)
                //         followState = Following_ForwardMin;
                // }

                // else if (fabs(RB_THETA) > BALL_TOLERANCE)
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                {
                    // followState = Following_OnTheSpot;
                    // BALL_THETA = ball_theta;
                    followState = Following_Rotate;
                }
                else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                    followState = Following_Revo;
                else if (ball_x < FORWARD_MIN_REVO)
                    followState = Following_ForwardMin;
                else {
                    // calcWalkingParam(0.15, 0.18, 0.1);
                    // calcWalkingParam(MIN_FB_STEP,0.14, -0.06);
                    // calcWalkingParam(0.15, 0.18, 0.1
                    // calcWalkingParam(0.3, 0.1 + (gain_theta*s_angle_yaw*0.4), -0.3 + (-gain_theta*s_angle_yaw));
                    // calcWalkingParam(0.3, -0.05, 0.37); mei 2024
                    // setWalkingParam(0.3, 0.04, -0.24);
                    setWalkingParam(0.1, 0.04, -0.24);

                    std::cout << "romi nggak babiiiiii" << std::endl;
                  
                }
                // else if(/*ball_x <= 60*/counterAction > 5){
                    
                //     followState = Following_Stop;
                // }
                // else{
                //     ROS_ERROR("BALL NOT FOUND");
                // }
            
                
            }

            break;
        

        case Following_Rotate:
            
            // std::cout<<"<<========ROTATE========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, 0, (RB_THETA < 0) ? -MAX_RL_TURN : MAX_RL_TURN);
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=20 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -20)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            }  

            if(/*ball_x < 40 ||*/ (rotate)){
              
                ROS_INFO("ROTATE_ANJAY");

                if(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90)) {
                    ROS_INFO("ROTATE CCW");
                } else {
                    ROS_INFO("ROTATE CW");
                }
             
                // if(fabs(ball_y) < 12 && ball_x < 20)
                //     rotate = false;
                // else if(fabs(ball_y) < 50 && ball_x > 100)
                //     rotate = false;
                if(ball_x > 70 && fabs(RB_THETA) <= BALL_TOLERANCE) //30 //60
                    rotate = false;
                setWalkingParam(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 0.0: -0.002,
                 ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -0.1:0.2,
                  ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 1.00 : -1.0); // CCW : CW
                // setWalkingParam(((TOTAL_THETA > 0 && TOTAL_THETA < 90) || TOTAL_THETA <= -90) ? 0.1 : 0.12, //nilai RB_THETA nya gajelas kemarin
                //  ((TOTAL_THETA > 0 && TOTAL_THETA < 90) || TOTAL_THETA <= -90) ? -0.15 : 0.07,
                //   ((TOTAL_THETA > 0 && RB_THETA < 90) || TOTAL_THETA <= -90) ? MAX_RL_TURN : -MAX_RL_TURN);
            
            }
            // else if(ball_x < 20)
            //     followState = Following_ForwardMin;
            else if(ball_x > 160){ // TAQY NAMBAH Y
               
                ROS_INFO("ROTATE_FORWARD_JAUUHHHHH");
              
                if(fabs(ball_y) > 125)
                    rotate = true;
                setWalkingParam(/*MAX_FB_STEP*/ 0.3,
                 ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 0.15 : -0.15/*0.05*/,
                 ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -1 : 1);
            }
            else {
             
                
                if(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90)) {
                    ROS_INFO("ROTATE_FORWARD CCW");
                } else {
                    ROS_INFO("ROTATE_FORWARD CW");
                }
           
                if(fabs(ball_y) > 30)
                    followState = Following_Shift;
                else if(fabs(RB_THETA) > BALL_TOLERANCE)
                    rotate = true;
                else if(ball_x < 60 && ball_x < 60)
                    followState = Following_ForwardMin;
                else if (ball_x <= 14 && fabs(ball_y)>=4)
                    followState = Following_Shift;
                else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                    followState = Following_Revo;
                setWalkingParam(0.25,
                 ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -0.2/*0.05*/ : 0.2 ,  //nilai RB_THETA nya gajelas kemarin
                ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 1.0 : -1);
                // if(fabs(RB_THETA) > BALL_TOLERANCE) {
                //     rotate = true;
                // }
                // if(fabs(ball_y) >= 20) {
                //     followState = Following_Shift;
                // }
                // else if(ball_x > 25) {
                //     followState = Following_Forward;
                // }
                // else {
                //     followState = Following_Stop;
                // }
            }



            if (fabs(RB_THETA) < BALL_TOLERANCE)
                followState = false;
            // else if (ball_x <= 14 && fabs(ball_y)>=4)
            //     followState = Following_Shift;
            // else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
            //     followState = Following_Revo;
            break;

        case Following_Revo:
         
            ROS_INFO("REVO");
            
            // std::cout<<"<<=========REVO=========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, ((robot_theta - ball_theta) < 0) ? REVO_CW_STEP : REVO_CCW_STEP, ((robot_theta - ball_theta) < 0) ? REVO_CW_TURN : REVO_CCW_TURN)
            // repo_tt = true;
            // if (TOTAL_THETA > 0)
            //     setWalkingParam(0.0, -0.3, 0.8); //CCW
            // else if(TOTAL_THETA == 0)
            //     repo_tt = false; 
            // else 
            //      //CW
            //     setWalkingParam(0.05, 0.3, -0.6);            
            //     // followState = Following_Shift;

            if (TOTAL_THETA > 0){
                ROS_INFO("============ CCW ============");
                setWalkingParam(0.15, -0.3, 0.5);
            } 
            else if(TOTAL_THETA < 0){
                ROS_INFO("============ CW ============");
                setWalkingParam(0.1, 0.2, -0.5);
            }
                // repo_tt = false; 
            // else 
                 //CW
                // followState = Following_Shift;

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_ForwardMin;
            }
            else
            {
                if (fabs(TOTAL_THETA) < ANGLE_TOLERANCE)
                {
                    // if (ball_x < 5 && fabs(ball_y)< 11)
                    //     followState = Following_Backward;
                    if (fabs(ball_y) > RL_MIN_LIMIT)
                        followState = Following_Shift;
                    else
                        followState = Following_ForwardMin;
                }
                // else if (ball_x < 1 && fabs(ball_y)< 6)
                //     followState = Following_Backward;
            }
            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            }
                
            break;

        case Following_Shift:
          
            ROS_INFO("SHIFT");
          
            // std::cout<<"<<========SHIFT========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);
            // setWalkingParam(-0.05, (ball_y > 1) ? MAX_RL_STEP : -MAX_RL_STEP, (ball_y > 1) ? 0.4 : -0.1);

            if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=10 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -10)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
            {
                // walkNow = true;
                followState = Following_Stop;
                break;
            }
            if(ball_y < -1) //-5
            {   
              
                ROS_ERROR("GESER KANAN");
            
                // setWalkingParam(0.1 , -0.1, -0.65);
                setWalkingParam(-0.1, -0.23, -0.3);
                if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
                {
                    // walkNow = true;
                    followState = Following_Stop;
                    break;
                }
            }
            
            else if(ball_y > 1) //5
            {   
              
                ROS_ERROR("GESER KIRI");
              
                // setWalkingParam(0.1, /*MAX_RL_STEP*/ 0.1, 0.4);
                setWalkingParam(-0.06, 0.2, 0.450);
                if (ball_x < FORWARD_MIN_LIMIT && ((ball_y<=5 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -5)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
                {
                    // walkNow = true;
                    followState = Following_Stop;
                    break;
                }

            }

            if(fabs(ball_y) <= RL_MIN_LIMIT && ball_x > FORWARD_MIN_REVO) {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_ForwardMin;
            }
            // if (ball_x > FORWARD_MIN_REVO)
            // {
            //     if (fabs(RB_THETA) > BALL_TOLERANCE)
            //         followState = Following_Rotate;
            //     else
            //         followState = Following_ForwardMin;
            // }
            else if (ball_x > 0 && fabs(ball_y) < RL_MIN_LIMIT)
            {
                if (ball_x > FORWARD_MIN_LIMIT)
                    followState = Following_ForwardMin;
                else if (fabs(TOTAL_THETA) > ANGLE_TOLERANCE)
                    followState = Following_Revo;
            }
            // else if (ball_x < 0)
            //     followState = Following_Backward;
            break;

        case Following_ForwardMin:
         
            ROS_INFO("FORWARD MIN");
         
            // std::cout<<"<<=====FORWARD MIN=====>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(MIN_FB_STEP,0, 0);
            // calcWalkingParam(0.15, 0.18, 0.1
            // calcWalkingParam(0.2, 0.1 + (gain_theta*s_angle_yaw*0.4), (-gain_theta*s_angle_yaw));
            // calcWalkingParam(0.2, 0.01, -0.5); //mwi 2024
            setWalkingParam(0.15, 0.04, -0.24);
            //coba ganti nambahin arya
            if (ball_x <= FORWARD_MIN_LIMIT && ((ball_y<=13 && ball_y>=1 ) || (ball_y <=-1 && ball_y >= -13)) && fabs(TOTAL_THETA) < KICKING_TOLERANCE)//dibesarin
            {
                // walkNow = true;
                
                followState = Following_Stop;
                break;
            }

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (fabs(ball_y) >= (RL_MIN_LIMIT + 4) && ball_x <= (FORWARD_MIN_LIMIT + 1))
                followState = Following_Shift;
            break;

        case Following_Backward:
           
            ROS_INFO("BACKWARD");
        
            // std::cout<<"<<=======BACKWARD======>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(MAX_BW_STEP, 0, 0);
            setWalkingParam(MAX_BW_STEP,0,0.3);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_ForwardMin;
            }
            else if (fabs(ball_x) <= FORWARD_MIN_REVO && fabs(ball_y) >= 5)
                followState = Following_Shift;
            else if (fabs(ball_x <= FORWARD_MIN_REVO))
                followState = Following_Revo;
            break;

        case Following_Stop:
           
            ROS_INFO("READY TO KICK");
           
            lastKick = true;
            kickTimeStart = ros::Time::now().toSec();
           
            // ROS_ERROR("KKKKKKKKKKIIIIIIIIIIIIIIIIIIIIICCCCCCCCCCCCCCCCKKKKKKKKKKKKK");
           
            //setWalkingParam(0, 0, 0);
            return true;   
        default:
           
            ROS_INFO("GAMASUK STATE WOYYYY");
           
            followState = Following_ForwardMin;
            break;  
        }

        return false;
    }

    bool Behavioral::AlignToOri(double robot_theta, double ball_x, double ball_y, double ball_theta)
    {
        ROS_INFO("GO TO BALL");
        ball_x = ball_x - 10;
        static double BALL_THETA, walkStart;
        double TOTAL_THETA = angle_yaw - 45;
        double RB_THETA = atan2(ball_y, ball_x) * 180 / PI; //Robot to Ball Theta
        double walkNow;
        static bool walkDelay;

        ROS_INFO("BALL X [%f], BALL Y[%f], RB_THETA[%f], TOTAL THETA[%f]", ball_x, ball_y, RB_THETA, TOTAL_THETA);
        // std::cout<<"Ball X\t"<<ball_x<<"\tBall Y\t"<<ball_y<<std::endl;
        // std::cout<<"RB_THETA\t"<<RB_THETA<<"\tROBOT THETA\t"<<robot_theta/*<<"\tBALL THETA\t"<<ball_theta*/<<std::endl;

        if (followState == Following_Forward || followState == Following_Rotate)
            BALL_THETA = ball_theta;

        if (ball_x < 8 && fabs(angle_yaw - 45) < 10)
            return true;
        // if(ball_x < FORWARD_MIN_LIMIT && fabs(ball_y) < RL_MIN_LIMIT && fabs(RB_THETA) < BALL_TOLERANCE && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
        //     return true;

        if (followState != Following_OnTheSpot)
            walkDelay = true;

        // if(followState == lastFollowState && followState != Following_Rotate){
        //     lastFollowState = followState;
        //     walkDelay = true;
        switch (followState)
        {
        case Following_Forward:
            ROS_INFO("FORWARD");
            // std::cout<<"<<=======FORWARD========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(MAX_FB_STEP, 0, 0);
            setWalkingParam(MAX_FB_STEP, 0, -0.1);

            if (fabs(RB_THETA) > BALL_TOLERANCE)
            {
                // followState = Following_OnTheSpot;
                // BALL_THETA = ball_theta;
                followState = Following_Rotate;
            }
            else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                followState = Following_Revo;
            else if (ball_x < FORWARD_MIN_REVO)
                followState = Following_ForwardMin;
            break;

        case Following_Rotate:
            ROS_INFO("ROTATE");
            // std::cout<<"<<========ROTATE========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, 0, (RB_THETA < 0) ? -MAX_RL_TURN : MAX_RL_TURN);
            setWalkingParam(0, 0, ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? MAX_RL_TURN : -MAX_RL_TURN);

            if (fabs(RB_THETA) < ANGLE_TOLERANCE)
                followState = Following_Forward;
            else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                followState = Following_Revo;
            break;

        case Following_Revo:
            ROS_INFO("REVO");
            // std::cout<<"<<=========REVO=========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, ((robot_theta - ball_theta) < 0) ? REVO_CW_STEP : REVO_CCW_STEP, ((robot_theta - ball_theta) < 0) ? REVO_CW_TURN : REVO_CCW_TURN)

            if (TOTAL_THETA >= 0)
                setWalkingParam(-0.05, REVO_CCW_STEP, REVO_CCW_TURN);
            else
                setWalkingParam(-0.05, REVO_CW_STEP, REVO_CW_TURN);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else
            {
                if (fabs(TOTAL_THETA) < ANGLE_TOLERANCE)
                {
                    if (ball_x < 0)
                        followState = Following_Rotate;
                    else if (fabs(ball_y) > RL_MIN_LIMIT)
                        followState = Following_Shift;
                }
                else if (fabs(RB_THETA) > BACKWARD_TOLERANCE)
                    followState = Following_Backward;
            }
            break;

        case Following_Shift:
            ROS_INFO("SHIFT");
            // std::cout<<"<<========SHIFT========>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            // calcWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);
            setWalkingParam(0, (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP, 0);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (ball_x > 0 && fabs(ball_y) < RL_MIN_LIMIT)
            {
                if (ball_x > FORWARD_MIN_LIMIT)
                    followState = Following_ForwardMin;
                else if (fabs(TOTAL_THETA) > ANGLE_TOLERANCE)
                    followState = Following_Revo;
            }
            else if (fabs(RB_THETA) > BACKWARD_TOLERANCE || ball_x < 0)
                followState = Following_Backward;
            break;

        case Following_ForwardMin:
            ROS_INFO("FORWARD MIN");
            // std::cout<<"<<=====FORWARD MIN=====>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            calcWalkingParam(MIN_FB_STEP, 0, 0);
            // setWalkingParam(MIN_FB_STEP,0,0);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (fabs(RB_THETA) > BACKWARD_TOLERANCE)
                followState = Following_Shift;
            break;

        case Following_Backward:
            ROS_INFO("BACKWARAD");
            // std::cout<<"<<=======BACKWARD======>>"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            calcWalkingParam(MAX_BW_STEP, 0, 0);
            // setWalkingParam(MAX_BW_STEP,0,0);

            if (ball_x > FORWARD_MIN_REVO)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                    followState = Following_Rotate;
                else
                    followState = Following_Forward;
            }
            else if (fabs(RB_THETA) < BACKWARD_TOLERANCE)
                followState = Following_Shift;
            else if (fabs(TOTAL_THETA) > ANGLE_TOLERANCE)
                followState = Following_Revo;
            break;

        case Following_OnTheSpot:
            // std::cout<<"-------ON THE SPOT------"<<"\tBall X : "<<ball_x<<"\tBall Y : "<<ball_y<<"\tRB T : "<<RB_THETA<<"\tBALL : "<<ball_theta<<std::endl;
            // std::cout<<"\t\t\t\tROBOT : "<<robot_theta<<"\tYAW : "<<angle_yaw<<"\t\t\tTHETA : "<<TOTAL_THETA<<std::endl;
            if (walkDelay)
            {
                walkStart = ros::Time::now().toSec();
                walkDelay = false;
            }
            setWalkingParam(-0.08, 0.05, -0.02);//0,0,0
            walkNow = ros::Time::now().toSec() - walkStart;

            if (walkNow > 2)
                followState = Following_Rotate;
            break;

        case Following_Trantition:
            calcWalkingParam(MIN_FB_STEP, 0, 0);
            // setWalkingParam(MIN_FB_STEP, 0, 0);

            if (FB_Step <= TRANTITION_BORDER)
            {
                if (fabs(RB_THETA) > BALL_TOLERANCE)
                {
                    // followState = Following_OnTheSpot;
                    // BALL_THETA = ball_theta;
                    followState = Following_Rotate;
                }
                else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
                    followState = Following_Revo;
            }
            break;
        }

        return false;
    }

    //================== MULAI STRATEGI 1 =======================
    void Behavioral::stupidSearchForRunner()
    {

        trackingMode = PillarTracker::Square;
        // std::cout << "<<=======SEARCHHH__PILLARRRR========>>"
        //         << "\tPillar X : " << robot_x << "\tPillar Y : " << robot_y << std::endl;
        setWalkingParam(0, 0, 1);
    }

    //================== STRATEGI 1 =======================
    bool Behavioral::goToPillar(double robot_theta, double pillar_x, double pillar_y, double pillar_theta)
    {
        static int last_pillar;
        // std::cout << "GO TO PILLAR" << std::endl;
        std::cout << "X = " << pillar_x << "  =====   Y =  " << pillar_y << "  =====   Z = " << angle_yaw << std::endl;
        if (pillar_x <= X_MIN_LIMIT && pillar_y < Y_RIGHT_LIMIT_FOURTH && pillar_y > Y_LEFT_LIMIT_FOURTH)
        {
            // stopFollowing();
            return true;
        }

        // FSM NIAS
        // if (fabs(angle_roll) < 10 && angle_pitch < 9 && angle_pitch > 6)
        //     last_pillar = pillar_y;

        // pillar_y = last_pillar;

        // ROS_ERROR("PPPPPPPPPUUUUUUUUNNNNNNNNYAAAAAAAAAAAKKKKKKKKKKUUUUUUU");
        // std::cout << "GO TO PILLAR" << std::endl;
        // std::cout << "X = " << pillar_x << "  =====   Y =  " << pillar_y << std::endl;

        // switch (followState)
        // {
        // case Following_Forward:
        //     setWalkingParam(MAX_FB_STEP, 0, 0);

        //     if (fabs(angle_yaw) > ANGLE_YAW_LIMIT)
        //         followState = Following_Rotate;
        //     else if (pillar_y > Y_RIGHT_LIMIT || pillar_y < Y_LEFT_LIMIT)
        //         followState = Following_Shift;

        //     break;

        // case Following_Shift:

        //     if (pillar_y > Y_RIGHT_LIMIT)
        //         setWalkingParam(0, MAX_RL_STEP, 0);
        //     else if (pillar_y < Y_LEFT_LIMIT)
        //         setWalkingParam(0, -MAX_RL_STEP, 0);
        //     else if (fabs(angle_yaw) > ANGLE_YAW_LIMIT)
        //         followState = Following_Rotate;
        //     else
        //         followState = Following_Forward;

        //     break;

        // case Following_Rotate:
        //     setWalkingParam(0, 0, angle_yaw < 0 ? -1.5 : 1.5);

        //     if (fabs(angle_yaw) > ANGLE_YAW_LIMIT)
        //         followState = Following_Forward;

        //     break;
        // }
        // FSM NIAS SELESAI

        // FSM ORIGINAL
        // if (fabs(angle_yaw) > ANGLE_YAW_LIMIT)
        // {
        //     ROS_WARN("KONNTOOOOLLLL");

        //     setWalkingParam(0, 0, angle_yaw < 0 ? -1.5 : 1.5);
        // }

        // else
        // {

        //     ROS_WARN("ZZZZZZZZZZZEUUUUUUUUUUUUUUSSSSSSSSSS");

        //     switch (runnerPillarState)
        //     {
        //     case Runner_Pillar_Forward:
        //         ROS_WARN("Following FORWARDDD SIFHAFAIK");
        //         if (checkForTimePillar)
        //         {
        //             timeStartForward = ros::Time::now().toSec();
        //             checkForTimePillar = false;
        //         }

        //         double timeBeforeCheck;
        //         timeBeforeCheck = ros::Time::now().toSec() - timeStartForward;

        //         if (timeBeforeCheck < 2)
        //             setWalkingParam(MAX_FB_STEP, 0, 0);
        //         else
        //         {
        //             ROS_WARN("Following Forward");
        //             std::cout << "Following Forward" << std::endl;
        //             setWalkingParam(MAX_FB_STEP, 0, 0);

        //             if (pillar_y > Y_RIGHT_LIMIT || pillar_y < Y_LEFT_LIMIT)
        //                 runnerPillarState = Runner_Pillar_Shift;
        //         }
        //         break;

        //     case Runner_Pillar_Shift:
        //         ROS_WARN("Following Shift");
        //         std::cout << "Following Shift" << std::endl;

        //         if (pillar_y > Y_RIGHT_LIMIT)
        //         {
        //             setWalkingParam(0, MAX_RL_STEP, 0);
        //         }
        //         else if (pillar_y < Y_LEFT_LIMIT)
        //         {
        //             setWalkingParam(0, -MAX_RL_STEP, 0);
        //         }
        //         if (pillar_y < Y_RIGHT_LIMIT || pillar_y > Y_LEFT_LIMIT)
        //         {
        //             runnerPillarState = Runner_Pillar_Forward;
        //             checkForTimePillar = true;
        //         }
        //         break;
        //     }
        // }
        // FSM ORIGINAL SELESAI

        //PERCOBAAN SOLUSI PILLAR_Y YANG TIDAK STABIL
        // double pillarYChecker;
        // if (checkForPillarY)
        // {
        //     previousPillarY = pillar_y;
        //     currentPillarY = pillar_y;
        //     checkForPillarY = false;
        // }
        // pillarYChecker = previousPillarY - pillar_y;
        // if (pillar_y < 0 && pillarYChecker < 7){
        //     previousPillarY = currentPillarY;
        //     currentPillarY = pillar_y;
        // }
        //kalau menggunakan ini, jangan lupa ganti pillar_y yang jadi penanda pindah kondisi menjadi currentPillarY
        //PERCOBAAN SOLUSI PILLAR_Y YANG TIDAK STABIL SELESAI

        //FSM MODEL BARU, PRIORITAS PADA ROTATE, Y LIMIT VARIATIF TERHADAP X
        switch (runnerPillarState)
        {
        case Runner_Pillar_Forward:
            ROS_WARN("Runner Pillar Forward");

            if (checkForTimePillar)
            {
                timeStartForward = ros::Time::now().toSec();
                checkForTimePillar = false;
            }

            double timeBeforeCheck;
            timeBeforeCheck = ros::Time::now().toSec() - timeStartForward;

            if (timeBeforeCheck < 2)
                setWalkingParam(MAX_FB_STEP, 0, 0);
            else
            {
                setWalkingParam(MAX_FB_STEP, 0, 0);
                if (fabs(angle_yaw) > 5)
                    runnerPillarState = Runner_Pillar_Rotate;
                else if (pillar_x < 100 && pillar_y > Y_RIGHT_LIMIT_FIRST || pillar_y < Y_LEFT_LIMIT_FIRST)
                    runnerPillarState = Runner_Pillar_Shift;
                else if (pillar_x < 50 && pillar_y > Y_RIGHT_LIMIT_SECOND || pillar_y < Y_LEFT_LIMIT_SECOND)
                    runnerPillarState = Runner_Pillar_Shift;
                else if (pillar_x < 30 && pillar_y > Y_RIGHT_LIMIT_THIRD || pillar_y < Y_LEFT_LIMIT_THIRD)
                    runnerPillarState = Runner_Pillar_Shift;
                else if (pillar_x < 27 && pillar_y > Y_RIGHT_LIMIT_FOURTH || pillar_y < Y_LEFT_LIMIT_FOURTH)
                    runnerPillarState = Runner_Pillar_Shift;
            }
            break;

        case Runner_Pillar_Shift:
            ROS_WARN("Runner Pillar Shift");
            setWalkingParam(0, pillar_y < -27 ? -MAX_RL_STEP : MAX_RL_STEP, 0);

            if (fabs(angle_yaw) > 10)
                runnerPillarState = Runner_Pillar_Rotate;
            else if (pillar_y < Y_RIGHT_LIMIT_FOURTH && pillar_y > Y_LEFT_LIMIT_FOURTH)
            {
                runnerPillarState = Runner_Pillar_Forward;
                checkForTimePillar = true;
            }
            break;

        case Runner_Pillar_Rotate:
            ROS_WARN("Runner Pillar Rotate");
            setWalkingParam(0, 0, angle_yaw < 0 ? -MAX_RL_TURN : MAX_RL_TURN);

            if (fabs(angle_yaw) < 5)
            {
                runnerPillarState = Runner_Pillar_Forward;
            }
            // else if (pillar_x < 100 && pillar_y > Y_RIGHT_LIMIT_FIRST || pillar_y < Y_LEFT_LIMIT_FIRST)
            //     runnerPillarState = Runner_Pillar_Shift;
            // else if (pillar_x < 50 && pillar_y > Y_RIGHT_LIMIT_SECOND || pillar_y < Y_LEFT_LIMIT_SECOND)
            //     runnerPillarState = Runner_Pillar_Shift;
            // else if (pillar_x < 30 && pillar_y > Y_RIGHT_LIMIT_THIRD || pillar_y < Y_LEFT_LIMIT_THIRD)
            //     runnerPillarState = Runner_Pillar_Shift;
            // else if (pillar_x < 27 && pillar_y > Y_RIGHT_LIMIT_FOURTH || pillar_y < Y_LEFT_LIMIT_FOURTH)
            //     runnerPillarState = Runner_Pillar_Shift;
            break;
        }
        //FSM MODEL BARU, PRIORITAS PADA ROTATE, Y LIMIT VARIATIF TERHADAP X SELESAI
        return false;
    }

    //================== STRATEGI 1 =======================
    bool Behavioral::RevoPillar_(double pillar_x)
    {
        // std::cout <<"PITCH "<< "====================+> " << angle_pitch << std::endl;
        // std::cout <<"ROLL  "<< "====================+> " << angle_roll << std::endl;

        double searchNow;
        static bool firstSearch_revoV2;
        if (!firstSearch_revoV2)
        {
            firstSearch_revoV2 = true;
            // std::cout << "=============     TURNING READY    ================="
            // << "       " << firstSearch_revoV2 << std::endl;
            searchStart = ros::Time::now().toSec();
            m_revo_state_ = first_;
            m_revo_status_ = firstRevo_;
            // std::cout << "=====BEHAVIORAL GASSSSS MUTER PILLAR========"
            // << searchStart << "....." << firstSearch_revoV2 << std::endl;
        }
        // std::cout << "======TURNING=======           " << std::endl;
        switch (m_revo_status_)
        {
        case firstRevo_:
            switch (m_revo_state_)
            {
            case first_:
                searchNow = ros::Time::now().toSec() - searchStart;
                // std::cout << "======FIRST_REVO=========     " << searchNow << std::endl;
                setWalkingParam(MAX_FB_STEP, 0, 0);
                // std::cout << "======FORWARD=========     " << searchNow << std::endl;
                if (searchNow > 6)
                {
                    m_revo_state_ = second_;
                    m_revo_status_ = firstRevo_;
                    searchStart = ros::Time::now().toSec();
                    // std::cout << "Go To Second" << std::endl;
                }
                break;

            case second_:
                searchNow = ros::Time::now().toSec() - searchStart;
                // std::cout << "========SECOND_REVO==          " << std::endl;
                setWalkingParam(0, 0, REVO_CW_TURN);
                // std::cout << "======ROTATE=========     " << angle_yaw << std::endl;
                if (angle_yaw > 90)
                {
                    m_revo_state_ = third_;
                    m_revo_status_ = firstRevo_;
                    searchStart = ros::Time::now().toSec();
                    // std::cout << "Go To Third" << std::endl;
                }
                break;

            case third_:
                searchNow = ros::Time::now().toSec() - searchStart;
                // std::cout << "======THIRD_REVO=========     " << searchNow << std::endl;
                setWalkingParam(MAX_FB_STEP, 0, 0);
                // std::cout << "======FORWARD=========     " << searchNow << std::endl;
                if (searchNow > 6)
                {
                    m_revo_state_ = fourth_;
                    m_revo_status_ = firstRevo_;
                    searchStart = ros::Time::now().toSec();
                    // std::cout << "Go To Fourth" << std::endl;
                }
                break;

            case fourth_:
                searchNow = ros::Time::now().toSec() - searchStart;
                // std::cout << "========FOURTH_REVO==          " << std::endl;
                setWalkingParam(0, 0, REVO_CW_TURN);
                // std::cout << "======ROTATE=========     " << angle_yaw << std::endl;
                if (angle_yaw > 170)
                {
                    // std::cout << "=====KELAR======" << std::endl;
                    return true;
                }
                break;
            }
            break;
        }
        return false;
    }
    //================== STRATEGI 1 =======================
    void Behavioral::backToHome()
    {
        // std::cout <<"PITCH "<< "=================================================+> " << angle_pitch << std::endl;
        // std::cout <<"ROLL  "<< "=================================================+> " << angle_roll << std::endl;

        // std::cout << "BACK TO HOME" << std::endl;
        searchStart = ros::Time::now().toSec();
        double timeBeforeCheckFollowing;
        double timeBeforeCheck;
        double fabs_angle_yaw;
        if (angle_yaw < 0)
        {
            fabs_angle_yaw = angle_yaw + 360;
        }

        if (checkForTime)
        {
            timeStartForward = ros::Time::now().toSec();
            checkForTime = false;
        }

        if (checkForStop)
        {
            timeForStop = ros::Time::now().toSec();
            checkForStop = false;
        }
        // timeBeforeCheckFollowing = ros::Time::now().toSec() - timeForStop;
        // if (timeBeforeCheckFollowing < 1)
        // {
        //     stopFollowing();
        //     std::cout << "stop following" << std::endl;
        // }
        // if (turnOnStartFollowing)
        // {
        //     startFollowing();
        //     std::cout << "start following" << std::endl;
        //     turnOnStartFollowing = false;
        // }
        // std::cout << "we are good to go" << std::endl;
        switch (runnerBackState)
        {
        case Runner_Back_Forward:

            // std::cout << "Following Forward" << std::endl;
            setWalkingParam(MAX_FB_STEP, 0, 0);
            // std::cout << fabs_angle_yaw << "   ==   " << angle_yaw << std::endl;
            if (checkForTime)
            {
                timeStartForward = ros::Time::now().toSec();
                checkForTime = false;
            }
            timeBeforeCheck = ros::Time::now().toSec() - timeStartForward;
            if (timeBeforeCheck < 4)
            {
                setWalkingParam(MAX_FB_STEP, 0, 0);
            }
            else
            {

                // std::cout << "Following Forward" << std::endl;
                setWalkingParam(MAX_FB_STEP, 0, 0);
                if (angle_yaw < 0)
                {
                    if (fabs_angle_yaw > 185)
                    {
                        runnerBackState = Runner_Back_Rotate;
                    }
                }
                else if (angle_yaw < 175)
                {
                    runnerBackState = Runner_Back_Rotate;
                }
            }
            break;

        case Runner_Back_Rotate:
            // std::cout << "Following Rotate" << std::endl;
            // std::cout << fabs_angle_yaw << "   ==   " << angle_yaw << std::endl;
            if (angle_yaw < 0)
            {
                setWalkingParam(0, 0, MAX_RL_TURN);
            }
            else
            {
                setWalkingParam(0, 0, -MAX_RL_TURN);
            }

            if (angle_yaw < 0)
            {
                if (fabs_angle_yaw < 182)
                {
                    runnerBackState = Runner_Back_Forward;
                    checkForTime = true;
                }
            }
            else if (angle_yaw > 177)
            {
                runnerBackState = Runner_Back_Forward;
                checkForTime = true;
            }

            break;
        }
    }
    //================== STRATEGI 1 =======================
    void Behavioral::awayFromHome()
    {
        // std::cout << "AWAY FROM HOME" << std::endl;

        // switch (m_pass_state)
        // {
        // case Pass_Forward:
        //     ROS_INFO("FORWARD");
        //     setWalkingParam(MAX_FB_STEP, 0, 0);
        //     if (angle_yaw > 10)
        //         m_pass_state = Pass_Rotate;
        //     break;

        // case Pass_Rotate:
        //     ROS_INFO("Rotate");
        //     setWalkingParam(0, 0, (angle_yaw > 0) ? 1 : -1);
        //     if (angle_yaw < 10)
        //         m_pass_state = Pass_Forward;
        //     break;
        // }
        std::cout << "Z = " << angle_yaw << std::endl;
        switch (runnerAwayState)
        {
        case Runner_Away_Forward:

            if (checkForTime)
            {
                timeStartForward = ros::Time::now().toSec();
                checkForTime = false;
            }

            double timeBeforeCheck;
            timeBeforeCheck = ros::Time::now().toSec() - timeStartForward;
            if (timeBeforeCheck < 4)
            {
                setWalkingParam(MAX_FB_STEP, 0, 0);
            }
            else
            {

                // std::cout << "Following Forward" << std::endl;
                setWalkingParam(MAX_FB_STEP, 0, 0);

                if (fabs(angle_yaw) > 8)
                {

                    runnerAwayState = Runner_Away_Rotate;
                }
            }

            break;

        case Runner_Away_Rotate:
            // std::cout << "Following Rotate" << std::endl;
            setWalkingParam(0, 0, angle_yaw < 0 ? -MAX_RL_TURN : MAX_RL_TURN);

            if (fabs(angle_yaw) < 3)
            {
                runnerAwayState = Runner_Away_Forward;
                checkForTime = true;
            }
            break;
        }
    }

    void Behavioral::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        Eigen::Quaterniond imu_orientation;

        imu_orientation.x() = msg->orientation.x;
        imu_orientation.y() = msg->orientation.y;
        imu_orientation.z() = msg->orientation.z;
        imu_orientation.w() = msg->orientation.w;

        // orientation_check = imu_orientation.x();

        // std::cout<<"orientasi y : "<<orientation_check *RADIAN2DEGREE<<std::endl;
    }
  
    //================== SELESAI STRATEGI 1 =======================

} // namespace robotis_op
