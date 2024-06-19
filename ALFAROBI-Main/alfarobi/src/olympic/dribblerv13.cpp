#include "alfarobi/dribblerv13.h"

namespace robotis_op
{
OlympicDribblerv13::OlympicDribblerv13()
    : SPIN_RATE(30),
      wait_count_(0),
      nh_(ros::this_node::getName()),
      s_camera_track(0),
      firstIn(true)
{
    enable_ = false;
    kick = false;
    nh_.param<bool>("calibration", s_calibration, false);
    nh_.param<bool>("role", s_role, false);
    // if (s_role)
    //     m_robot_state = Robot_Waiting;
    // else
    //     m_robot_state = Robot_Positioning;
    m_robot_state = Robot_Waiting;
    // m_robot_state = Robot_Start;
    boost::thread queue_thread_ = boost::thread(boost::bind(&OlympicDribblerv13::queueThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&OlympicDribblerv13::processThread, this));
}

OlympicDribblerv13::~OlympicDribblerv13()
{
}

void OlympicDribblerv13::queueThread()
{
    demo_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &OlympicDribblerv13::CommandCallback, this);
    // pengTrue_pub_ = nh_.advertise<std_msgs::Bool>("/topic_sendiri", 1);
    // pengTrue_sub_ = nh_.subscribe("/topic_sendiri", 1, &OlympicDribblerv13::pengTrueCallback, this);

    ros::Rate rate(30);

    while (nh_.ok())
    {
        ros::spinOnce();

        rate.sleep();
    }
}

// void OlympicDribblerv13::pengTrueCallback(const std_msgs::Bool::ConstPtr &msg)
// {
//     behavioral_.friend_status = msg->data;
// }

void OlympicDribblerv13::CommandCallback(const std_msgs::String::ConstPtr &msg) //TOPICNYA DARI MANA BELUM TAU
{
    if (enable_ == false)
        return;
}

void OlympicDribblerv13::setModuleEnable()
{
    count = 0;
    kick = false;
    enable_ = true;
    startPassMode();
}

void OlympicDribblerv13::setModuleDisable()
{
    tracker_.stopTracking();
    behavioral_.stopFollowing();

    enable_ = false;

    behavioral_.setModuleTo("base_module");
}

void OlympicDribblerv13::startPassMode()
{
    ROS_INFO("Start Dribbler Demo");

    enable_ = true;

    m_dribbler_state = Dribbler_Start;
}

void OlympicDribblerv13::stopPassMode()
{
    ROS_INFO("Stop Dribbler Demo");

    enable_ = false;

    m_dribbler_state = Dribbler_Stop;
}

void OlympicDribblerv13::processThread()
{
    ros::Rate loop_rate(SPIN_RATE);
    kick = false;
    tracker_.startTracking();

    while (ros::ok())
    {
        if (wait_count_ <= 0) // waktu untuk start
        {
            if (enable_)
            {
                behavioral_.CALIBRATION = s_calibration;
                switch (s_camera_track)
                {
                case Ball:
                    m_tracking_status = tracker_.processTracking(s_tracking_mode);
                    break;

                // case Goal:
                //     m_tracking_status = goalTracker_.processTracking(s_tracking_mode);
                //     break;
                }
                if (m_tracking_status == BallTracker::DontTrack )
                // || m_tracking_status == GoalpostTracker::DontTrack
                {
                    if (s_camera_track == Ball)
                        tracker_.startTracking();
                    // else
                    //     goalTracker_.startTracking();
                }
                else
                {
                    check_fallen = behavioral_.checkFallen();
                    m_button_status = behavioral_.buttonStatus;
                    if (check_fallen){
                        // behavioral_.stopFollowing();
                        // m_robot_state = Robot_Waiting ;
                        // check_fallen= false;
                        ROS_ERROR("ROBOT JATUH CUYYYY");
                    }

                    if (m_stand_state == Behavioral::Nothing)
                    {
                        buttonStatus();
                        dribblerState();
                    }
                    standState();
                }
            }
        }
        else wait_count_ -= 1;

        loop_rate.sleep();

    } // end while
}

void OlympicDribblerv13::buttonStatus()
{
    switch (m_button_status)
    {
    // case Alfarobi::L1: //start
    //     // // behavioral_.send();
    //     // // behavioral_.friend_status = true;
    //     // // behavioral_.startFollowing();
    //     // kick = true;
    //     // behavioral_.firstKick = false;
    //     // // behavioral_.stopFollowing();
    //     break;
    // case Alfarobi::L2:
    //     // s_calibration = false;
    //     // behavioral_.startFollowing();
    //     // m_dribbler_state = Dribbler_Run;
    //     break;
    // case Alfarobi::L3:
    //     // s_calibration = true;
    //     // // if(!s_role && count > 1)
    //     // behavioral_.stopFollowing();
    //     // count = 5;
    //     break;
    // case Alfarobi::L4: //reset
    //     if (s_role)
    //         m_robot_state = Robot_Waiting;
    //     else
    //         m_robot_state = Robot_Positioning;

    //     behavioral_.lastKick = false;
    //     count = 0;
    //     firstIn = false;
    //     behavioral_.resetComm();
    //     behavioral_.resetFSM();
    //     // behavioral_.m_pass_state = Pass_Forward;
    //     // behavioral_.startFollowing();
    //     behavioral_.friend_status = false;
        
    //     break;
    // case Alfarobi::R1:
    //     // ros::Duration(2).sleep();
    //     //pengTruePublish();
    //     // kick = true;
    //     // behavioral_.firstKick = false;
    //     // // behavioral_.stopFollowing();
    //     behavioral_.send();
    //     // behavioral_.alfaStartPass();
    //     behavioral_.friend_status=true;
    //     behavioral_.startFollowing();
    //     std::cout << "BAGUS BANGKEEEEEEEEEEEEEEEE" << std::endl;
    //     break;
    // case Alfarobi::R2:
    //     behavioral_.resetOffset = behavioral_.m_yaw;
    //     break;
    // case Alfarobi::R3: //stop
    //     behavioral_.stopFollowing();
    //     // behavioral_.friend_status = false;
    //     // behavioral_.bo_ol = true;
    //     break;
    // case Alfarobi::R4:
    //    // m_robot_state = Robot_Kicking;
    //     // behavioral_.startFollowing();
    //     // kick = false;
    //     //JANGAN DIKASI APA2, BUAT IMU GACHA -KEITOT
    //     break;
    case Alfarobi::B1:
        ROS_INFO("BUTTON_1");
        behavioral_.resetOffset = behavioral_.m_yaw;
        break;
    case Alfarobi::B2: //stop
        ROS_INFO("BUTTON_2");
        behavioral_.stopFollowing();
        behavioral_.friend_status = false;
        break;
    case Alfarobi::B3:
        ROS_INFO("BUTTON_3");
        ros::Duration(2).sleep();
        //pengTruePublish();
        // kick = true;
        // behavioral_.firstKick = false;
        // // behavioral_.stopFollowing();
        // behavioral_.send();
        // behavioral_.alfaStartPass();
        behavioral_.friend_status=true;
        behavioral_.firstIn = false;
        behavioral_.startFollowing();
        std::cout << "BAGUS BANGKEEEEEEEEEEEEEEEE" << std::endl;
        break;
    case Alfarobi::B4:
        ROS_INFO("BUTTON_4");
        break;
    default:
        ROS_INFO("BUTTON_NONE");
        break;
    }
}

void OlympicDribblerv13::dribblerState()
{
    switch (m_dribbler_state)
    {
    case Dribbler_Start:
        // ROS_INFO("START");
        m_stand_state = Behavioral::Stand;
        m_dribbler_state = Dribbler_Run;
        wait_count_ = 1 * SPIN_RATE;
        break;

    case Dribbler_Run:
        // ROS_INFO("RUN");
        if (behavioral_.finishedKick)
        {
            ROS_INFO("SUDAH SELESAI KICKING WOIII");
            // count += 1;
            // if (s_role)
            // {
            //     if (m_robot_state == Robot_Kicking && count == 1)
            //         m_robot_state = Robot_Waiting;
            // }
            m_robot_state = Robot_Kicking;
            // ros::Duration(3).sleep();
            // behavioral_.setBodyModuleTo("quintic_walk", true);
            behavioral_.passerCheckFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, kick_angle());
            s_tracking_mode = BallTracker::Sweep;
            m_tracking_status = tracker_.processTracking(s_tracking_mode);
            behavioral_.startFollowing();
            behavioral_.kickingStatus = Behavioral::Not_Kicking;
            behavioral_.finishedKick = false;
            behavioral_.kickingDelay = false;
            behavioral_.lastKick = false;
        }
        // if (s_role)
        //     strategyStateA();
        // else
        //     strategyStateB();
        strategyStateA();

        break;
    }
}

void OlympicDribblerv13::standState()
{
    switch (m_stand_state)
    {
    case Behavioral::Stand:
        std::cout << "STAND" << std::endl;
        behavioral_.stopFollowing();
        behavioral_.setBodyModuleTo("action_module", true);
        ros::Duration(1).sleep();
        //behavioral_.playMotion(behavioral_.Motion_Index.WalkingReady);
        while (behavioral_.isActionRunning() == true)
            ros::Duration(5).sleep();
        behavioral_.setBodyModuleTo("quintic_walk", true);
        // behavioral_.startFollowing();
        m_stand_state = Behavioral::Nothing;
        break;

    case Behavioral::Nothing:
        break;

    default: // State 1 2 3 4
        std::cout << "FALLEN" << std::endl;
        behavioral_.stopFollowing();
        // behavioral_.fallen(m_stand_state); 
        while (behavioral_.isActionRunning() == true)
            ros::Duration(0.05).sleep();
        m_dribbler_state = Dribbler_Start;
        m_stand_state = Behavioral::Nothing;
        break;
    }
}

void OlympicDribblerv13::strategyStateA()
{
    // ballFilter(tracker_.ballPos_x,tracker_.ballPos_y, ball_x_filter, ball_y_filter);
    
    std::cout<< "ODOM X: "<< behavioral_.odometry_.x<<std::endl;
    std::cout<< "ODOM Y: "<< behavioral_.odometry_.y<<std::endl;
    // std::cout<< "BALL X: "<< ball_x_filter <<std::endl;
    // std::cout<< "BALL Y: "<< ball_y_filter <<std::endl;
    std::cout << "KICK ANGLE : " << kick_angle() << std::endl;
    std::cout << "Friend Status : " << behavioral_.friend_status << std::endl;

    switch (m_robot_state)
    {
    case Robot_Kicking:
        s_tracking_mode = BallTracker::Line;

        ROS_WARN("KICKING====================>");

        // if (firstIn)
        //     behavioral_.startFollowing();

        // firstIn = false;
        behavioral_.friend_status = false;
        behavioral_.lastKick = false;

        ballState(0);

        break;
    

    case Robot_Waiting:
        ROS_WARN("WAITING=================>");

        ROS_INFO("<= = = => %f", behavioral_.m_yaw);

        if (behavioral_.friend_status == 1){
            behavioral_.startFollowing();

            // ROS_INFO("=====STABLE WALK==== : %d", stable_walk);
            if(behavioral_.stableWalk() == 1)
            {
                ROS_ERROR("=====UDAH STABLE WALK====");
                m_robot_state = Robot_Kicking;
                behavioral_.firstIn = false;
                // firstIn = true;
            }
        }
        else{
            ROS_INFO("=====FRIEND STATUS BELUM TRUE====");
            behavioral_.stopFollowing();
            // behavioral_.abiWaiting();
        }

        s_tracking_mode = BallTracker::Nod;

        // commServer.init();
        // if(commServer.receiveMsg()){
        //     behavioral_.friend_status = true;
        //     ROS_WARN("ALFAAAA SUDAAAAH PASSSSINGGGGG");
        // }
        ROS_INFO("DRIBBLERRRRR V13");
        
        break;

    case Robot_Positioning:
        s_tracking_mode = BallTracker::Sweep;

        ROS_WARN("POSITIONING=================>");

        if(behavioral_.timePositioning())
        {
            firstIn = true;
            m_robot_state = Robot_Kicking;
        }
        // positioning_finished = behavioral_.timePositioning();
        // if(positioning_finished)
        // {
        //     firstIn = true;
        //     m_robot_state = Robot_Kicking;
        // }
        break;
 }

}

void OlympicDribblerv13::strategyStateB()
{
    switch (m_robot_state)
    {
    case Robot_Positioning:
        ROS_WARN("POSITIONING=================>");

        s_camera_track = Ball;

        if (behavioral_.friend_status)
            m_robot_state = Robot_Waiting;
        else
            behavioral_.stopFollowing();
        break;

    case Robot_Waiting:
        ROS_WARN("WAITING====================>");

        s_camera_track = Ball;

        if (behavioral_.timePositioning())
        {
            // if(true)
            if (behavioral_.friend_status)
                m_robot_state = Robot_Kicking;
            else
                behavioral_.stopFollowing();
        }
        break;

    case Robot_Dribbling:
        ROS_WARN("DRIIBBLING=================>");
        s_camera_track = Ball;

        if (behavioral_.AlignToOri(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, 0))
            if (behavioral_.dribblePasser())
                m_robot_state = Robot_Dribbling;

        break;

    case Robot_Kicking:
        ROS_WARN("KICKING=================>");
        s_camera_track = Ball;

        static bool firstIn = true;

        if (firstIn)
            behavioral_.startFollowing();

        firstIn = false;

        ballState(ball_to_friend());

        if (count == 2)
            behavioral_.stopFollowing();
        break;
    }
}

double OlympicDribblerv13::kick_angle(){
    double koor_bola_x = behavioral_.odometry_.x;// + (tracker_.ballPos_x * cos(behavioral_.s_angle_yaw) + tracker_.ballPos_y * sin(behavioral_.s_angle_yaw));
    double koor_bola_y = behavioral_.odometry_.y;// + (-tracker_.ballPos_x * sin(behavioral_.s_angle_yaw) + tracker_.ballPos_y * cos(behavioral_.s_angle_yaw));

    double goalpost_x = 900 /*875*/, goalpost_y = 340; //y=150,320, 20

    double kick_angle = atan2(fabs(goalpost_y - koor_bola_y), goalpost_x - koor_bola_x) * 180 / M_PI;
    // double kick_angle = atan2(koor_bola_y - goalpost_y  , goalpost_x - koor_bola_x) * 180 / M_PI;
    return (koor_bola_y>=goalpost_y) ? -kick_angle : kick_angle;
    // return -30;
}

double OlympicDribblerv13::ball_to_friend(){
    double koor_bola_x = behavioral_.odometry_.x;// + (tracker_.ballPos_x * cos(behavioral_.s_angle_yaw) + tracker_.ballPos_y * sin(behavioral_.s_angle_yaw));
    double koor_bola_y = behavioral_.odometry_.y;// + (-tracker_.ballPos_x * sin(behavioral_.s_angle_yaw) + tracker_.ballPos_y * cos(behavioral_.s_angle_yaw))
    double ball_target_x = 575, ball_target_y = 160;

    double ball_target = atan2(ball_target_y - koor_bola_y, ball_target_x - koor_bola_x) * 180 / M_PI;
    return ball_target;
}


void OlympicDribblerv13::ballState(double orientation)
{

    switch (m_tracking_status)
    {

    case BallTracker::Found:

        if (!s_first_found)
        {
            s_first_found = true;
            behavioral_.passerCheckFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, orientation);
    
            }
        // ROS_WARN("BALL STATE AAAAAAAAAAAAAAAAAAAAA");
        behavioral_.dribblerFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, orientation);
        behavioral_.searchState = Behavioral::LookAround;
        m_resetTime = true;
        break;

    case BallTracker::NotFound:

        if (m_resetTime)
            p_resetTimeStart = ros::Time::now().toSec();
        m_resetTime = false;
        resetTimeNow = ros::Time::now().toSec() - p_resetTimeStart;
        if (resetTimeNow > 2)
            s_first_found = false;
        else
            break;

        behavioral_.passerSearch(s_role);
        break;
    }
}

// void OlympicDribblerv13::ballFilter(double ballPos_x, double ballPos_y, double ball_x_filter, double ball_y_filter){
    
//     static std::vector<int> ball_x_ma,ball_y_ma;
//     int size_buff = 5;
 
//     ball_x_ma.insert(ball_x_ma.end(), ballPos_x);
//     ball_y_ma.insert(ball_y_ma.end(), ballPos_y);

//     if(ball_x_ma.size() < size_buff){
//             ball_x_filter = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / ball_x_ma.size());
//             ball_y_filter = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / ball_y_ma.size());
//     } else{
//             ball_x_filter = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / size_buff);
//             ball_x_ma.erase(ball_x_ma.begin());
//             ball_y_filter = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / size_buff);
//             ball_y_ma.erase(ball_y_ma.begin());
//     }

//     ball_x_filter = fabs(ball_x_filter) - 10;
// }

bool OlympicDribblerv13::goalState()
{

    switch (m_tracking_status)
    {
    case GoalpostTracker::Found:

        if (behavioral_.positioning(goalTracker_.goalpostPos_x, goalTracker_.goalpostPos_y))
            return true;

        break;

    case GoalpostTracker::NotFound:

        behavioral_.passerSearch(s_role);

        break;
    }
    return false;
}


} // namespace robotis_op
