#include "alfarobi/soccer.h"
#include "alfarobi/alfarobi.h"

namespace robotis_op
{

SoccerDemo::SoccerDemo()
    : SPIN_RATE(30),
      wait_count_(0),
      DEBUG_PRINT(false),
      nh_(ros::this_node::getName()),
      m_soccer_state(Soccer_Start),
      m_robot_status(Waited),
      s_tracking_mode(BallTracker::Sweep),
      s_ball_prediction(false),
      p_localization_border(300),
      p_stupid_localization_border(150)
{
    //init ros
    enable_ = false;

    //S = STRATEGY
    s_start = false;
    s_stop = true;
    s_first_play = true;
    m_first_zone = false;
    m_search_status = false;
    m_positioning = false;
    m_resetTime = true;
    s_pickup = false;

    std::string default_path = ros::package::getPath("op3_gui_demo") + "/config/demo_config.yaml";
    std::string path = nh_.param<std::string>("demo_config", default_path);

    nh_.param<bool>("calibration", s_calibration, false);

    boost::thread queue_thread_  = boost::thread(boost::bind(&SoccerDemo::queueThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&SoccerDemo::processThread, this));
}

SoccerDemo::~SoccerDemo()
{
}

void SoccerDemo::queueThread()
{
    demo_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &SoccerDemo::CommandCallback, this);
    ros::Rate rate(30);

    while (nh_.ok())
    {
        ros::spinOnce();

        rate.sleep();
    }
}

void SoccerDemo::CommandCallback(const std_msgs::String::ConstPtr &msg)
{
    if (enable_ == false)
        return;

    if (msg->data == "start")
    {
        if (soccerState() == true)
            stopSoccerMode();
        else
            startSoccerMode();
    }
    else if (msg->data == "stop")
    {
        stopSoccerMode();
    }
}

void SoccerDemo::setModuleEnable()
{
    enable_ = true;
    s_start = false;

    startSoccerMode();
}

void SoccerDemo::setModuleDisable()
{
    tracker_.stopTracking();
    behavioral_.stopFollowing();

    enable_ = false;

    behavioral_.setModuleTo("base_module");
}

void SoccerDemo::startSoccerMode()
{
    ROS_INFO("Start Soccer Demo");

    enable_ = true;

    m_soccer_state = Soccer_Start;
}

void SoccerDemo::stopSoccerMode()
{
    ROS_INFO("Stop Soccer Demo");

    enable_ = false;

    m_soccer_state = Soccer_Stop;
}

void SoccerDemo::processThread()
{
    ros::Rate loop_rate(SPIN_RATE);

    tracker_.startTracking();

    while (ros::ok())
    {
        if (wait_count_ <= 0)
        {
            if (enable_ == true)
            {
                behavioral_.CALIBRATION = s_calibration;
                s_communication = behavioral_.COMMUNICATION;
                s_localization = behavioral_.LOCALIZATION;
                tracker_.predictedBall(s_ball_prediction);
                m_score_state = behavioral_.comm_scoreChanged();
                
                if(behavioral_.kickingDelay)
                    m_tracking_status = BallTracker::Found;
                else
                    m_tracking_status = tracker_.processTracking(s_tracking_mode);

                if(m_tracking_status == BallTracker::DontTrack)
                    tracker_.startTracking();
                else
                {
                    m_stand_state = behavioral_.checkFallen();
                    m_button_status = behavioral_.buttonStatus;
                    m_game_state = behavioral_.comm_gameState;
                    m_game_time = behavioral_.comm_timeGame;
                    s_allowed_to_move = behavioral_.comm_allowedToMove;
                    m_strategy_state = Strategy::WorkAlone;
                    if(m_stand_state == Behavioral::Nothing)
                    {
                        buttonStatus();
                        if(s_communication == true)
                            communicationState();
                        soccerState();
                    }
                    standState();
                    // robotCommunication();
                }
                s_tracking_mode = behavioral_.trackingMode;
            }
        }
        else
            wait_count_ -= 1;

        loop_rate.sleep()  ;
    }
}

void SoccerDemo::buttonStatus()
{
    switch(m_button_status)
    {
        // case Alfarobi::L1:
        //     break;
        // case Alfarobi::L2:
        //     s_calibration = false;
        //     behavioral_.startFollowing();
        //     m_soccer_state = Soccer_Running;
            // break;
        // case Alfarobi::L3:
        //     s_first_play = true;
        //     s_calibration = true;
        //     s_ball_prediction = false;
        //     behavioral_.stopFollowing();
        //     break;
        // case Alfarobi::L4:
        //     break;
        // case Alfarobi::R1:
        //     break;
        // case Alfarobi::R2:
        //     break;
        // case Alfarobi::R3:
        //     s_pickup = true;
        //     behavioral_.mileage_xDist = 0;
        //     behavioral_.mileage_yDist = 0;
        //     break;
        // case Alfarobi::R4:
        //     break;
        case Alfarobi::B1:
            break;
        case Alfarobi::B2:
            break;
        case Alfarobi::B3:
            break;
        case Alfarobi::B4:
            break;
    }
}

void SoccerDemo::communicationState()
{
    bool positioning = false;
    double positioningTime;

    switch(m_game_state)
    {
        case Game_Initial:
            std::cout<<"INITIAALLLLLLLLLLLLLLLLLLL"<<std::endl;
            behavioral_.mileage_xDist = 0;
            behavioral_.mileage_yDist = 0;
            s_first_play = true;
            s_ball_prediction = false;
            s_stop = false;
            behavioral_.stopFollowing();
            if(!s_start)
                m_soccer_state = Soccer_Start;
            break;

        case Game_Ready:
            // std::cout<<"RRRRRRRRRRRRRRREADYYYYYYYYYYYYYYYY"<<std::endl;
            s_ball_prediction = false;
            if(m_game_time >= behavioral_.GAME_TIME || m_score_state == true || s_positioning == true)
            {
                s_positioning = true;
                behavioral_.swap_score = false;
                if(!s_start)
                    m_soccer_state = Soccer_Start;
                if(s_calibration)
                    break;
                if(s_first_play)
                {
                    m_positioning_start = ros::Time::now().toSec();
                    behavioral_.mileage_xDist = 0;
                    behavioral_.mileage_yDist = 0;
                    behavioral_.startFollowing();
                    s_first_play = false;
                    s_stop = false;
                }
                std::cout<<"Positioning"<<std::endl;
                if(!s_stop)
                {
                    s_stop = behavioral_.forward(200);
                    std::cout<<"FORWARD DONE : "<<s_stop<<"\tMILEAGE : "<<behavioral_.mileage_xDist<<std::endl;
                }
                else
                {
                    s_stop = true;
                    behavioral_.stopFollowing();
                    m_soccer_state = Soccer_Waiting;
                }
            }
            else
            {
                if(!s_start)
                    m_soccer_state = Soccer_Start;
                std::cout<<"Drop Ball"<<std::endl;
                s_first_play = true;
                behavioral_.stopFollowing();
            }
            break;

        case Game_Set:
            std::cout<<"SEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEET"<<std::endl;
            behavioral_.swap_score = false;
            if(behavioral_.walkingStatus)
                behavioral_.stopFollowing();
            s_first_play = true;
            behavioral_.swap_score = false;
            s_ball_prediction = false;
            if(!s_start)
                m_soccer_state = Soccer_Start;
            else
                m_soccer_state = Soccer_Waiting;
            behavioral_.mileage_xDist = 0;
            behavioral_.mileage_yDist = 0;
            break;

        case Game_Play:
            if(!s_allowed_to_move)
            {
                std::cout<<"PICKUPP----------------------------"<<std::endl;
                s_calibration = true;
                s_first_play = true;
                behavioral_.stopFollowing();
            }
            if(s_allowed_to_move == true && s_first_play == true)
            {
                std::cout<<"PLAAAAAAAAAAAAAAAAAAAAAAAAAAYYYYYYY"<<std::endl;
                s_calibration = false;
                s_first_play = false;
                if(!s_start)
                    m_soccer_state = Soccer_Start;
                else
                    m_soccer_state = Soccer_Running;
                m_stand_state = Behavioral::Nothing;
                if(s_calibration)
                    break;
                behavioral_.startFollowing();
            }
            break;

        case Game_Finish:
            std::cout<<"FINISH..................................."<<std::endl;
            s_start = false;
            s_stop = true;
            s_first_play = true;
            m_first_zone = false;
            m_search_status = false;
            m_positioning = false;
            s_ball_prediction = false;
            m_soccer_state = Soccer_Stop;
            break;
    }
}

bool SoccerDemo::soccerState()
{
    bool soccerCondition;
    double waiting_time;

    switch(m_soccer_state)
    {
        case Soccer_Stop:
            std::cout<<"STOP"<<std::endl;
            soccerCondition = false;
            s_first_play = true;
            s_stop = true;
            tracker_.stopTracking();
            behavioral_.stopFollowing();
            wait_count_ = 0;
            break;

        case Soccer_Start:
            std::cout<<"START"<<std::endl;
            soccerCondition = true;
            s_waiting_start = true;
            s_first_found = false;
            s_first_play = true;
            s_start = true;
            m_first_zone = false;
            m_search_status = false;
            m_stand_state = Behavioral::Stand;
            s_tracking_mode = BallTracker::Square;
            m_soccer_state = Soccer_Running;
            wait_count_ = 1 * SPIN_RATE;
            break;

        case Soccer_Running:
            if(s_communication == true && (s_allowed_to_move == false || m_game_state != Game_Play || m_score_state == true))
                break;
            soccerCondition = true;
            s_first_play = false;
            s_start = true;
            if(behavioral_.finishedKick)
            {
                behavioral_.setBodyModuleTo("quintic_walk", true);
                m_following_state = behavioral_.checkFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta).x();
                behavioral_.startFollowing();
                behavioral_.kickingStatus = Behavioral::Not_Kicking;
                behavioral_.finishedKick = false;
                behavioral_.kickingDelay = false;
            }
            if((behavioral_.kickingDelay==true && behavioral_.kickTimeNow > behavioral_.KICK_DELAY) || behavioral_.walkingStatus == false)
                behavioral_.startFollowing();
            // if(tracker_.robotState_x != 999 && tracker_.robotState_y != 999)
                strategyState();
            // else
            //     m_soccer_state = Soccer_Waiting;
            break;

        case Soccer_Waiting:
            if(s_communication == true && (s_allowed_to_move == false || m_game_state != Game_Play || m_score_state == true))
                break;
            std::cout<<"WAITING.........."<<std::endl;
            soccerCondition = true;
            behavioral_.stopFollowing();
            s_tracking_mode = BallTracker::Square;
            behavioral_.searchState = Behavioral::FirstZone;
            // if(s_waiting_start == true)
            // {
            //     s_waiting_start = false;
            //     p_waiting_start = ros::Time::now().toSec();
            // }
            // waiting_time = ros::Time::now().toSec() - p_waiting_start;
            // if(waiting_time > 7)
            //     m_soccer_state = Soccer_Stop;
            if(tracker_.robotState_x != 999 && tracker_.robotState_y != 999)
            {
                // s_waiting_start = true;
                s_tracking_mode = BallTracker::Sweep;
                if(s_communication == false || s_allowed_to_move == true)
                    m_soccer_state = Soccer_Running;
            }
            break;
    }

    return soccerCondition;
}

void SoccerDemo::standState()
{
    switch (m_stand_state)
    {
        case Behavioral::Stand:
            std::cout<<"STAND"<<std::endl;
            behavioral_.stopFollowing();
            behavioral_.setBodyModuleTo("action_module", true);
            ros::Duration(1).sleep();
            //behavioral_.playMotion(behavioral_.Motion_Index.WalkingReady);
            while(behavioral_.isActionRunning() == true)
                ros::Duration(5).sleep();
            behavioral_.setBodyModuleTo("quintic_walk", true);
            behavioral_.startFollowing();
            m_stand_state = Behavioral::Nothing;
            break;

        case Behavioral::Nothing:
            break;

        default:
            std::cout<<"FALLEN"<<std::endl;
            behavioral_.stopFollowing();
            behavioral_.fallen(m_stand_state);
            while(behavioral_.isActionRunning() == true)
                ros::Duration(0.05).sleep();
            m_soccer_state = Soccer_Start;
            m_stand_state = Behavioral::Nothing;
            break;
    }
}

void SoccerDemo::strategyState()
{
    double resetTimeNow;

    switch(m_strategy_state)
    {
        case Strategy::WorkAlone:
            if(s_pickup)
            {
                std::cout<<"POSITIONINGGGGGGGGGGGGGGG\tMILEAGE : "<<behavioral_.mileage_xDist<<std::endl;
                if((behavioral_.kickingDelay==true && behavioral_.kickTimeNow > behavioral_.KICK_DELAY) || behavioral_.walkingStatus == false)
                    behavioral_.startFollowing();
                m_positioning = behavioral_.forward(300);
                if(m_positioning)
                {
                    behavioral_.stopFollowing();
                    s_pickup = false;
                    behavioral_.mileage_xDist = 0;
                    behavioral_.mileage_yDist = 0;
                }
                else
                    break;
            }

            switch(m_tracking_status)
            {
                case BallTracker::Found:
                    s_pickup = false;
                    if(tracker_.ballPos_x > behavioral_.FORWARD_MIN_LIMIT || fabs(tracker_.ballPos_y) > behavioral_.RL_MIN_LIMIT )
                        s_ball_prediction = true;
                    else
                        s_ball_prediction = false;
                    behavioral_.mileage_xDist = 0;
                    behavioral_.mileage_yDist = 0;
                    if(!s_first_found)
                    {
                        s_first_found = true;
                        m_following_state = behavioral_.checkFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta).x();
                        behavioral_.followState = behavioral_.lastFollowState = behavioral_.checkFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta).y();
                        std::cout<<"FOLLOWING STATE ======================== "<<m_following_state<<std::endl;
                    }
                    behavioral_.following(m_following_state, tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta);
                    behavioral_.searchState = Behavioral::LookAround;
                    behavioral_.lookStatus = true;
                    m_resetTime = true;
                    break;

                case BallTracker::NotFound:
                    std::cout<<"X: "<<tracker_.ballPos_x<<" Y: "<<tracker_.ballPos_y<<std::endl;
                    if(m_resetTime)
                        p_resetTimeStart = ros::Time::now().toSec();
                    m_resetTime = false;
                    resetTimeNow = ros::Time::now().toSec() - p_resetTimeStart;
                    if(resetTimeNow > 2)
                        s_first_found = false;
                    else
                        break;
                    behavioral_.kickingDelay = false;
                    s_ball_prediction = true;
                    if(s_localization)
                        behavioral_.smartSearch(tracker_.robotState_x, tracker_.robotState_y, tracker_.robotState_theta, p_localization_border);
                    else
                        behavioral_.stupidSearch(tracker_.robotState_x, tracker_.robotState_y, tracker_.robotState_theta, p_stupid_localization_border);
                    break;
            }
            break;

        case Strategy::GoToBall:
            break;

        case Strategy::WaitHere:
            break;

        case Strategy::AlignToBall:
            break;

        case Strategy::SearchBall:
            break;
    }
}

void SoccerDemo::robotCommunication(){
    //Nanti ubah load datanya dari beberapa config
    robot_info.robot_name = "ROBI";
    robot_info.robot_id = 2;
    robot_info.robot_pos.x = 0;
    robot_info.robot_pos.y = 0;
    robot_info.robot_pos.theta = 0;//pake orientasi robot harusnya
    robot_info.robot_status = 0;
    robot_info.ball_status = 0;
    robot_info.is_moving = 0;
}

}
