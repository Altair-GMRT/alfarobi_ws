#include "alfarobi/kepeer.h"
#include "alfarobi/alfarobi.h"
#include "alfarobi/passer.h"
namespace robotis_op
{

KeeperDemo::KeeperDemo()
    : SPIN_RATE(30),
      wait_count_(0),
      DEBUG_PRINT(false),
      nh_(ros::this_node::getName()),
      m_keeper_state(Keeper_Start),
      m_robot_status(Waited),
      s_tracking_mode(BallTracker::Sweep),
      s_ball_prediction(false),
      s_camera_track(0)

{
    //init ros
    enable_ = false;

    //S = STRATEGY
    s_start = false;
    s_first_play = true;
    m_first_zone = false;
    m_search_status = false;
    m_positioning = false;
    s_pickup = false;
    bool nth_cycle = false; 

    std::string default_path = ros::package::getPath("op3_gui_demo") + "/config/demo_config.yaml";
    std::string path = nh_.param<std::string>("demo_config", default_path);

    nh_.param<bool>("calibration", s_calibration, false);

    boost::thread queue_thread_  = boost::thread(boost::bind(&KeeperDemo::queueThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&KeeperDemo::processThread, this));
}

KeeperDemo::~KeeperDemo()
{
}

void KeeperDemo::queueThread()
{
    demo_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &KeeperDemo::CommandCallback, this);
    ros::Rate rate(30);

    while (nh_.ok())
    {
        ros::spinOnce();

        rate.sleep();
    }
}

void KeeperDemo::CommandCallback(const std_msgs::String::ConstPtr &msg)
{
    if (enable_ == false)
        return;

    if (msg->data == "start")
    {
        if (keeperState() == true)
            stopKeeperMode();
        else
            startKeeperMode();
    }
    else if (msg->data == "stop")
    {
        stopKeeperMode();
    }
}

void KeeperDemo::setModuleEnable()
{
    enable_ = true;
    s_start = false;

    startKeeperMode();
}

// /robotis/head_control/scan_command
// /robotis/head_control/set_joint_states
// /robotis/head_control/set_joint_states_offset

void KeeperDemo::setModuleDisable()
{
    tracker_.stopTracking();
    behavioral_.stopFollowing();

    enable_ = false;

    behavioral_.setModuleTo("base_module");
}

void KeeperDemo::startKeeperMode()
{
    ROS_INFO("Start Keeper Demo");

    enable_ = true;

    m_keeper_state = Keeper_Start;
}

void KeeperDemo::stopKeeperMode()
{
    ROS_INFO("Stop Keeper Demo");

    enable_ = false;

    m_keeper_state = Keeper_Stop;
}

void KeeperDemo::processThread()
{
    ros::Rate loop_rate(SPIN_RATE);

    tracker_.startTracking();

    while (ros::ok())
    {
        if (wait_count_ <= 0)
        {
            if (enable_ == true)
            {
                // behavioral_.CALIBRATION = s_calibration;
                // s_communication = behavioral_.COMMUNICATION;
                // s_localization = behavioral_.LOCALIZATION;
                // tracker_.predictedBall(s_ball_prediction);
              

                // switch (s_camera_track)
                // {
                // case Ball:
                //     m_tracking_status = tracker_.processTracking(s_tracking_mode);
                //     break;

               
                // }
                // if (m_tracking_status == BallTracker::DontTrack )
               
                // {
                    m_stand_state = behavioral_.checkFallen();
                    m_button_status = behavioral_.buttonStatus;
                    m_game_state = behavioral_.comm_gameState;          //edit
                    s_allowed_to_move = behavioral_.comm_allowedToMove; //edit
                    m_strategy_state = Strategy::WorkAlone;
                        buttonState();
                        strategyState();
                    standState();
            }
        }
        else
            wait_count_ -= 1;    

        loop_rate.sleep()  ;  
    }
}

void KeeperDemo::buttonState()
{
    switch(m_button_status)
    {
        // case Alfarobi::L1:
        // s_communication = true;
        //     break;
        // case Alfarobi::L2:
        //     s_calibration = false;
        //     m_keeper_state = Keeper_Running;
        //     break;
        // case Alfarobi::L3:
        //     s_first_play = true;
        //     s_calibration = true;
        //     s_ball_prediction = false;
        //     // behavioral_.stopFollowing(); //edit
        //     m_keeper_state = Keeper_Waiting;//edit
        //     break;
        // case Alfarobi::L4:
        //     break;
        // case Alfarobi::R1:
        //     break;
        // case Alfarobi::R2:
        //     break;
        // case Alfarobi::R3:
        //     s_pickup = true;
        //     break;
        // case Alfarobi::R4:
        //     break;
        case Alfarobi::B1:
            m_keeper_state = Keeper_Start;
            ROS_INFO("B1 oyy;");
            break;
        case Alfarobi::B2:
            m_keeper_state = Keeper_Stop;
            break;
        case Alfarobi::B3:
            break;
        case Alfarobi::B4:
            break;
    }
}

void KeeperDemo::communicationState()
{
    bool positioning = false;

    switch(m_game_state)
    {
        case Game_Initial:
            std::cout<<"INITIAALLLLLLLLLLLLLLLLLLL"<<std::endl;
            behavioral_.mileage_xDist = 0;
            behavioral_.mileage_yDist = 0;
            s_first_play = true;
            s_ball_prediction = false;
            s_stop = false;
            if(behavioral_.walkingStatus)
                behavioral_.stopFollowing();
            if(!s_start)
                m_keeper_state = Keeper_Start;
            else
                m_keeper_state = Keeper_Waiting;
            break;

        case Game_Ready:
            std::cout<<"RRRRRRRRRRRRRRREADYYYYYYYYYYYYYYYY"<<std::endl;
            s_ball_prediction = false;
            s_stop = true;
            if(!s_start)    
                m_keeper_state = Keeper_Start;
            else
                m_keeper_state = Keeper_Waiting;  
            break;
        
        case Game_Set:
            std::cout<<"SEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEET"<<std::endl;
            if(behavioral_.walkingStatus)
                behavioral_.stopFollowing();
            s_first_play = true;
            s_ball_prediction = false;
            if(!s_start)    
                m_keeper_state = Keeper_Start;
            else
                m_keeper_state = Keeper_Waiting;
            behavioral_.mileage_xDist = 0;
            behavioral_.mileage_yDist = 0;
            break;

        case Game_Play:
            // if(!s_allowed_to_move)
            // {
                std::cout<<"PICKUPP----------------------------"<<std::endl;
                s_calibration = true;
                s_first_play = true;
                behavioral_.stopFollowing();
            // }
            // if(s_allowed_to_move == true && s_first_play == true)
            // {
                std::cout<<"PLAAAAAAAAAAAAAAAAAAAAAAAAAAYYYYYYY"<<std::endl;
                s_calibration = false;
                s_first_play = false;
                if(!s_start)    
                    m_keeper_state = Keeper_Start;
                else
                    m_keeper_state = Keeper_Running;
                m_stand_state = Behavioral::Nothing;
            // }
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
            m_keeper_state = Keeper_Stop;
            break;
    }
}

bool KeeperDemo::keeperState()
{
    bool keeperCondition;
    double waiting_time;

    switch(m_keeper_state)
    {
        case Keeper_Stop:
            std::cout<<"STOP"<<std::endl;
            keeperCondition = false;
            s_first_play = true;
            s_stop = true;
            tracker_.stopTracking();
            behavioral_.stopFollowing();
            wait_count_ = 0;
            break;

        case Keeper_Start:
            std::cout<<"START"<<std::endl;
            keeperCondition = true;
            s_waiting_start = true;
            s_first_play = true;
            s_start = true;
            m_stand_state = Behavioral::Stand;
            // s_tracking_mode = BallTracker::Square;
            m_keeper_state = Keeper_Running;
            wait_count_ = 1 * SPIN_RATE;
            break;

        case Keeper_Running:
            ROS_ERROR("RUNNINGGGGGGGGGGGGGGGGGGGGGGGG");
            keeperCondition = true;
            s_first_play = false;
            s_start = true;
            behavioral_.finishedKick = true; //edit
            if(behavioral_.finishedKick)
            {
                behavioral_.setBodyModuleTo("quintic_walk", true);
                m_following_state = behavioral_.checkFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta).x();
                behavioral_.startFollowing();
                behavioral_.kickingStatus = Behavioral::Not_Kicking;
                behavioral_.finishedKick = false;
            }
            if((behavioral_.kickingDelay==true && behavioral_.kickTimeNow > behavioral_.KICK_DELAY) || behavioral_.walkingStatus == false)
                behavioral_.startFollowing();
            strategyState();
            break;

        case Keeper_Waiting:
            keeperCondition = true;
            if(s_waiting_start == true)
            {
                s_waiting_start = false;
                p_waiting_start = ros::Time::now().toSec();
            }
            waiting_time = ros::Time::now().toSec() - p_waiting_start;
            if(waiting_time > 1 && tracker_.robotState_x != 999 && tracker_.robotState_y != 999)
            {
                std::cout << "TRACKING BOLAAng" << std::endl;
                s_waiting_start = true;
                s_tracking_mode = BallTracker::Sweep;
                if(s_communication == true)
                    m_keeper_state = Keeper_Running;
            }
            break;    
    }

    return keeperCondition;
}

void KeeperDemo::standState()
{
    switch (m_stand_state)
    {
        case Behavioral::Stand:
            std::cout<<"STAND"<<std::endl;
            behavioral_.stopFollowing();
            behavioral_.setModuleTo("action_module");
            ros::Duration(0.1).sleep();
            behavioral_.playMotion(behavioral_.Motion_Index.WalkingReady);
            while(behavioral_.isActionRunning() == true)
                ros::Duration(0.05).sleep();
            behavioral_.setBodyModuleTo("quintic_walk", true);
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
            m_keeper_state = Keeper_Start;
            break;
    }
}

void KeeperDemo::strategyState()
{        
    // double timeFound;

    switch(m_strategy_state)
    {
        case Strategy::WorkAlone:
            if(s_pickup)
            {
                std::cout<<"POSITIONINGGGGGGGGGGGGGGG"<<std::endl;
                if((behavioral_.kickingDelay==true && behavioral_.kickTimeNow > behavioral_.KICK_DELAY) || behavioral_.walkingStatus == false)
                    behavioral_.startFollowing();
                m_positioning = behavioral_.forward(30);
                if(m_positioning)
                {
                    behavioral_.stopFollowing();
                    s_pickup = false;
                }
                else
                    break; 
            }

            // switch(m_tracking_status)
            // {
            //     case BallTracker::Found:
            //         break;

            //     case BallTracker::NotFound:
            //         p_first_found = ros::Time::now().toSec();
            //         s_tracking_mode = BallTracker::Sweep;
            //         break;
            // }
            // timeFound = ros::Time::now().toSec() - p_first_found;
            // behavioral_.gabKeeper(Behavioral::Fallen_Mode, tracker_.robotState_x, tracker_.robotState_y, tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta, timeFound);
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

}
