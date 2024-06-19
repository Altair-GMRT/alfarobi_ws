#include "alfarobi/penalty_manual.h"
#include "alfarobi/alfarobi.h"

namespace robotis_op
{
OlympicPenalty::OlympicPenalty()
    : SPIN_RATE(30),
      wait_count_(0),
      nh_(ros::this_node::getName()),
      s_camera_track(0),
      firstIn(true)
{
    enable_ = false;
    kick = false;
    s_pickup = false;
    m_positioning = false;
    m_rotating = false;
    line_trigger = false;
    //penalti baru
    robot_penalty = false;
    penaltySelesai = false;
    //
    nh_.param<bool>("calibration", s_calibration, false);
    // std::srand(100);
    std::srand((unsigned) time(0));
    m_robot_state = Robot_Start;
    //penalti baru
    m_penalty_state =  Penalty_Start;
    //
    boost::thread queue_thread_ = boost::thread(boost::bind(&OlympicPenalty::queueThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&OlympicPenalty::processThread, this));
}

OlympicPenalty::~OlympicPenalty()
{
}

void OlympicPenalty::queueThread()
{
    demo_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &OlympicPenalty::CommandCallback, this);
   
    ros::Rate rate(30);

    while (nh_.ok())
    {
        ros::spinOnce();

        rate.sleep();
    }
}

void OlympicPenalty::CommandCallback(const std_msgs::String::ConstPtr &msg) 
{
    if (enable_ == false)
        return;
}

void OlympicPenalty::setModuleEnable()
{
    count = 0;
    kick = false;
    enable_ = true;
    startPenaltyMode();
}

void OlympicPenalty::setModuleDisable()
{
    tracker_.stopTracking();
    behavioral_.stopFollowing();

    enable_ = false;

    behavioral_.setModuleTo("base_module");
}

void OlympicPenalty::startPenaltyMode()
{
    ROS_INFO("Start Penalty Demo");

    enable_ = true;

    m_passer_state = Passer_Start;
}

void OlympicPenalty::stopPenaltyMode()
{
    ROS_INFO("Stop Penalty Demo");

    enable_ = false;

    m_passer_state = Passer_Stop;
}

void OlympicPenalty::processThread()
{
    ros::Rate loop_rate(SPIN_RATE);
    kick = false;
    tracker_.startTracking();

    while (ros::ok())
    {
        if (wait_count_ <= 0)
        {
            if (enable_ == true)
            {
                behavioral_.CALIBRATION = s_calibration;
                switch (s_camera_track)
                {  
                case Ball:
                    s_tracking_mode = BallTracker::Nod;
                    m_tracking_status = tracker_.processTracking(s_tracking_mode);
                    break;
                }

                if (m_tracking_status == BallTracker::DontTrack )
                {
                    if (s_camera_track == Ball)
                        tracker_.startTracking();
                }
                else
                {
                    // check_fallen = behavioral_.checkFallen();
                    m_button_status = behavioral_.buttonStatus;
                    // if (check_fallen){
                    //     behavioral_.stopFollowing();
                    //     m_robot_state = Robot_Start;
                    //     check_fallen= false;
                    // }

                    if (m_stand_state == Behavioral::Nothing)
                    {
                        buttonStatus();
                        penaltyState();
                    }
                    standState();
                }
            }
        }
        else
            wait_count_ -= 1;

        loop_rate.sleep();
    }
}

void OlympicPenalty::buttonStatus()
{
    switch (m_button_status)
    {
    // case Alfarobi::L1: //start
    //     // // behavioral_.send();
    //     // // behavioral_.friend_status = true;
    //     m_robot_state = Robot_Waiting;
    //     // kick = true;
    //     // behavioral_.firstKick = false;
    //     // // behavioral_.stopFollowing();
    //     break;
    // case Alfarobi::L2: //jangan dipake
    //     // s_calibration = false;
    //     // behavioral_.startFollowing();
    //     // m_passer_state = Passer_Run;
    //     break;
    // case Alfarobi::L3: //jangan dipake
    //     // s_calibration = true;
    //     // // if(!s_role && count > 1)
    //     // behavioral_.stopFollowing();
    //     // count = 5;
    //     break;
    // case Alfarobi::L4: //reset
    //     behavioral_.startFollowing();
    //     m_robot_state = Robot_Positioning;
    //     behavioral_.lastKick = false;
    //     m_positioning = false;
    //     count = 0;
    //     behavioral_.CALIBRATION = false;
    //     firstIn = false;
    //     // behavioral_.resetComm();
    //     // behavioral_.resetFSM();
    //     // behavioral_.m_pass_state = Pass_Forward;
    //     // behavioral_.stopFollowing();
    //     behavioral_.friend_status = false;
        
    //     break;
    // case Alfarobi::R1:
    //     // behavioral_.stopFollowing();
    //     behavioral_.startFollowing();
    //     // m_robot_state = Robot_Running;
    //     robot_penalty = true;
    //     // ros::Duration(2).sleep();
    //     // //pengTruePublish();
    //     // // kick = true;
    //     // // behavioral_.firstKick = false;
    //     // // // behavioral_.stopFollowing();
    //     // behavioral_.send();
    //     // behavioral_.alfaStartPass();
    //     // behavioral_.friend_status=true;
    //     // behavioral_.startFollowing();

    //     break;
    // case Alfarobi::R2:
    //     behavioral_.resetOffset = behavioral_.m_yaw;
    //     break;
    // case Alfarobi::R3: //pickup
    //     // s_pickup = true;
    //     // behavioral_.mileage_xDist = 0;
    //     // behavioral_.mileage_yDist = 0;
    //     // behavioral_.startFollowing();
    //     m_robot_state = Robot_Stop;
    //     // m_positioning = false;
    //     //behavioral_.stopFollowing();
    //     // behavioral_.friend_status = false;
    //     // behavioral_.bo_ol = true;
    //     break;
    // case Alfarobi::R4: //jangan dipake
    //     robot_penalty = true;
        
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

void OlympicPenalty::penaltyState()
{
    switch (m_penalty_state)
    {
    case Penalty_Start:
        ROS_INFO("START");
        m_stand_state = Behavioral::Stand;
        m_penalty_state = Penalty_Run;
        wait_count_ = 1 * SPIN_RATE;
        break;

    case Penalty_Run:
        ROS_INFO("RUN");
        
        // // behavioral_.stopFollowing();
        // // m_penalty_state = Penalty_Start;
        // ros::Duration(3).sleep();
        // behavioral_.setBodyModuleTo("quintic_walk", true);
        // behavioral_.kickingStatus = Behavioral::Not_Kicking;
        // behavioral_.finishedKick = false;
        // behavioral_.kickingDelay = false;
      if (behavioral_.finishedKick)
        {   
            #ifdef STATE
            ROS_INFO("SUDAH SELESAI KICKING WOIII");
            #endif
            // count += 1;
            // if (s_role)
            // {
            //     if (m_robot_state == Robot_Kicking && count == 1)
            //         m_robot_state = Robot_Waiting;
            // }
            // if(s_role)
            // {
                m_robot_state = Robot_Start;
                ros::Duration(3).sleep();
                behavioral_.setBodyModuleTo("quintic_walk", true);
                // behavioral_.passerCheckFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, penalty_angle());
                // s_tracking_mode = BallTracker::Line;
                // m_tracking_status = tracker_.processTracking(s_tracking_mode);
                // behavioral_.startFollowing();
                behavioral_.kickingStatus = Behavioral::Not_Kicking;
                behavioral_.finishedKick = false;
                behavioral_.kickingDelay = false;
        }


        strategyState();


        break;
    }
}

void OlympicPenalty::standState()
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

    // default: // State 1 2 3 4
    //     std::cout << "FALLEN" << std::endl;
    //     behavioral_.stopFollowing();
    //     behavioral_.fallen(m_stand_state); 
    //     while (behavioral_.isActionRunning() == true)
    //         ros::Duration(0.05).sleep();
    //     m_passer_state = Passer_Start;
    //     m_stand_state = Behavioral::Nothing;
    //     break;
    }
}


//penalti baru
//blm tambahin button
void OlympicPenalty::strategyState()
{
    static int goalpost_y_input;
    switch (m_robot_state)
    {
    case Robot_Start:
        ROS_WARN("<====START PENALTY====>");
        m_stand_state = Behavioral::Nothing;
        m_tracking_status = BallTracker::Nod;
        s_camera_track = Ball; //ke default, camera dimatiin
        // std::srand(100); //biar hasil randomnya ga sama terus
        // rand_penalty = std::rand(); 
        RNG = 1 + (rand() % 5) ; //CARI CARA LEBIH EFEKTIF BUAR ANGKA ACAK
        if (robot_penalty)
            {
                m_robot_state = Robot_Running;
                robot_penalty = false;
        }
        break;

    case Robot_Running:
        ROS_WARN("<====PENALTY RUNNING====>");
        //random number generator, modulo 5, blm bikin
        goalpost_y = titikTendangan(RNG);
        std::cout<<"goalpost_y : "<<goalpost_y<<std::endl;
        // ROS_INFO("RNG : %i", RNG);
        std::cout << "RNG [" << RNG << "]\n";
        behavioral_.startFollowing();
        // behavioral_.penaltyGoal(penalty_angle(goalpost_y), penaltySelesai);
        ballState(penalty_angle(goalpost_y_input));
        // if(penaltySelesai)
        //     m_penalty_state = Penalty_Stop
        break;

    case Robot_Stop:
        ROS_WARN("<====PENALTY STOP====>");
        behavioral_.stopFollowing();
        break;

    }
}
//

//penalti baru
double OlympicPenalty::penalty_angle(double kickDirection){
    double koor_bola_x = behavioral_.odometry_.x;// + (tracker_.ballPos_x * cos(behavioral_.s_angle_yaw) + tracker_.ballPos_y * sin(behavioral_.s_angle_yaw));
    double koor_bola_y = behavioral_.odometry_.y;// + (-tracker_.ballPos_x * sin(behavioral_.s_angle_yaw) + tracker_.ballPos_y * cos(behavioral_.s_angle_yaw));

    double goalpost_x = 950, goalpost_y; 
    goalpost_y = kickDirection;

    double penalty_angle = atan2(fabs(goalpost_y - koor_bola_y), goalpost_x - koor_bola_x) * 180 / M_PI;
    // double penalty_angle = atan2(koor_bola_y - goalpost_y  , goalpost_x - koor_bola_x) * 180 / M_PI;
    return (koor_bola_y>=goalpost_y) ? -penalty_angle : penalty_angle;
    // return -30;
}
//

//penalti baru
/*
BISA JUGA BUAT ANGKA2 TERTENTU KEK GINI
case 1:
case 3:
case 9:
    goalpost_y = ..;
btw nilainya masih random
*/
double OlympicPenalty::titikTendangan(int RNG){
    double goalpost_y_prediksi;
    switch (RNG)
    {
    case 1:
        goalpost_y_prediksi = 260;
        break;
    case 2:
        goalpost_y_prediksi = 280;
        break;
    case 3:
        goalpost_y_prediksi = 300;
        break;
    case 4:
        goalpost_y_prediksi = 330;
        break;
    case 5:
        goalpost_y_prediksi = 360;
        break;
    return goalpost_y_prediksi;
}}
//



void OlympicPenalty::ballState(double orientation)
{

    switch (m_tracking_status)
    {
    case BallTracker::Found:

        if (!s_first_found)
        {
            s_first_found = true;
            // behavioral_.passerCheckFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, orientation);
            // behavioral_.passerCheckFollowing((behavioral_.angle_yaw - behavioral_.resetOffset), tracker_.ballPos_x, tracker_.ballPos_y, orientation);
            behavioral_.passerCheckFollowing(0, tracker_.ballPos_x, tracker_.ballPos_y, orientation);
            }

        // ROS_WARN("BALL STATE AAAAAAAAAAAAAAAAAAAAA");
        // behavioral_.passerFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, orientation);
            behavioral_.passerFollowing(0, tracker_.ballPos_x, tracker_.ballPos_y, orientation);
        behavioral_.searchState = Behavioral::LookAround;
        m_resetTime = true;
        break;

    case BallTracker::NotFound:

        if (m_resetTime)
            p_resetTimeStart = ros::Time::now().toSec();
        m_resetTime = false;
        resetTimeNow = ros::Time::now().toSec() - p_resetTimeStart;
        if (resetTimeNow > 5){
            s_first_found = false;
        }
        else
            break;

        // behavioral_.passerSearch(s_role);
        break;
    }  
  }


} // namespace robotis_op
