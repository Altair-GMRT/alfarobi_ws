#include "alfarobi/passer.h"
#include "alfarobi/alfarobi.h"
#include <termios.h>

namespace robotis_op
{
OlympicPasser::OlympicPasser()
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
    // robot_penalty = false;
    nh_.param<bool>("calibration", s_calibration, false);
    nh_.param<bool>("role", s_role, false);
    m_robot_state = Robot_Start;
    // m_penalty_state = Penalty_Start;
    int awal = ros::Time::now().toSec();
    boost::thread queue_thread_ = boost::thread(boost::bind(&OlympicPasser::queueThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&OlympicPasser::processThread, this));
}

OlympicPasser::~OlympicPasser()
{
}

void OlympicPasser::queueThread()
{
    demo_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &OlympicPasser::CommandCallback, this);
    // pengTrue_pub_ = nh_.advertise<std_msgs::Bool>("/topic_sendiri", 1);
    // pengTrue_sub_ = nh_.subscribe("/topic_sendiri", 1, &OlympicPasser::pengTrueCallback, this);

    ros::Rate rate(30);

    while (nh_.ok())
    {
        ros::spinOnce();

        rate.sleep();
    }
}

// void OlympicPasser::pengTrueCallback(const std_msgs::Bool::ConstPtr &msg)
// {
//     behavioral_.friend_status = msg->data;
//}

// void OlympicPasser::pengTruePublish()
// {
//     if(!firstMasuk){
//         timeAwal = ros::Time::now().toSec();
//         firstMasuk = true;
//     }

//     timeTotal = ros::Time::now().toSec() - timeAwal;

//     std::cout << "\nTIME TOTAL = " << timeTotal << std::endl;

//     if(timeTotal > 2){
//         //std_msgs::Bool msg;
//         //msg.data = true;
//         //pengTrue_pub_.publish(msg);
//         behavioral_.friend_status = true;
//     }
//     // else
//     // {
//     //     std_msgs::Bool msg;
//     //     msg.data = false;
//     //     pengTrue_pub_.publish(msg);
//     // }
// }

void OlympicPasser::CommandCallback(const std_msgs::String::ConstPtr &msg) //TOPICNYA DARI MANA BELUM TAU
{
    if (enable_ == false)
        return;
}

void OlympicPasser::setModuleEnable()
{
    count = 0;
    kick = false;
    enable_ = true;
    startPassMode();
}

void OlympicPasser::setModuleDisable()
{
    tracker_.stopTracking();
    behavioral_.stopFollowing();

    enable_ = false;

    behavioral_.setModuleTo("base_module");
}

void OlympicPasser::startPassMode()
{
    ROS_INFO("Start Passer Demo");

    enable_ = true;

    m_passer_state = Passer_Start;
}

void OlympicPasser::stopPassMode()
{
    ROS_INFO("Stop Passer Demo");

    enable_ = false;

    m_passer_state = Passer_Stop;
}

void OlympicPasser::processThread()
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
                    m_tracking_status = tracker_.processTracking(s_tracking_mode);
                    break;

                // case Line:
                //     m_tracking_status = tracker_cat2_.processTracking(s_tracking_mode);
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
                    m_stand_state = behavioral_.checkFallen();
                    m_button_status = behavioral_.buttonStatus;
                    // if (check_fallen){
                    //     behavioral_.stopFollowing();
                    //     m_robot_state = Robot_Start;
                    //     check_fallen= false;
                    // }
                    ROS_WARN("checkFallen: %d", m_stand_state);

                    if (m_stand_state == Behavioral::Nothing)
                    {
                        buttonStatus();
                        
                        if (m_penalty_state == Penalty_Start) {
                            strategyPenaltyState();
                        }
                        else {
                            passerState();
                        }
                    }
                    // else{
                    //     behavioral_.stopFollowing();
                    //     m_robot_state = Robot_Start;
                    //     // check_fallen= false;
                    // }
                    standState();
                }
            }
        }
        else
            wait_count_ -= 1;

        loop_rate.sleep();
    }
}

void OlympicPasser::altairGamepad()
{
    char chjul = getchar();

    switch(chjul)
    {
        case 'w':
            behavioral_.setWalkingParam(0.145, 0.05, -0.1);  
            altairGamepad();
        
        case 'x':
            behavioral_.stopFollowing();
            altairGamepad();

    }

}

void OlympicPasser::buttonStatus()
{
    std_msgs::String msgjul;
    switch (m_button_status)
    {
    // case Alfarobi::L1: //start robi
    //     // // behavioral_.send();
    //     // // behavioral_.friend_status = true;
    //     m_robot_state = Robot_Waiting;
    //     // robot_penalty = true;
    //     // behavioral_.firstIn = false;
    //     // kick = true;
    //     // behavioral_.firstKick = false;
    //     // // behavioral_.stopFollowing();
    //     break;
    // // case Alfarobi::L2: //jangan dipake
    // //     // s_calibration = false;
    // //     // behavioral_.startFollowing();
    // //     // m_passer_state = Passer_Run;
    // //     break;
    // // case Alfarobi::L3: //jangan dipake
    // //     // s_calibration = true;
    // //     // // if(!s_role && count > 1)
    // //     // behavioral_.stopFollowing();
    // //     // count = 5;
    // //     break;
    // case Alfarobi::L4: //reset
    //     behavioral_.startFollowing();
    //     m_robot_state = Robot_Positioning;
    //     behavioral_.lastKick = false;
    //     m_positioning = false;
    //     count = 0;
    //     behavioral_.CALIBRATION = false;
    //     firstIn = false;
    //     behavioral_.firstIn = false;
    //     // behavioral_.resetComm();
    //     // behavioral_.resetFSM();
    //     // behavioral_.m_pass_state = Pass_Forward;
    //     // behavioral_.stopFollowing();
    //     behavioral_.friend_status = false;
    //     break;
    // case Alfarobi::R1:
    //     // behavioral_.stopFollowing();
    //     m_robot_state = Robot_Start;
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
    //     s_pickup = true;
    //     behavioral_.mileage_xDist = 0;
    //     behavioral_.mileage_yDist = 0;
    //     behavioral_.startFollowing();
    //     m_robot_state = Robot_Positioning;
    //     m_positioning = false;
    //     //behavioral_.stopFollowing();
    //     // behavioral_.friend_status = false;
    //     // behavioral_.bo_ol = true;
    //     break;
    // case Alfarobi::R4: //penalty mode
    //     // robot_penalty = true;
    //     // s_role = false;
    //    // m_robot_state = Robot_Kicking;
    //     // behavioral_.startFollowing();
    //     // kick = false;
    //     //JANGAN DIKASI APA2, BUAT IMU GACHA -KEITOT
    //     break;
        case Alfarobi::B1:
            ROS_INFO("BUTTON B1");
            behavioral_.resetOffset = behavioral_.m_yaw; 
            // behavioral_.setBodyModuleTo("quintic_walk", false);

            break;
        case Alfarobi::B2:
            ROS_INFO("BUTTON B2");
            behavioral_.stopFollowing();
            // m_robot_state = Robot_Waiting;
            char chjul;
            // altairGamepad();

            // // getWalkingParam();
            // std_msgs::String _command_msg;
            // _command_msg.data = "start";
            // set_walking_command_pub_.publish(_command_msg);
            
            // JUL BREAKING BALLSSS
            while (ros::ok)
            {
                // if(m_tracking_status == BallTracker::Found){
                ROS_INFO("BALL FOUND");
                chjul = getchar();
                switch (chjul){ 
                    case 'w':
                    ROS_INFO("MAJUUUUUUUUUUUU");
                    behavioral_.startFollowing();
                    behavioral_.setWalkingParam(0.4, 0.0, 0.0 );
                    // behavioral_.setWalkingParam(0.100, 0.100, -0.40 );
                    // behavioral_.setWalkingParam(0.100, 0.0, 0.0 );
                    // behavioral_.setWalkingParam(0.100, 0.00, -0.100 );
                    break;

                    case 's':
                    ROS_INFO("FORWARDDDDD MINNNNNNN");
                    behavioral_.startFollowing();
                    behavioral_.setWalkingParam(0.09, 0.0, 0.0 );
                    // behavioral_.setWalkingParam(0.100, 0.100, -0.40 );
                    // behavioral_.setWalkingParam(0.100, 0.00, -0.100 );
                    break;

                    case 'a':
                    ROS_INFO("SHIFTT KIRIIIIIIIIIIII");
                    behavioral_.startFollowing();
                    behavioral_.setWalkingParam(0.0, 0.160, 1.0);
                    // behavioral_.setWalkingParam(-0.03, 0.18, 0.4);
                    // behavioral_.setWalkingParam(-0.05, 0.15, 0.5);
                    break;

                    case 'd':
                    ROS_INFO("SHIFTT KANANNNNNN");
                    behavioral_.startFollowing();
                    behavioral_.setWalkingParam(-0.050, -0.200, -0.500);
                    // behavioral_.setWalkingParam(-0.05, -0.12, -0.35);
                    // behavioral_.setWalkingParam(-0.08, -0.23, -0.3);
                    break;

                    case 'q':
                    ROS_INFO("REVO KIRIIIIII");
                    behavioral_.startFollowing();
                    // behavioral_.setWalkingParam(-0.02, 0.016, -0.2);
                    behavioral_.setWalkingParam(0.0, 0.1, 0.0);
                    break;

                    case 'e':
                    ROS_INFO("REVO KANANNNNNN");
                    behavioral_.startFollowing();
                    behavioral_.setWalkingParam(0.0, -0.180, 0.300);
                    // behavioral_.setWalkingParam(0.0, -0.19, 0.6);
                    break;

                    case 't':
                    ROS_INFO("ROTATEEEE CW");
                    behavioral_.startFollowing();
                    behavioral_.setWalkingParam(-0.03, 0.050, -0.6);
                    break;

                    case 'r':
                    ROS_INFO("ROTATEEE CCW");
                    behavioral_.startFollowing();
                    // behavioral_.setWalkingParam(-0.03, 0.050, -0.6);
                    behavioral_.setWalkingParam(0.01, -0.050, 0.6);
                    break;

                    case 'k':
                    ROS_INFO("KIIIIIIIIIIIIICKKKKKKKKKKKKKKKKK");
                    behavioral_.stopFollowing();
                    behavioral_.setBodyModuleTo("kicking", false);
                    behavioral_.startKicking("left_kick");
                    break;

                    case 'x':
                    ROS_INFO("STOOOOOOOOOOOOOOOOOPPPPPPPPPPP");
                    behavioral_.stopFollowing();
                    break;

                    case 'o':
                    ROS_INFO("QUINTICCCCCCCCCCCC");
                    behavioral_.setBodyModuleTo("quintic_walk", false);
                    break;

                    
                }
                // }
            }
            break;
        case Alfarobi::B3: //start
            ROS_INFO("BUTTON B3");
         
            ros::Duration(2).sleep();
            

            behavioral_.timeAwal = ros::Time::now().toSec();
            behavioral_.counterAction = 0;

            behavioral_.firstKick = false;
            behavioral_.stopFollowing();
            behavioral_.send();
            behavioral_.alfaStartPass();
            behavioral_.friend_status=true;
            m_robot_state = Robot_Waiting;

            
            // behavioral_.forward(10);
            // startFollowing

            

            break;
        case Alfarobi::B4: //reset
            ROS_INFO("BUTTON B4");
            // behavioral_.startFollowing();
            m_robot_state = Robot_Positioning;
            behavioral_.lastKick = false;
            m_positioning = false;
            count = 0;
            behavioral_.CALIBRATION = false;
            firstIn = false;
            behavioral_.firstIn = false;
            // behavioral_.resetComm();
            // behavioral_.resetFSM();
            // behavioral_.m_pass_state = Pass_Forward;
            // behavioral_.stopFollowing();
            behavioral_.friend_status = false;

            // Penalty State
            m_penalty_state = Penalty_Start;

            break;
        default:
            ROS_INFO("BUTTON NONE");
    }
}

void OlympicPasser::passerState()
{
    switch (m_passer_state)
    {
    case Passer_Start:
        // ROS_INFO("START");

        m_stand_state = Behavioral::Stand;
        m_passer_state = Passer_Run;
        wait_count_ = 1 * SPIN_RATE;
        break;

    case Passer_Run:
        // ROS_INFO("RUN");
        
        
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
                m_robot_state = Robot_Waiting;
                ros::Duration(3).sleep();
                behavioral_.setBodyModuleTo("quintic_walk", true);
                behavioral_.passerCheckFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, kick_angle());
                s_tracking_mode = BallTracker::Square;
                m_tracking_status = tracker_.processTracking(s_tracking_mode);
                // behavioral_.startFollowing();
                behavioral_.kickingStatus = Behavioral::Not_Kicking;
                behavioral_.finishedKick = false;
                behavioral_.kickingDelay = false;

                //untuk strategi parkir bus
                 // s_tracking_mode = BallTracker::Ndungkluk;

                // s_tracking_mode = BallTracker::DontTrack;
                // if(behavioral_.rotate(180)){
                //     if(tracker_cat2_.linePos_x != -1 && tracker_cat2_.linePos_y != -1){
                //         if(tracker_cat2_.linePos_y >= 300){
                //             line_trigger = true;
                //         }
                //     }
                // }
                // if(line_trigger){ //blm selese
                //     if(behavioral_.rotate(0)){
                //         line_trigger = false;
                //         behavioral_.finishedKick = false;
                //         behavioral_.kickingDelay = false;
                //     }
                // }
            // }
            // else
            // {
            //     behavioral_.stopFollowing();
            //     // m_penalty_state = Penalty_Start;
            //     ros::Duration(3).sleep();
            //     behavioral_.setBodyModuleTo("quintic_walk", true);
            //     behavioral_.kickingStatus = Behavioral::Not_Kicking;
            //     behavioral_.finishedKick = false;
            //     behavioral_.kickingDelay = false;
            // }
            
        }
        if (s_role)
        strategyStateA();
        else
            strategyStateB();

        //penalti baru
        // if(s_role)
            // strategyStateA();
        // else
        //     strategyPenaltyState();


        break;
    }
}



void OlympicPasser::standState()
{
    switch (m_stand_state)
    {
    case Behavioral::Stand:
        #ifdef STATE
        std::cout << "STAND" << std::endl;
        #endif
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
        ROS_INFO("ROBOT GA JATUH");
        break;

    default: // State 1 2 3 4
        #ifdef STATE
        std::cout << "FALLEN" << std::endl;
        #endif
        behavioral_.stopFollowing();
        // di-command untuk regional (gapake jatuh bangun)
        // behavioral_.fallen(m_stand_state); 
        // while (behavioral_.isActionRunning() == true)
        //     ros::Duration(0.05).sleep();
        m_passer_state = Passer_Start;
        m_stand_state = Behavioral::Nothing;
        break;
    }
}

void OlympicPasser::strategyStateA()
{   
    static bool stableWalk2kick;
    #ifdef STATE
    // ballFilter(tracker_.ballPos_x,tracker_.ballPos_y, ball_x_filter, ball_y_filter);
    std::cout<< "ODOM X: "<< behavioral_.odometry_.x<<std::endl;
    std::cout<< "ODOM Y: "<< behavioral_.odometry_.y<<std::endl;
    // std::cout<< "BALL X: "<< ball_x_filter <<std::endl;
    // std::cout<< "BALL Y: "<< ball_y_filter <<std::endl;
    std::cout << "KICK ANGLE : " << kick_angle() << std::endl;
    #endif
    switch (m_robot_state)
    {
    case Robot_Start:
        #ifdef STATE
        ROS_WARN("DO NOTHING =================>");
        #endif
        m_stand_state = Behavioral::Nothing;
        behavioral_.stopFollowing();
        firstIn = false;
        #ifdef STATE
        std::cout<<"PICKUP ["<<s_pickup<<"]\n";
        #endif
        // s_tracking_mode = BallTracker::Square; //edit
        s_tracking_mode = BallTracker::Ndungkluk;
        behavioral_.firstIn = false;

        /* ===untuk rekaman video, gapake button untuk start=== */
        // ros::Duration(20).sleep();
        // m_robot_state = Robot_Waiting;
        // behavioral_.resetOffset = behavioral_.m_yaw;
        break;
    case Robot_Waiting:
        ROS_WARN("WAITING=================>");

        ROS_INFO("<= = = => %f", behavioral_.m_yaw);
        // ROS_INFO("<= = = => resetTimenow : %f", resetTimeNow);
        // double TOTAL_THETA = angle_yaw - resetOffset; // fariz waiting
        // ROS_INFO("TOTAL THETA[%f]", behavioral_.s_angle_yaw); // fariz waiting
        if(m_tracking_status == BallTracker::Found){
            ROS_INFO("BALL FOUND");
        
            if (behavioral_.friend_status){
                ROS_INFO("MULAI JALAN");
                behavioral_.startFollowing();
                behavioral_.lastKick = false;
                // stableWalk2kick = behavioral_.stableWalk(); 
                if(behavioral_.stableWalk())
                {
                    m_robot_state = Robot_Kicking;
                    behavioral_.firstIn = false;
                    firstIn = true;
                }
            } else {
                ROS_ERROR("Friend status belum true");
            behavioral_.stopFollowing();
            }
            // behavioral_.startFollowing();

            // if(behavioral_.forward(5)) {
            //     m_robot_state = Robot_Kicking;
            //     firstIn = true;
            // }
        }

        /* ===ini gapake vision===
        behavioral_.startFollowing();
        if(behavioral_.stableWalk() == 1) {
            ROS_ERROR("HARUSNYA UDAH KICKING");
            // behavioral_.firstIn = false;
            m_robot_state = Robot_Kicking; 
        } */

        /* ===ini langsung ke kicking===
        behavioral_.firstIn = false;
        m_robot_state = Robot_Kicking; */
        
       
        // else if(m_tracking_status == BallTracker::NotFound)
        // {
        //     if (m_resetTime)
        //         p_resetTimeStart = ros::Time::now().toSec();
        //     m_resetTime = false;
        //     resetTimeNow = ros::Time::now().toSec() - p_resetTimeStart;
        //     if (resetTimeNow > 7){
        //         s_first_found = false;
        //         behavioral_.startFollowing();
        //         behavioral_.rotate(340);
        //         // m_robot_state = Robot_Waiting;
        //     }
        //     else
        //         break;
        // }

        // else{
        //     // untuk video
        //     behavioral_.stopFollowing();
        //     // behavioral_.abiWaiting();
        // }

        s_tracking_mode = BallTracker::Square; //edit

        // commServer.init();
        // if(commServer.receiveMsg()){
        //     behavioral_.friend_status = true;
        //     ROS_WARN("ALFAAAA SUDAAAAH PASSSSINGGGGG");
        // }
        
        break;

    case Robot_Positioning:
        // s_tracking_mode = BallTracker::Square;
        // if (tracker_.ballPos_x <150 && fabs(tracker_.ballPos_y) < 100){
        //     behavioral_.startFollowing();
        //     if(behavioral_.stableWalk())
        //     {
        //         m_robot_state = Robot_Kicking;
        //         behavioral_.firstIn = false;
        //         firstIn = true;
        //     }
        // }
        if(tracker_.ballPos_x <150 && fabs(tracker_.ballPos_y) < 300/*m_tracking_status == BallTracker::Found){
            m_robot_state = Robot_Kicking;
            firstIn = true;
        }
        else
        {   
            #ifdef STATE
            ROS_WARN("POSITIONING=================>");
            std::cout<<"PICKUP POSITIONING ["<<s_pickup<<"]\n";
            #endif
            if(s_pickup)
            {   
                #ifdef STATE
                std::cout<<"POSITIONINGGGGGGGGGGGGGGG PICKUPPP\tMILEAGE : "<<behavioral_.mileage_xDist<<std::endl;
                // if((behavioral_.kickingDelay==true && behavioral_.kickTimeNow > behavioral_.KICK_DELAY) || behavioral_.walkingStatus == false)
                    // behavioral_.startFollowing();
                // if(!firstIn)
                // {
                //     m_positioning = behavioral_.forward(10);
                //     firstIn = true;
                // }
                #endif
                m_positioning = behavioral_.forward(50); //10
                // m_positioning = behavioral_.timePositioning();
                if(m_positioning/*behaviora9l_.forward(10))
                {
                    // behavioral_.stopFollowing();
                    // m_rotating = behavioral_.rotate(0);
                    if(behavioral_.rotate(0))
                    {
                        behavioral_.stopFollowing();
                        s_pickup = false;
                        behavioral_.mileage_xDist = 0;
                        behavioral_.mileage_yDist = 0;
                        m_rotating = false;
                        m_positioning = false;
                        m_robot_state = Robot_Waiting;
                        behavioral_.firstIn = false;
                    }
                }
            }
            else
            {   
                // #ifdef STATE
                std::cout<<"POSITIONINGGGGGGGGGG BIASAAAA" << std::endl;
                // #endif
                m_positioning = behavioral_.timePositioning();
                if(m_positioning/*behavioral_.timePositioning()*/)
                {   
                    firstIn = true;
                    behavioral_.firstIn = false;
                    m_positioning = false;
                    m_robot_state = Robot_Waiting; //edit

                }
            // }
        // }
        // positioning_finished = behavioral_.timePositioning();
        // if(positioning_finished)
        // {
        //     firstIn = true;
        //     m_robot_state = Robot_Kicking;
        // }
        break;
 

    case Robot_Kicking:
        s_tracking_mode = BallTracker::Square;
        
        ROS_WARN("KICKING====================>");
        
        if (firstIn)
            behavioral_.startFollowing();
        behavioral_.firstIn = false;
        firstIn = false;
        behavioral_.friend_status = false;
        ballState(0);
        // behavioral_.forward(5);
        // if(m_tracking_status == BallTracker::Found)
        // {
        //     ballState(0);
        // }        
        // else {
        //     behavioral_.stopFollowing();
        //     behavioral_.lastKick = false;
        //     // m_robot_state = Robot_Waiting;
        //     }

        break;
    }
}

void OlympicPasser::strategyStateB()
{
    switch (m_robot_state)
    {
    case Robot_Positioning:
        ROS_WARN("POSITIONING=================>");

        s_camera_track = Ball;

        // if (behavioral_.friend_status)
        //     m_robot_state = Robot_Waiting;
        // else
        //     behavioral_.stopFollowing();
        behavioral_.stopFollowing();

        break;

    case Robot_Waiting:
        ROS_WARN("WAITING====================>");

        s_camera_track = Ball;

        // if (behavioral_.timePosballStatitioning())
        // {
            // if(true)
            if (behavioral_.friend_status)
                m_robot_state = Robot_Kicking;
            else
                behavioral_.stopFollowing();
        // }
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

        // ballState(ball_to_friend());

        if (count == 2)
            behavioral_.stopFollowing();
        break;
    }
}

//pinalti baru
//blum tambahin button
void OlympicPasser::strategyPenaltyState()
{
    static int goalpost_y_input;
    std::cout<<"HASIL PREDIKSi ["<<RNG<<"]\n";
    switch(m_penalty_state)
    {
        case Penalty_Start:
            ROS_WARN("<======START PENALTY=======>");
            m_stand_state = Behavioral::Nothing;
            m_tracking_status = BallTracker::Square;
            s_camera_track = Ball ; // ke default, kamera dimatiin
            std::srand(time(0)); //biar hasil random ga sama terus
            RNG = (std::rand() % 5) + 1; //CARI CARA LEBIH EFEKTIF BUAT ANGKA ACAK
            if (robot_penalty)
            {
                m_penalty_state = Penalty_Running;
                robot_penalty = false;
            }
            break;

        case Penalty_Running:
            ROS_WARN("<======PENALTY RUNNING=======>");
            goalpost_y_input = titikTendangan(RNG);
            behavioral_.startFollowing();
            ballState(penalty_angle(goalpost_y_input));
            break;

        case Penalty_Stop:
            ROS_WARN("<=======PENALTY STOP======>");
            behavioral_.stopFollowing();
            break;
    }
}

//penalti

double OlympicPasser::penalty_angle(double kickDirection){
    double koor_bola_x = behavioral_.odometry_.x;// + (tracker_.ballPos_x * cos(behavioral_.s_angle_yaw) + tracker_.ballPos_y * sin(behavioral_.s_angle_yaw));
    double koor_bola_y = behavioral_.odometry_.y;// + (-tracker_.ballPos_x * sin(behavioral_.s_angle_yaw) + tracker_.ballPos_y * cos(behavioral_.s_angle_yaw));

    double goalpost_x = 375 /*875*/, goalpost_y = kickDirection; //y=150,320, 20

    double penalty_angle = atan2(fabs(goalpost_y - koor_bola_y), goalpost_x - koor_bola_x) * 180 / M_PI;
    // double kick_angle = atan2(koor_bola_y - goalpost_y  , goalpost_x - koor_bola_x) * 180 / M_PI;
    return (koor_bola_y>=goalpost_y) ? - penalty_angle : penalty_angle;
    // return -30;
}

//penalti
double  OlympicPasser::titikTendangan(int RNG){
    double goalpost_y_prediksi;
    switch(RNG)
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
            goalpost_y_prediksi = 320;
            break;
        case 5:
            goalpost_y_prediksi = 340;
            break;
        return goalpost_y_prediksi;   
    }
}

double OlympicPasser::kick_angle(){
    double koor_bola_x = behavioral_.odometry_.x;// + (tracker_.ballPos_x * cos(behavioral_.s_angle_yaw) + tracker_.ballPos_y * sin(behavioral_.s_angle_yaw));
    double koor_bola_y = behavioral_.odometry_.y;// + (-tracker_.ballPos_x * sin(behavioral_.s_angle_yaw) + tracker_.ballPos_y * cos(behavioral_.s_angle_yaw));

    double goalpost_x = 875 /*875*/, goalpost_y = 320; //y=150,320, 20

    double kick_angle = atan2(fabs(goalpost_y - koor_bola_y), goalpost_x - koor_bola_x) * 180 / M_PI;
    // double kick_angle = atan2(koor_bola_y - goalpost_y  , goalpost_x - koor_bola_x) * 180 / M_PI;
    return (koor_bola_y>=goalpost_y) ? -kick_angle : kick_angle;
    // return -30;
}

double OlympicPasser::ball_to_friend(){
    double koor_bola_x = behavioral_.odometry_.x;// + (tracker_.ballPos_x * cos(behavioral_.s_angle_yaw) + tracker_.ballPos_y * sin(behavioral_.s_angle_yaw));
    double koor_bola_y = behavioral_.odometry_.y;// + (-tracker_.ballPos_x * sin(behavioral_.s_angle_yaw) + tracker_.ballPos_y * cos(behavioral_.s_angle_yaw))
    double ball_target_x = 575, ball_target_y = 160;

    double ball_target = atan2(ball_target_y - koor_bola_y, ball_target_x - koor_bola_x) * 180 / M_PI;
    return ball_target;
}


void OlympicPasser::ballState(double orientation)
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
            // behavioral_.rotate(360);
            // m_robot_state = Robot_Waiting;
        }
        else
            break;
        // m_robot_state = Robot_Waiting;
        // behavioral_.passerSearch(s_role);
        break;
    }
}

// void OlympicPasser::ballFilter(double ballPos_x, double ballPos_y, double ball_x_filter, double ball_y_filter){
    
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

bool OlympicPasser::goalState()
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
