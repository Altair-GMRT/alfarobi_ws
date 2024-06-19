#include "alfarobi/dribbler.h"
#include "alfarobi/alfarobi.h"

namespace robotis_op
{
    OlympicDribbler::OlympicDribbler()
        : SPIN_RATE(30),
          wait_count_(0),
          nh_(ros::this_node::getName())
    {
        enable_ = false;
        firstSearch = false;
        closer = false;
        dribble = false;
        nh_.param<bool>("calibration", s_calibration, false);
        boost::thread queue_thread_ = boost::thread(boost::bind(&OlympicDribbler::queueThread, this));
        boost::thread process_thread = boost::thread(boost::bind(&OlympicDribbler::processThread, this));
    }

    OlympicDribbler::~OlympicDribbler()
    {
    }

    void OlympicDribbler::queueThread()
    {
        demo_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &OlympicDribbler::CommandCallback, this);

        ros::Rate rate(30);

        while (nh_.ok())
        {
            ros::spinOnce();

            rate.sleep();
        }
    }

    void OlympicDribbler::CommandCallback(const std_msgs::String::ConstPtr &msg) //TOPICNYA DARI MANA BELUM TAU
    {
        if (enable_ == false)
            return;

        // if (msg->data == "start")
        // {
        //     if (soccerState() == true)
        //         stopSoccerMode();
        //     else
        //         startSoccerMode();
        // }
        // else if (msg->data == "stop")
        // {
        //     stopSoccerMode();
        // }
    }

    void OlympicDribbler::setModuleEnable()
    {
        enable_ = true;
        startDribbleMode();
    }

    void OlympicDribbler::setModuleDisable()
    {
        tracker_.stopTracking();
        behavioral_.stopFollowing();

        enable_ = false;

        behavioral_.setModuleTo("base_module");
    }

    void OlympicDribbler::startDribbleMode()
    {
        ROS_INFO("Start Dribbler Demo");

        enable_ = true;

        m_dribbler_state = Dribbler_Start;
    }

    void OlympicDribbler::stopDribbleMode()
    {
        ROS_INFO("Stop Dribbler Demo");

        enable_ = false;

        m_dribbler_state = Dribbler_Stop;
    }

    void OlympicDribbler::processThread()
    {
        ros::Rate loop_rate(SPIN_RATE);

        tracker_.startTracking();
                    m_tracking_state = Ball_Tracking;
            m_strategy_state = GOPILLAR; 


        while (ros::ok())
        {
            if (wait_count_ <= 0)
            {
                if (enable_ == true)
                {
                    behavioral_.CALIBRATION = s_calibration;
                    m_tracking_status = tracker_.processTracking(s_tracking_mode);
                    if (m_tracking_status == BallTracker::DontTrack)
                        tracker_.startTracking();
                    else
                    {
                        m_stand_state = behavioral_.checkFallen();
                        m_button_status = behavioral_.buttonStatus;

                        if (m_stand_state == Behavioral::Nothing)
                        {
                            buttonStatus();
                            std::cout<<"======================MASOKKKKKKK==================="<<std::endl;
                                        std::cout<<"======================FIRSTTT_SEARCHHH=================== "<<firstSearch<<std::endl;

                            dribblerState();
                        }
                        standState();
                    }
                    s_tracking_mode = behavioral_.trackingMode;
                }
            }
            else
                wait_count_ -= 1;

            loop_rate.sleep();
        }
    }

    void OlympicDribbler::buttonStatus()
    {
        switch (m_button_status)
        {
        }
    }

    void OlympicDribbler::dribblerState()
    {
        // static bool firstt;
        switch (m_dribbler_state)
        {
        case Dribbler_Start:
            m_stand_state = Behavioral::Stand;
            m_dribbler_state = Dribbler_Run;
            wait_count_ = 1 * SPIN_RATE;
            break;

        case Dribbler_Run:
            std::cout<<"======================DRIBBLERRRUN==================="<<std::endl;
                                                    std::cout<<"======================FIRSTTT_SEARCHHH=================== "<<firstSearch<<std::endl;
            strategyState();
            // if(!firstt)
            // {
            //     firstt = true;
                // m_tracking_state = Ball_Tracking;
                // m_strategy_state = GOPILLAR;
            // }            // std::cout<<firstSearch<<"\t"<<std::endl;
            break;
        }
    }

    void OlympicDribbler::standState()
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
            behavioral_.startFollowing();
            m_stand_state = Behavioral::Nothing;
            break;

        case Behavioral::Nothing:
            break;

        default: // State 1 2 3 4
            std::cout << "FALLEN" << std::endl;
            behavioral_.stopFollowing();
            behavioral_.fallen(m_stand_state);
            while (behavioral_.isActionRunning() == true)
                ros::Duration(0.05).sleep();
            m_dribbler_state = Dribbler_Start;
            m_stand_state = Behavioral::Nothing;
            break;
        }
    }

    void OlympicDribbler::strategyState() //anyar
    {
        // static bool firstSearch_;
        if (!firstSearch)
        {
            std::cout<<"======================FIRSTTT_SEARCHHH==================="<<firstSearch<<std::endl;
            m_tracking_state = Ball_Tracking;
            m_strategy_state = GOPILLAR;      
        }

        switch (m_strategy_state)
        {
            case GOPILLAR:
                switch(m_tracking_state)
                {
                case Ball_Tracking:
                    // behavioral_.positionCheck = false;  
                    std::cout<<"======================DRIBLLEEEEE____BALL_TRACKING==================="<<std::endl;
                    ballTrack(); 
                    if(ballTrackStatus)
                    {
                        std::cout<<"======================DRIBLEEEEEE______FACEUPHOME==================="<<std::endl;
                        closer = behavioral_.faceUpHomeV2(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta);

                        if(closer)
                        {
                                                                    firstSearch = true;
                            std::cout<<"======================DRIBLEEE______closerrrrrrrr===================\t"<<std::endl;
                            m_tracking_state = Pillar_Tracking;
                            //TAMBAHAN 14-08-2020
                            // behavioral_.mileage_xDist = 0;
                            // dribble = behavioral_.dribble(4);
                            // if(dribble)
                            // {
                            //     // std::cout<<"======================DRIBBLE_DONE_DONE==================="<<std::endl;
                            //     // m_tracking_state = Ball_Tracking;
                            //     // firstSearch = true; //nanti di ilangin kalo uda pillar
                            //     dribble = false;
                            // }
                            // closer = false;                            
                        }
                    }

                    if(nearPillar)
                    {
                        // m_tracking_state = First_Revo;
                        m_strategy_state = GOTURN;
                    }
                    break;

                case Pillar_Tracking:
                    // pillarTrack();
                    // if(pillarTrackStatus)
                    // {
                        std::cout<<"======================PILLAR_TRACKING==================="<<std::endl;
                        if(!behavioral_.positionCheck(tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta))
                            m_tracking_state = Ball_Tracking;
                        else
                        {
                            align = behavioral_.alignToPillar(); //align = false (awal)
                            if(align && behavioral_.positionCheck(tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta) == true)
                            {
                                std::cout<<"======================DRIBLEEE______ALIGNNNN__DONEEEEE===================\t"<<std::endl;
                                dribble = behavioral_.dribble(4);
                                if(dribble)
                                {
                                    m_tracking_state = Ball_Tracking;
                                    firstSearch = false;
                                }
                            }

                        }


                    // }
                    // int x_pillar = 8;
                    //pakae pillar pos dlu
                    // if(tracker_.pillarPos_x < 10) //blm tau angle tolerance nya brp // pake jarak x bukan angle
                    //     nearPillar = true;
                    break;
                }
            break;
    //         case GOTURN:
        
    //            // switch(m_tracking_state)
    //            // {
    //                 //case First_Revo:
    //                 behavioral_.RevoPillar(tracker_.pillarPos_x);
    //                 if(behavioral_.RevoPillar(tracker_.pillarPos_x))
    //                 {
    //                     m_tracking_status = GOHOME;
    //                     m_tracking_state = Ball_Tracking;
    //                 }
    //                 else
    //                 {
    //                     m_tracking_state = Ball_Tracking;
    //                     m_tracking_status = GOPILLAR;
    //                 }
    //                 //break;
    //            // }
    //         break;
    //         case GOHOME:
                
    //             switch(m_tracking_state)
    //             {
    //             case Ball_Tracking:
    //                 ballTrack(); 
    //                 if(ballTrackStatus)
    //                 {
    //                     closer = behavioral_.faceUpHomeV2(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta);
    //                     if(closer)
    //                         m_tracking_state = Home_Tracking;
    //                 }
    //                 break;

    //             case Home_Tracking:
    //                 homeTrack();
    //                 if(homeTrackStatus)
    //                 {
    //                     if(!behavioral_.positionCheck(tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta))
    //                         m_tracking_state = Ball_Tracking;
    //                     else
    //                     {
    //                         align = behavioral_.alignToHome(tracker_.homePos_thetaHome_x, tracker_.homePos_thetaHome_y, tracker_.homePos_x, tracker_.homePos_y); //align = false (awal)
    //                         m_following_state = behavioral_.checkFollowing(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta).x();
    //                         behavioral_.following(m_following_state, tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta);
    //                         behavioral_.searchState = Behavioral::LookAround;
    //                         // kick = true;
    //                         m_tracking_state = Ball_Tracking;
    //                     }
                        

    //                     // if(kick)
    //                     // //jalan dlu baru cari bola
    //                     //     behavioral_.mileage_xDist = 0;
    //                     //     dribble = behavioral_.dribble(400); 
    //                     //     if(dribble)
    //                     //         m_tracking_state = Ball_Tracking;
    //                 }
    //                 break;
    //             }
        }
        
     }

    void OlympicDribbler::ballTrack()
    {
        switch (m_tracking_status)
        {
            case BallTracker::Found:
            std::cout<<"======================BALLLLLLLLLLLLFOUUUUUUNDDDDD==================="<<std::endl;
            ballTrackStatus = true;
            break;

            case BallTracker::NotFound:
            behavioral_.back_dribble(50);
            // behavioral_.setWalkingParam(-0.15, 0, 0);
            ballTrackStatus = false;
            behavioral_.stupidSearch(tracker_.robotState_x, tracker_.robotState_y, tracker_.robotState_theta, p_stupid_localization_border);
            break;
        }
    }

    void OlympicDribbler::pillarTrack()
    {
        // switch (m_tracking_status)  //nggawe m_tracking_status = tracker_.processTracking DEWEEEEE
        // {

        //     case PillarTracker::Found: 
        //     pillarTrackStatus = true;
        //     behavioral_.changeDirectionP = 0;
        //     break;

        //     case PillarTracker::NotFound:
        //     pillarTrackStatus = true;
        //     // pillarTrackStatus = false;
        //     //     behavioral_.searchPillar(); //rekursif nya?
        //     break;
        // }
    }

    void OlympicDribbler::homeTrack()
    {
    //     switch (m_tracking_status)  //nggawe m_tracking_status = tracker_.processTracking DEWEEEEE
    //     {
    //         case HomeTracker::Found:
    //         homeTrackStatus = true;
    //         behavioral_.changeDirectionH = 0;
    //         break;

    //         case HomeTracker::NotFound:
    //         pillarTrackStatus = true;
    //         // homeTrackStatus = false;
    //         //     behavioral_.searchHome();
    //         break;
    //     }
    }
} // namespace robotis_op
