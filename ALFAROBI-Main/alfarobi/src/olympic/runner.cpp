#include "alfarobi/runner.h"

namespace robotis_op
{
    OlympicRunner::OlympicRunner()
        : SPIN_RATE(30),
          wait_count_(0),
          nh_(ros::this_node::getName()),
          p_stupid_localization_border(150)
    {
        enable_ = false;
        nh_.param<bool>("calibration", s_calibration, false);
        boost::thread queue_thread_ = boost::thread(boost::bind(&OlympicRunner::queueThread, this));
        boost::thread process_thread = boost::thread(boost::bind(&OlympicRunner::processThread, this));
        revo_succes = false; //New
    }

    OlympicRunner::~OlympicRunner()
    {
    }

    void OlympicRunner::queueThread()
    {
            demo_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &OlympicRunner::CommandCallback, this);

        ros::Rate rate(30);

        while (nh_.ok())
        {
            ros::spinOnce();

            rate.sleep();
        }
    }

    void OlympicRunner::CommandCallback(const std_msgs::String::ConstPtr &msg) //TOPICNYA DARI MANA BELUM TAU
    {
    if (enable_ == false)
        return;

    }

    void OlympicRunner::setModuleEnable()
    {
        enable_ = true;
        startRunMode();
    }

    void OlympicRunner::setModuleDisable()
    {
        tracker_pillar_.stopTracking();
        behavioral_.stopFollowing();

        enable_ = false;

        behavioral_.setModuleTo("base_module");
    }

    void OlympicRunner::startRunMode()
    {
        ROS_INFO("Start Runner Demo");

        enable_ = true;

        m_runner_state = Runner_Start;
    }

    void OlympicRunner::stopRunMode()
    {
        ROS_INFO("Stop Runner Demo");

        enable_ = false;

        m_runner_state = Runner_Stop;
    }

    void OlympicRunner::processThread()
    {
        ros::Rate loop_rate(SPIN_RATE);
        
        tracker_pillar_.startTracking();
        // s_tracking_mode = PillarTracker::Square;
        s_tracking_mode = PillarTracker::Nod;
        status = GoPillar;
        m_facing_state = FacingStatus::faceUpPole;
        behavioral_.runnerPillarState = behavioral_.Runner_Pillar_State::Runner_Pillar_Forward;
        behavioral_.runnerAwayState = behavioral_.Runner_Away_State::Runner_Away_Forward;
        behavioral_.runnerBackState = behavioral_.Runner_Back_State::Runner_Back_Forward;

        while (ros::ok())
        {
            if (wait_count_ <= 0)
            {
                if (enable_ == true)
                {
                    behavioral_.CALIBRATION = s_calibration;
                    if(firstTrue){
                        timeStart = ros::Time::now().toSec();
                        firstTrue = false;
                     }
        
                    double timeBeforeTrack;
                    timeBeforeTrack = ros::Time::now().toSec() - timeStart;

                    if(timeBeforeTrack<3)
                    {
                        // std ::cout <<"Haven't Tracking yet"<< std::endl;
                        tracker_pillar_.setUsingHeadScan(false);
                         
                    }
                    else 
                    {
                        tracker_pillar_.setUsingHeadScan(true);
                        tracker_pillar_.startTracking();
                        m_tracking_status = tracker_pillar_.processTracking(s_tracking_mode);
                    }
                    
                    tracker_pillar_.startTracking();
                    m_tracking_status = tracker_pillar_.processTracking(s_tracking_mode);                
                    // m_stand_state = behavioral_.checkFallen();
                    m_button_status = behavioral_.buttonStatus;

                    if (m_stand_state == Behavioral::Nothing)
                    {
                        buttonStatus();
                        runnerState();
                    }
                    standState();
                
                }
            }
            else
                wait_count_ -= 1;

            loop_rate.sleep();
        }
    }

    void OlympicRunner::buttonStatus()
    {
        switch (m_button_status)
        {
            // case Alfarobi::L1:
            // break;
            // case Alfarobi::L2:
            // break;
            // case Alfarobi::L3:
            //     s_calibration = false;
            //     m_runner_state = Runner_Start;
            //     behavioral_.startFollowing();
            // break;
            // case Alfarobi::L4:
            //     behavioral_.stopFollowing();
            //     timeStart = ros::Time::now().toSec();
            // break;
            // case Alfarobi::R1:
                
            // break;
            // case Alfarobi::R2:
            // break;
            // case Alfarobi::R3:
            // break;
            // case Alfarobi::R4:
            // break;
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

    void OlympicRunner::runnerState()
    {
        switch (m_runner_state)
        {
        case Runner_Start:
            m_stand_state = Behavioral::Stand;
            m_runner_state = Runner_Run;
            wait_count_ = 1 * SPIN_RATE;
            break;

        case Runner_Run:
            // std::cout<<"Runner Run State"<<std::endl;
            strategyState();

            break;
        }
    }

    void OlympicRunner::standState()
    {
        switch (m_stand_state)
        {
        case Behavioral::Stand:
            // std::cout << "STAND" << std::endl;
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
            // std::cout << "FALLEN" << std::endl;
            behavioral_.stopFollowing();
            behavioral_.fallen(m_stand_state);
            while (behavioral_.isActionRunning() == true)
                ros::Duration(0.05).sleep();
            m_runner_state = Runner_Start;
            m_stand_state = Behavioral::Nothing;
            break;
        }
    }

    void OlympicRunner::strategyState()
    {
        if(firstTrue){
                        timeStart = ros::Time::now().toSec();
                        firstTrue = false;
                     }
        
        double timeForForward;
        bool statusFaceUpHome =false;
        timeForForward = ros::Time::now().toSec() - timeStart;
        // std::cout<<"waktu  "<<timeForForward<<std::endl;

        if(timeForForward<35)
        {
            // std ::cout <<" ===== Majuuu Lurus  ======    waktu :   "<<timeForForward << std::endl;
            behavioral_.awayFromHome();
             
        }
        else
        {
            switch (m_facing_state)
            {
            
            case faceUpPole:
                switch(status)
                {
                    case GoPillar:
                    
                        switch (m_tracking_status)
                        {
                            case PillarTracker::Found://GANTI JADI PillarTracker
                            // std::cout<<"FOUND !"<<std::endl;

                                s_first_found = true;
                                // std::cout << "===AYO MAJUUUUU KE PILLAR====" << std::endl;
                                statusFaceUpHome = behavioral_.goToPillar(tracker_pillar_.robotState_theta, tracker_pillar_.pillarPos_x, tracker_pillar_.pillarPos_y, tracker_pillar_.pillarPos_theta);
                                if(statusFaceUpHome == true)
                                {
                                    // behavioral_.stopFollowing();
                                     // std::cout<<"RUN TO PILLAR KELAR"<<std::endl;
                                    m_revo_status_ = firstRevo_;
                                    status = GoTurning;
                                }
                                // std::cout<<"false"<<std::endl;
                                m_resetTime = true;
                            break;


                            case PillarTracker::NotFound:

                                // std::cout<< "NOT FOUND"<<std::endl;
                                behavioral_.stupidSearchForRunner();
                            
                            break;
                        }
                    break;

                    case GoTurning:
                        revo_succes = behavioral_.RevoPillar_(tracker_pillar_.pillarPos_x);

                        if(revo_succes)
                        {
                            // std::cout<<"============ REVO DONEE ========="<<"           "<<revo_succes<<std::endl;
                            revo_succes = false;
                            // std::cout << " ===kelar==  " << std::endl;
                            // behavioral_.stopFollowing();
                            m_facing_state = faceUpSquare;
                        }

                    break;
                }
            break;

            case faceUpSquare:

                // std::cout << "GO TO FINISH" << std::endl;
                behavioral_.backToHome();

            break;
            }  
        }
    }
}

