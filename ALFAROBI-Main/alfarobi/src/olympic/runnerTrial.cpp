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

    void OlympicRunner::setModuleEnable()
    {
        enable_ = true;
        startRunMode();
    }

    void OlympicRunner::setModuleDisable()
    {
        tracker_.stopTracking();
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

        tracker_.startTracking();

        while (ros::ok())
        {
            if (wait_count_ <= 0)
            {
                if (enable_ == true)
                {
                    // behavioral_.CALIBRATION = s_calibration;
                    // m_tracking_status = tracker_.processTracking(s_tracking_mode);
                    // if (m_tracking_status == BallTracker::DontTrack)
                    //     tracker_.startTracking();
                    // else
                    //{
                        m_stand_state = behavioral_.checkFallen();
                        m_button_status = behavioral_.buttonStatus;

                        if (m_stand_state == Behavioral::Nothing)
                        {
                            buttonStatus();
                            runnerState();
                        }
                        standState();
                   // }
                    //s_tracking_mode = behavioral_.trackingMode;
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
            strategyState();
            break;
        }
    }

    void OlympicRunner::standState()
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
            m_runner_state = Runner_Start;
            m_stand_state = Behavioral::Nothing;
            break;
        }
    }

    void OlympicRunner::strategyState()
    {
        // switch (m_facing_state)
        // {case faceUpPole:
        // switch (m_tracking_status)
        // {
            // case BallTracker::Found://GANTI JADI PillarTracker

            // behavioral_.mileage_xDist = 0;
            // behavioral_.mileage_yDist = 0;
            // if (!s_first_found)
            // {
            //     s_first_found = true;
                behavioral_.faceUpPole(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta);
                // behavioral_.followingRunner(int m_facing_state,double robot_theta, double ball_x, double ball_y, double ball_theta);
            //     if(finish_turn){
            //         m_facing_state = FacingStatus::faceUpSquare;//NEW, BUTUH KEADAAN YANG MENANDAKAN SUDAH SELESAI BERPUTAR
            //     }
            // }
            // m_resetTime = true;
            // break;

        // case BallTracker::NotFound:
        //     std::cout << "X: " << tracker_.ballPos_x << " Y: " << tracker_.ballPos_y << std::endl;
        //     if (m_resetTime)
        //         p_resetTimeStart = ros::Time::now().toSec();
        //     m_resetTime = false;
        //     resetTimeNow = ros::Time::now().toSec() - p_resetTimeStart;
        //     if (resetTimeNow > 2)
        //         s_first_found = false;
        //     else
        //         break;

        //     behavioral_.searchPillar(tracker_.robotState_x, tracker_.robotState_y, tracker_.robotState_theta,p_stupid_localization_border);
        //     break;
        //     }
        // break;
        // case faceUpSquare:
        // switch (m_tracking_status) 
        // {
        //     case BallTracker::Found: // GANTI JADI SquareTracker

        //     behavioral_.mileage_xDist = 0;
        //     behavioral_.mileage_yDist = 0;
        //     if (!s_first_found)
        //     {
        //         s_first_found = true;
        //         behavioral_.faceUpSquare(tracker_.robotState_theta, tracker_.ballPos_x, tracker_.ballPos_y, tracker_.ballPos_theta);
        //         // behavioral_.followingRunner(int m_facing_state,double robot_theta, double ball_x, double ball_y, double ball_theta);
        //     }
        //     m_resetTime = true;
        //     break;

        // case BallTracker::NotFound:
        //     std::cout << "X: " << tracker_.ballPos_x << " Y: " << tracker_.ballPos_y << std::endl;
        //     if (m_resetTime)
        //         p_resetTimeStart = ros::Time::now().toSec();
        //     m_resetTime = false;
        //     resetTimeNow = ros::Time::now().toSec() - p_resetTimeStart;
        //     if (resetTimeNow > 2)
        //         s_first_found = false;
        //     else
        //         break;

        //     behavioral_.searchPillar(tracker_.robotState_x, tracker_.robotState_y, tracker_.robotState_theta,p_stupid_localization_border);
        //     break;
        //     }
        // break;
        } 
    }
}