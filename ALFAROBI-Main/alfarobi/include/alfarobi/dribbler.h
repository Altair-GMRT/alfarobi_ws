#ifndef DRIBBLER_H
#define DRIBBLER_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "alfarobi.h"
#include "behavioral.h"
#include "ball_tracker.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "strategycontroller.h"

namespace robotis_op
{
    class OlympicDribbler : public Alfarobi
    {

    public:
        OlympicDribbler();
        ~OlympicDribbler();

        void setModuleEnable();
        void setModuleDisable();

        enum DribblingStatus
        {
            Dribbler_Stop = 0,
            Dribbler_Start = 1,
            Dribbler_Run = 2,
            Dribbler_Waiting = 3,
        };

        enum MotionState{
            GOPILLAR = 0,
            GOTURN = 1,
            GOHOME = 2,
        };

        enum TrackState{
            Ball_Tracking = 0,
            Pillar_Tracking = 1,
            Home_Tracking = 2,
        };

        

    protected:
        const int SPIN_RATE;

        BallTracker tracker_;
        Behavioral behavioral_;
        Strategy strategy_;

        //ROS
        ros::NodeHandle nh_;
        int wait_count_;
        ros::Subscriber demo_command_sub_;

        void loadConfig(const std::string &path);
        void queueThread();
        void processThread();
        void CommandCallback(const std_msgs::String::ConstPtr &msg);

        void startDribbleMode();
        void stopDribbleMode();

        void buttonStatus();

        void dribblerState();
        void standState();
        void strategyState();

        void ballTrack();
        void pillarTrack();
        void homeTrack();

        bool s_first_found;
        bool s_calibration;
        int s_tracking_mode;

        int m_tracking_status;
        int m_button_status;
        int m_head_status;
        int m_stand_state;
        int m_strategy_state;
        int m_tracking_state;
        int m_dribbler_state;
        double m_resetTime;

        double p_resetTimeStart;

        double resetTimeNow;

        //anyar
        bool ballTrackStatus;
        bool closer;
        bool nearPillar;
        bool pillarTrackStatus;
        bool homeTrackStatus;
        bool align;
        int m_following_state;
        bool firstSearch;
        bool dribble;
        double p_stupid_localization_border;

    };
} // namespace robotis_op
#endif
