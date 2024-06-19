#ifndef SOCCER_H
#define SOCCER_H

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

    class SoccerDemo : public Alfarobi
    {
    public:
        SoccerDemo();
        ~SoccerDemo();

        void setModuleEnable();
        void setModuleDisable();

    protected:
        enum SoccerStatus
        {
            Soccer_Stop = 0,
            Soccer_Start = 1,
            Soccer_Running = 2,
            Soccer_Waiting = 3,
        };

        enum GameStatus
        {
            Game_Initial = 0,
            Game_Ready = 1,
            Game_Set = 2,
            Game_Play = 3,
            Game_Finish = 4
        };

        enum SecondaryStatus
        {
            State_Normal = 0,
            State_PenaltyShoot = 1,
            State_Overtime = 2,
            State_Timeout = 3,
            State_Freekick = 4,
            State_PenaltyKick = 5,
        };

        const int SPIN_RATE;
        const bool DEBUG_PRINT;

        void loadConfig(const std::string &path);
        void queueThread();
        void processThread();
        void CommandCallback(const std_msgs::String::ConstPtr &msg);

        void startSoccerMode();
        void stopSoccerMode();

        void buttonStatus();
        void communicationState();
        bool soccerState();
        void standState();
        void strategyState();

        // Team communication utils
        team_communication::robot_data robot_info;
        void robotCommunication();

        BallTracker tracker_;
        Behavioral behavioral_;
        Strategy strategy_;

        //ROS
        ros::NodeHandle nh_;
        ros::Subscriber demo_command_sub_;

        int wait_count_;

        //S = STRATEGY
        //State
        int s_soccer_half;
        bool s_calibration;
        bool s_communication;
        int s_tracking_mode;
        bool s_ball_prediction;
        bool s_waiting_start;
        bool s_first_found;
        bool s_pickup;
        bool s_positioning;
        //Communication
        bool s_start;
        bool s_stop;
        bool s_waiting;
        bool s_first_play;
        bool s_allowed_to_move;
        bool s_localization;
        //Positioning
        int s_first_zone;
        int s_destination_zone;
        int s_robot_zone;

        //M = MONITORING
        //State
        int m_soccer_state;
        int m_strategy_state;
        int m_stand_state;
        int m_following_state;
        int m_game_state;
        int m_game_time;
        int m_score_state;
        double m_positioning_start;
        //Status
        int m_robot_status;
        int m_tracking_status;
        int m_button_status;
        bool m_resetTime;
        //Positioning
        bool m_search_status;
        int m_first_zone;
        bool m_positioning;

        //P = PARAMETER
        //State
        double p_waiting_start;
        //Positioning
        double p_destination_x;
        double p_destination_y;
        double p_trajectory_distance;
        double p_trajectory_theta;
        double p_localization_border;
        double p_stupid_localization_border;
        double p_resetTimeStart;
    };
} // namespace robotis_op
#endif
