#ifndef KEEPER_H
#define KEEPER_H

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

class KeeperDemo : public Alfarobi
{
public:
    KeeperDemo();
    ~KeeperDemo();

    void setModuleEnable();
    void setModuleDisable();

protected:
    enum KeeperStatus
    {
        Keeper_Stop    = 0,
        Keeper_Start   = 1,
        Keeper_Running = 2,
        Keeper_Waiting = 3,
    };

    enum GameStatus
    {
        Game_Initial = 0,
        Game_Ready   = 1,
        Game_Set     = 2,
        Game_Play    = 3,
        Game_Finish  = 4,
    };

    enum SecondaryStatus
    {
        State_Normal       = 0,
        State_PenaltyShoot = 1,
        State_Overtime     = 2,
        State_Timeout      = 3,
        State_Freekick     = 4,
        State_PenaltyKick  = 5,
    };
    

    enum Cameara_Track
    {
        Ball = 0,
        Goal = 1,
    };
    const int SPIN_RATE;
    const bool DEBUG_PRINT;

    void queueThread();
    void processThread();
    void CommandCallback(const std_msgs::String::ConstPtr& msg);

    void startKeeperMode();
    void stopKeeperMode();

    void buttonState();
    void communicationState();
    bool keeperState();
    void standState();
    void strategyState();
    void conditionkeeper();

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
    bool s_localization;
    //Communication
    bool s_start;
    bool s_stop;
    bool s_first_play;
    bool s_allowed_to_move;
    //Positioning
    bool s_pickup;
    int s_first_zone;
    int s_destination_zone;
    int s_robot_zone;

    //M = MONITORING
    //State
    int m_keeper_state;
    int m_strategy_state;
    int m_stand_state;
    int m_following_state;
    int m_game_state;
    int m_condition_status;
    int m_game_time;
    //Status
    int m_robot_status;
    int m_tracking_status;
    int m_button_status;
    bool m_shifting_status;
    //Positioning
    bool m_search_status;
    int m_first_zone;
    bool m_positioning;

    //P = PARAMETER
    //State
    double p_waiting_start;
    //Found
    double p_first_found;
    double p_first_following;
    int s_camera_track;
};
}
#endif
