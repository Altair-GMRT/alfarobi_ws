#ifndef OLYMPIC_H
#define OLYMPIC_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "alfarobi.h"
#include "behavioral.h"
#include "ball_tracker.h"
#include "pillar_tracker.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "strategycontroller.h"

namespace robotis_op
{
    class OlympicRunner : public Alfarobi
    {

    public:
        OlympicRunner();
        ~OlympicRunner();

        void setModuleEnable();
        void setModuleDisable();

        enum RunningStatus
        {
            Runner_Stop = 0,
            Runner_Start = 1,
            Runner_Run = 2,
            Runner_Waiting = 3,
        };
        enum FacingStatus//NEW
        {
            faceUpPole = 0,
            faceUpTurn = 1,
            faceUpSquare = 2,
        };
        enum PhaseStatus//NEW
        {
            Runner_To_Pillar = 0,
            Runner_To_Square = 1,
        };
        enum RevoState
        {
            firstRevo_ = 0,
            Second_Revo_ = 1,
        };
        enum strategy_state
        {
            GoPillar = 0,
            GoTurning = 1,
        };
double timeStart;
        bool firstTrue = true; //STRATEGI 1
        bool firstTrack = true; //STRATEGI 1

    protected:
        const int SPIN_RATE;

        BallTracker tracker_;
        PillarTracker tracker_pillar_;
        Behavioral behavioral_;
        Strategy strategy_;
    //    FacingStatus face = 0;//NEW
       // PhaseStatus phase = 0;//NEW

        //ROS
        ros::NodeHandle nh_;
        int wait_count_;
        ros::Subscriber demo_command_sub_;

        void loadConfig(const std::string &path);
        void queueThread();
        void processThread();
        void CommandCallback(const std_msgs::String::ConstPtr &msg);

        void startRunMode();
        void stopRunMode();

        void buttonStatus();

        void runnerState();
        void standState();
        void strategyState();

        bool s_first_found;
        bool s_calibration;
        bool finish_run_up;
        bool finish_turn;//NEW
        int s_tracking_mode;

        int m_tracking_status;
        int m_strategy_state;
        int m_button_status;
        int m_stand_state;
        int m_runner_state;
        int status; //new
        int m_revo_status_; // new
        int m_facing_state ;
        double m_resetTime;

        double p_resetTimeStart;

        double resetTimeNow;
        double  p_stupid_localization_border;
        bool revo_succes;//New
    };
} // namespace robotis_op
#endif
