#ifndef RUNNER_H
#define RUNNER_H

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
#include "std_msgs/Bool.h"
//baru v12
#include "cat2_tracker.h"


namespace robotis_op
{
    class OlympicPasser : public Alfarobi
    {

    public:
        OlympicPasser();
        ~OlympicPasser();

        void setModuleEnable();
        void setModuleDisable();

        enum Passing_Status
        {
            Passer_Stop = 0,
            Passer_Start = 1,
            Passer_Run = 2,
            Passer_Waiting = 3,
        };

        enum Cameara_Track
        {
            Ball = 0,
            // Line = 1,
        };

        enum Robot_Status
        {
            Robot_Passing = 0,
            Robot_Positioning = 1,
            Robot_Dribbling = 2,
            Robot_Kicking = 3,
            Robot_Waiting = 4,
            Robot_Rotating = 5,
            Robot_Start = 6,
        };

        //penalti baru
        enum Penalty_Status
        {
            Penalty_Start = 0,
            Penalty_Running = 1,
            Penalty_Stop = 2,
        };

        // enum RNG_Status
        // {
        //     Titik_Satu = 0,
        //     Titik_Dua = 1,
        //     Titik_Tiga = 2,
        //     Titik_Empat = 3,
        //     Titik_Lima = 4,
        // };

    protected:
        const int SPIN_RATE;

        GoalpostTracker goalTracker_;
        BallTracker tracker_;
        Behavioral behavioral_;
        Strategy strategy_;
        //baru v12
        Cat2Tracker tracker_cat2_;
        
        //ROS
        ros::NodeHandle nh_;
        int wait_count_;
        ros::Subscriber demo_command_sub_;

        void loadConfig(const std::string &path);
        // void ROS_INGFO(const std::string &ingfo);
        void queueThread();
        void processThread();
        void CommandCallback(const std_msgs::String::ConstPtr &msg);

        void startPassMode();
        void stopPassMode();

        void buttonStatus();
        void controllerStatus();

        void altairGamepad();

        void passerState();
        void standState();
        void strategyStateA();
        void strategyStateB();
        void ballState(double orientation);
        bool goalState();
        double kick_angle();
        double ball_to_friend();
        void pengTrueCallback(const std_msgs::Bool::ConstPtr &msg);
        void pengTruePublish();

        //penalti baru
        bool startPenalty;
        void strategyPenaltyState();
        double penalty_angle(double kickDirection);
        double titikTendangan(int RNG);
        //

        bool s_first_found;
        bool s_calibration;
        bool s_role;
        bool firstIn;
        int s_tracking_mode;
        int s_camera_track;

        int m_tracking_status;
        int m_button_status;
        int m_controller_status;
        int m_stand_state;
        int m_runner_state;
        double m_resetTime;
        int m_robot_state;
        int m_passer_state;
        int count;
        bool kick;
        double p_resetTimeStart;
        bool s_pickup;
        bool m_positioning;
        bool m_rotating;
        bool line_trigger;

        double resetTimeNow;
        bool check_fallen = false;

        //filter bola dari behavioral fungsi goingToBall(romii)
        double ball_x_filter;

        double ball_y_filter;
        void ballFilter(double ballPos_x, double ballPos_y, double ball_x_filter, double ball_y_filter);

        // ros::Publisher pengTrue_pub_;
        // ros::Subscriber pengTrue_sub_;

        bool firstMasuk = false;
        double timeAwal, timeTotal;

        //penalti baru
        bool robot_penalty;
        int RNG;
        int m_penalty_state;

        // time keeper
        int awal;
        int akhir;
        int selisih;

    };
} // namespace robotis_op
#endif
