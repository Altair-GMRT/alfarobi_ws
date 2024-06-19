    #ifndef BEHAVIORAL_H_
#define BEHAVIORAL_H_

#include <iostream>
#include <random>
#include <vector>
#include <math.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include "controller.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
//baru v12
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>

#include "game_controller/GameState.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "zmp_walking_module_msgs/ZMPWalkingParam.h"
#include "zmp_walking_module_msgs/GetZMPParam.h"
#include "quintic_walk_msgs/WalkingParam.h"
#include "quintic_walk_msgs/GetWalkingParam.h"
#include "pillar_tracker.h" //STRATEGI 1
#include "ball_tracker.h"
#include "goalpost_tracker.h"
#include "pillar_tracker.h"
#include "square_tracker.h"
#include "robotis_math/robotis_math.h"
//baru v12
#include "cat2_tracker.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
// #include "v9_localization/v9_localization.h"

#define PI 3.14159265

namespace robotis_op
{

class Behavioral
{
public:

   //STRATEGI 1
    void faceUpSquare(double robot_theta, double ball_x, double ball_y, double ball_theta);//NEW
    void followingRunner(int following_state, double robot_theta, double ball_x, double ball_y, double ball_theta);//NEW
    // void searchPillar (double robot_x, double robot_y, double robot_theta, double border);//NEW
    void initiateTurning();//NEW
    bool faceUpTurning(double robot_theta, double ball_x, double ball_y, double ball_theta);//NEW
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void stupidSearchForRunner();//STRATEGI 1
    bool goToPillar(double robot_theta, double ball_x, double ball_y, double ball_theta);//STRATEGI1
    void backToHome();//STRATEGI 1
    void awayFromHome();//STRATEGI 1
    void send();
    bool stableWalk();
    void resetComm();
    bool alreadyStraight = false;
    bool checkForTime = true;
    bool checkForTimePillar = true;
    bool checkForPillarY = true;
    bool turnOnStartFollowing = true;
    bool checkForStop = true;
    bool setGain = true;
    double timeForForward = 6;
    double forwardTime;
    double timeForStop;
    double timeStartForward;
    double orientation_check;
    int runnerPillarState;
    int runnerBackState;
    int runnerAwayState;
    double previousPillarY;
    double currentPillarY;
    ros::Subscriber imu_callback;
    int runnerSearchState;
    double PILLAR_X_LIMIT,PILLAR_Y_LIMIT;
    enum Runner_Pillar_State
    {
        Runner_Pillar_Forward    = 0,
        Runner_Pillar_Shift      = 1,
        Runner_Pillar_Rotate     = 2,
        Runner_Pillar_Revo       = 3,
        Runner_Pillar_ForwardMin = 4,
        Runner_Pillar_Backward   = 5,
        Runner_Pillar_OnTheSpot  = 6,
        Runner_Pillar_Trantition = 7
    };
    enum Runner_Away_State
    {
        Runner_Away_Forward    = 0,
        Runner_Away_Shift      = 1,
        Runner_Away_Rotate     = 2,
        Runner_Away_Revo       = 3,
        Runner_Away_ForwardMin = 4,
        Runner_Away_Backward   = 5,
        Runner_Away_OnTheSpot  = 6,
        Runner_Away_Trantition = 7
    };
    enum Runner_Back_State
    {
        Runner_Back_Forward    = 0,
        Runner_Back_Shift      = 1,
        Runner_Back_Rotate     = 2,
        Runner_Back_Revo       = 3,
        Runner_Back_ForwardMin = 4,
        Runner_Back_Backward   = 5,
        Runner_Back_OnTheSpot  = 6,
        Runner_Back_Trantition = 7
    };
    //STRATEGI 1 SELESAI

    enum KeeperMode
    {
        Fallen_Mode     = 0,
        Shifting_Mode   = 1,
    };

    enum Pass_State{
        Pass_Forward = 0,
        Pass_Rotate = 1,
        Pass_Diam = 2,
    };

    enum Stand_Status
    {
        Stand          = 0,
        Fallen_Forward = 1,
        Fallen_Behind  = 2,
        Fallen_Right   = 3,
        Fallen_Left    = 4,
        Nothing        = 5,
        Waiting        = 6,
    };

    enum Following_State
    {
        Following_Forward    = 0,
        Following_Shift      = 1,
        Following_Rotate     = 2,
        Following_Revo       = 3,
        Following_ForwardMin = 4,
        Following_Backward   = 5,
        Following_OnTheSpot  = 6,
        Following_Trantition = 7,
        Following_Stop = 8,
    };

    enum AllignBall
    {
        GoToBall = 0,
        GoToEnemy = 1,
    };

    enum Following
    {
        Face_Up_Enemy = 0,
        Face_Up_Home  = 1
    };

    enum Keeper
    {
        Do_Nothing      = 0,
        Keeper_Front    = 1,
        Keeper_Right    = 2,
        Keeper_Left     = 3,
        Shift_Right     = 4,
        Shift_Left      = 5,
        Rotate_CW       = 6,
        Rotate_CCW      = 7,
        Keeper_Rotate   = 8,
        Keeper_Shifting = 9,
        Following       = 10,
        Align_Pose      = 11,
        Positioning     = 12,
        Prediction      = 13,


        Kick = 99
    };

    enum Kicking
    {
        Not_Kicking   = 0,
        Kicking       = 1,
        Finished_Kick = 2
    };

    enum Smart_Search
    {
        FirstZone   = 0,
        Destination = 1,
        LookAround  = 2,
        AlignToZone = 3,
        GoingToZone = 4
    };

    enum Stupid_Search{
        Search_First = 0,
        Search_Forward = 1,
        Search_Rotate = 2,
    };

    enum Rotate_State{
        Inner = 0,
        Outer = 1,
    };

    //anyar
    enum Search_State{
        Search_Pillar = 0,
        Search_Home = 1,
    };

    enum Revo_Status{
        firstRevo = 0,
        secondRevo = 1,
    };

    enum Revo_State{
        first = 0,
        second = 1,
        dribbling = 2,
    };

      // enum for STRATEGI 1
    enum Revo_Status_{    
        firstRevo_ = 0,
        //secondRevo = 1,
    };
    // enum for STRATEGI 1
    enum Turning_Condition{
        first_   = 0,
        second_  = 1,
        third_   = 2,
        fourth_  = 3,
        fifth_   = 4,
    };
       

    class MotionIndex
    {
        public:
            int InitPose;
            int WalkingReady;
            int GetUpFront;
            int GetUpBack;
            int GetUpRight;
            int GetUpLeft;
            int Ceremony;
            int KeeperReady;
            int KeeperFront;
            int KeeperRight;
            int KeeperLeft;
            int RightKick;
            int LeftKick;
    } Motion_Index;

    Behavioral();
    ~Behavioral();

    int comm_gameState;
    bool comm_allowedToMove;
    double comm_timeGame;
    bool swap_score;
    bool comm_scoreChanged();

    int buttonStatus;
    int keyStatus;
    bool walkingStatus;
    int kickingStatus;
    bool lookStatus;
    int trackingMode;
    int counterAction;
    int keyCheck;

    //BARU V12
    float jarakX;

    int keeperState;
    int searchState;
    
    int timeAwal;

    int GAME_TIME;
    bool LOCALIZATION;
    bool COMMUNICATION;

    void queueThread();
    void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
    void controllerCallback(const std_msgs::UInt8 &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void line_pos_callback(const geometry_msgs::Point);
    // void RobotStateCb(const geometry_msgs::PoseStamped &robot_state_msg);


    void angleCallback(const geometry_msgs::PointConstPtr &_msg);
    void odometryCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void getMsgHeadCallback(const std_msgs::String::ConstPtr& msg);
    void currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void kickingCallback(const std_msgs::String::ConstPtr& msg);
    void friendCallback(const std_msgs::Bool::ConstPtr& msg);
    void walkingCallback(const std_msgs::String::ConstPtr& msg);
    void gameControllerCallback(const game_controller::GameState::ConstPtr& msg);
    
    //KOMUNIKASI KRSBI 2022 DHONAN
    void robotStatusCallback(const std_msgs::Int8::ConstPtr& msg);
    void alfaStartPass();
    void abiWaiting();
    void resetFSM();


    bool CALIBRATION;
    double FORWARD_MIN_LIMIT;
    double RL_MIN_LIMIT;
    double KICK_DELAY;
    double kickTimeNow;
    bool kickingDelay;

    void start();
    void startFollowing();
    void stopFollowing();
    void startKicking(const std::string &position);
    void playMotion(int motion_index);

    bool isHeadJoint(const int &id);
    bool isActionRunning();

    int checkFallen();

    void setWalkingParam(double x_move, double y_move, double rotation_angle);

    double remain_xDist;
    double remain_yDist;
    double mileage_xDist;
    double mileage_yDist;
    int robotZone(double robot_x, double robot_y);
    int destinationZone(int first_zone, int robot_zone);
    Eigen::Vector2d randomizedZone(int destination_zone, double border);
    Eigen::Vector2d calcRobotDiff(double robot_x, double robot_y, double robot_theta, double destination_x, double destination_y);
    bool goToLocation(double x_dist, double y_dist, double robot_theta);
    void smartSearch(double robot_x, double robot_y, double robot_theta, double border);
    void stupidSearch(double robot_x, double robot_y, double robot_theta, double border);
    void passerSearch(bool role);
    void passerFollowing(double robot_theta, double ball_x, double ball_y, double ball_theta);
    void dribblerFollowing(double robot_theta, double ball_x, double ball_y, double ball_theta);
    bool positioning(double goal_x, double goal_y);
    bool timePositioning();
    bool dribblePasser();
    bool myBalls(double robot_theta, double ball_x, double ball_y, double ball_theta);
    void yourBalls(double robot_theta, double ball_x, double ball_y, double ball_theta);
    bool yourBalls_2(double robot_theta, double ball_x, double ball_y, double ball_theta);
    bool yourBalls_3(double robot_theta, double ball_x, double ball_y, double ball_theta);

    bool GoingToBall(double robot_theta, double ball_x, double ball_y, double ball_theta);
    bool FarizsBalls(double robot_theta, double ball_x, double ball_y, double ball_theta);
    void RizCheckBalls(double robot_theta, double ball_x, double ball_y, double ball_theta);
    bool kickingHarusKanan();
    bool AlignToOri(double robot_theta, double ball_x, double ball_y, double ball_theta);
    void passerCheckFollowing(double robot_theta, double ball_x, double ball_y, double ball_theta);
    bool WaitingBall();
    
    //anyar
    void searchPillar(double robot_x, double robot_y,  double robot_theta, double border);
    void searchHome(double robot_x, double robot_y, double robot_theta, double border);
    bool faceUpHomeV2(double robot_theta, double ball_x, double ball_y, double ball_theta);
    bool positionCheck(double ball_x, double ball_y, double ball_theta);
    bool alignToPillar();
    bool alignToHome(double ball_x_to_home, double ball_y_to_home, double home_x, double home_y);
    bool dribble(double distance);
    bool back_dribble(double distance);
    bool RevoPillar(double pillar_x);

    bool fallen(int fallen_status);
    void fallenKeeper(int keeper_fallen);
    void keeper(int keeper_mode, double robot_theta, double ball_x, double ball_y, double hold_time, double ball_theta);
    void gabKeeper(int keeper_mode, double hold_time, double robot_x, double robot_y, double robot_theta, double ball_x, double ball_y, double ball_theta);

    Eigen::Vector2d checkFollowing(double robot_theta, double ball_x, double ball_y, double ball_theta);
    void following(int following_state, double robot_theta, double ball_x, double ball_y, double ball_theta);
    bool faceUpEnemy(double robot_theta, double ball_x, double ball_y, double ball_theta);
    bool faceUpHome(double robot_theta, double ball_x, double ball_y, double ball_theta);

    void waiting(Eigen::Vector2d ball_pos, Eigen::Vector3d robot_pos);
    bool alignToBall(Eigen::Vector2d ball_pos, Eigen::Vector3d robot_pos);
    bool reposition();

    bool forward(double distance);
    bool shift(double distance);
    bool rotate(double angle);
    bool revo(double angle);
    void kicking();

    void setModuleTo(const std::string &module_name);
    void setBodyModuleTo(const std::string &body_module, bool with_head_control = true);

    void setEnemyDir();

    int rotateState;
    int followState;
    int lastFollowState;
    double resetOffset;
    double m_yaw;
    bool firstSearch;
    bool firstSearch_homeV2;
    bool dribble_condition;
    bool kanan;
    bool finishedKick=false;
    bool lastKick;
    bool firstKick;
    bool friend_status;
    bool firstIn, firstWait;
    bool repo_tt=true;
    //anyar
    int m_revo_status;
    int m_revo_state;
        //Turn
    int m_revo_status_;//STRATEGI 1
    int m_revo_state_;//STRATEGI 1
    bool RevoPillar_(double pillar_x); //STRATEGI 1

//baruuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu kahlishaa
    double s_angle_yaw;
    double ball_theta;

    geometry_msgs::Pose2D odometry_;
    std::vector<geometry_msgs::Pose2D> odometry_buffer_;
    Eigen::Vector2d getOdometry();

    bool bo_ol = false;

    // KOMUNIKASI REGIONAl KRSBI 2022 DHONAN
    short m_robot_status = -1;

    // //position from localization
    // geometry_msgs::Pose2D robot_position;

protected:

    //anyar
    BallTracker tracker_;
    GoalpostTracker tracker_goalpost_;
    PillarTracker tracker_pillar_;
    SquareTracker tracker_square_;

    bool DEBUG_PRINT;

    double FRONT_FALL_LIMIT;
    double BEHIND_FALL_LIMIT;
    double RIGHT_FALL_LIMIT;
    double LEFT_FALL_LIMIT;

    double MAX_FB_STEP;
    double MIN_FB_STEP;

    double MAX_BW_STEP;
    double MIN_BW_STEP;

    double MAX_RL_STEP;
    double MIN_RL_STEP;

    double MAX_RL_TURN;
    double MIN_RL_TURN;

    double REVO_CW_STEP;
    double REVO_CW_TURN;
    double REVO_CCW_STEP;
    double REVO_CCW_TURN;

    double VELOCITY_LIMIT;
    double BALL_X_LIMIT;

    double FORWARD_MIN_REVO;
    double RL_MIN_REVO;

    double TRANTITION_BORDER;

    double KICKING_TOLERANCE;
    double ANGLE_TOLERANCE;
    double BALL_TOLERANCE;
    double REVO_TOLERANCE;
    double BACKWARD_TOLERANCE;
    double SHIFTING_TOLERANCE;

    //strategI 1
    double X_MIN_LIMIT;
    double Y_LEFT_LIMIT_FIRST;
    double Y_RIGHT_LIMIT_FIRST;
    double Y_LEFT_LIMIT_SECOND;
    double Y_RIGHT_LIMIT_SECOND;
    double Y_LEFT_LIMIT_THIRD;
    double Y_RIGHT_LIMIT_THIRD;
    double Y_LEFT_LIMIT_FOURTH;
    double Y_RIGHT_LIMIT_FOURTH;
    double ANGLE_YAW_LIMIT;

//

    double FB_Step;
    double RL_Step;
    double RL_Turn;
    double UNIT_INCREMENT;
    double increment_fb;
    double increment_rl;
    double increment_turn;

    double kickTimeStart;

    bool ANGLE_LIMIT_TOLERANCE;
    std_msgs::Bool activate_cp_;
    double searchStart;
    double searchStartt;
    int changeDirection;
    static int changeDirectionP;
    

    double angle_pitch;
    double angle_roll;
    double angle_yaw;

    bool penalty;

    //SMART SEARCH
    Eigen::Vector3d robot_zone;
    Eigen::Vector2d zone_point;
    Eigen::Vector2d zone_dist;
    double orientation;

    bool firstGetScore;
    int ownScore, rivalScore;
    geometry_msgs::Pose2D score_;
    std::vector<geometry_msgs::Pose2D> score_buffer_;

    // geometry_msgs::Pose2D odometry_;
    // std::vector<geometry_msgs::Pose2D> odometry_buffer_;
    // Eigen::Vector2d getOdometry();

    geometry_msgs::Pose2D head_;
    geometry_msgs::Pose2D offsetHead;
    std::vector<geometry_msgs::Pose2D> head_buffer_;
    Eigen::Vector2d getHead();

    void loadConfig(const std::string &behavioral, const std::string &offset);
    void parseJointNameFromYaml(const std::string &path);
    void parseMotionIndex(const std::string &path);
    bool getWalkingParam();
    int getPanZone();
    bool getJointNameFromID(const int &id, std::string &joint_name);
    bool getIDFromJointName(const std::string &joint_name, int &id);
    int  getJointCount();

    void setWalkingCommand(const std::string &command);


    void calcWalkingParam(double x_move, double y_move, double rotation_angle);
    void calcFootstep(double target_distance, double target_angle, double delta_time,
                      double& fb_move, double& rl_move, double& turn_move);

    //ros node handle
    ros::NodeHandle nh_;

    //publisher
    ros::Publisher module_control_pub_;
    ros::Publisher ball_proj;
    ros::Publisher angle_yaw_pub_;
    ros::Publisher motion_index_pub_;
    ros::Publisher set_walking_command_pub_;
    ros::Publisher set_walking_param_pub_;
    ros::Publisher set_kicking_command_pub_;
    ros::Publisher kick_status_pub_;
    ros::Publisher comm_status_pub_;
    ros::Publisher cp_pub_;

    //subscriber
    ros::Subscriber button_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber angle_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber game_controller_sub_;
    ros::Subscriber current_joint_states_sub_;
    ros::Subscriber kicking_status_sub_;
    ros::Subscriber friend_status_sub_;
    ros::Subscriber walking_status_sub_;
    ros::Subscriber robot_status_sub_;
    ros::Subscriber controller_sub_;
    //baru v12
    ros::Subscriber jarak_X_sub_;
    ros::Subscriber robot_state_sub_;
    
    //position from localization
    // geometry_msgs::PoseStamped robot_position;

    // void RobotStateCb(const geometry_msgs::PoseStampedConstPtr &msg);

    //client
    ros::ServiceClient is_running_client_;
    ros::ServiceClient get_walking_param_client_;

    std::map<int, std::string> id_joint_table_;
    std::map<std::string, int> joint_id_table_;

    // op3_walking_module_msgs::WalkingParam current_walking_param_;
    zmp_walking_module_msgs::ZMPWalkingParam current_zmp_param_;
    quintic_walk_msgs::WalkingParam current_walking_param_;

    //Prediksi Sudiro
    std::vector<cv::Point2f> vp;
    double getVelocity(std::vector<cv::Point2f> _vp);
    double x_y_score(std::vector<cv::Point2f> _vp, int nilaiX);
    int counterSize;

    //Prediksi
    double CENTER_MARGIN;
    double FALLEN_MARGIN;
    double FOLLOWING_MARGIN;
    double FRAME_NUMBER;
    double REGRESSION_NUMBER;

    bool m_following_status;

    double x[4], y[4];
    double vel_x[2], vel_y[2];
    double x_frame[4], y_frame[4];
    double p_pred_x[10], p_avg_vel[10];

    int predictionSudiro(double pos_x, double pos_y, double ball_x, double ball_y);
    int prediction(double ball_x, double ball_y, double hold_time);

    int predictionMode(double robot_theta, double ball_x, double ball_y, double hold_time, double ball_theta);
    int shiftingMode(double robot_theta, double ball_x, double ball_y, double hold_time, double ball_theta);
    int shifting(double ball_x, double ball_y, double hold_time);

    bool gabPrediction(double hold_time, double robot_x, double robot_y, double robot_theta, double ball_x, double ball_y, double ball_theta);
    bool gabShifting(double robot_x, double robot_y, double robot_theta, double ball_x, double ball_y, double ball_theta);
};
}

#endif
