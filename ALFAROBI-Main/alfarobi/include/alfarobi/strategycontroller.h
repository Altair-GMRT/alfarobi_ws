#ifndef STRATEGYCONTROLLER_H
#define STRATEGYCONTROLLER_H

#include <string>
#include <vector>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

#include <robotis_math/robotis_math.h>

#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <team_communication/team_data.h>
#include <team_communication/robot_data.h>

namespace robotis_op
{

class Strategy
{
public:
    enum StrategyState
    {
        WorkAlone   = 0,
        SearchBall  = 1,
        GoToBall    = 2,
        WaitHere    = 3,
        AlignToBall = 4,
    };

    Strategy();
    ~Strategy();

    char process();
    Eigen::Vector2d getGlobalBall(){return global_mean;}
    Eigen::Vector3d getRobotPos(){return robot_pos;}
    void setballFound(bool state){ r_ballFound = state;}
    void setAlignment(bool state){ ballisAligned = state;}

    void TeamDataCallback(const team_communication::team_data::ConstPtr &msg);
    void LocalizationDataCallback();
private:
    std::string robot_name;
    unsigned char robot_id;
    bool r_ballFound; //self robot
    bool f_ballFound; //friend robot
    bool friendAvail;
    bool useComm;
    unsigned char num_robot;
    unsigned char toDo;
    bool ballisAligned;

    team_communication::team_data comm_data_;

    Eigen::Vector3d robot_pos; //self robot
    std::vector<Eigen::Vector3d> robots_pos_;
    Eigen::Vector2d robot_ball;
    Eigen::Vector2d global_mean;
    double mean_x;
    double mean_y;

    void calculateGlobalMean();
    void calculateSelf();
    void CalculateDistance();
    char whatToDo();
};

}

#endif // STRATEGYCONTROLLER_H
