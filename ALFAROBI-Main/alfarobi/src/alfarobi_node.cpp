#include <ros/ros.h>
#include <std_msgs/String.h>

#include "alfarobi/alfarobi.h"
#include "alfarobi/kepeer.h"
#include "alfarobi/soccer.h"
#include "alfarobi/runner.h"
// #include "alfarobi/dribbler.h"
#include "alfarobi/dribblerv13.h"
#include "alfarobi/passer.h"
#include "alfarobi/penalty_manual.h"
#include "robotis_math/robotis_math.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_controller_msgs/StatusMsg.h"

enum ModuleStatus
{
    Status_Ready = 0,
    Status_Soccer = 1,
    Status_Keeper = 2,
    Status_Running = 3,
    Status_Dribbling = 4,
    Status_Passing = 5,
    Status_Penalty = 6,

};

void buttonHandlerCallback(const std_msgs::String::ConstPtr &msg);
void goInitPose();
bool checkManagerRunning(std::string &manager_name);
void checkBaseReady(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);
void dxlTorqueChecker();

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = true;

ros::Publisher init_pose_pub;
ros::Publisher play_sound_pub;
ros::Publisher led_pub;
ros::Publisher dxl_torque_pub;

bool keeper;
bool runner, dribbler, passer, penalty_manual;
int desired_status;
int current_status = Status_Ready;
bool apply_desired = true;
bool manager_is_ready = false;

////node main
int main(int argc, char **argv)
{
    ////  init ros
    ros::init(argc, argv, "alfarobi_node");

    //create ros wrapper object
    robotis_op::Alfarobi *current = NULL;
    robotis_op::SoccerDemo *soccer_demo = new robotis_op::SoccerDemo();
    robotis_op::KeeperDemo *keeper_demo = new robotis_op::KeeperDemo();
    robotis_op::OlympicRunner *running_demo = new robotis_op::OlympicRunner();
    robotis_op::OlympicDribblerv13 *dribbling_demo = new robotis_op::OlympicDribblerv13();
    robotis_op::OlympicPasser *passing_demo = new robotis_op::OlympicPasser();
    robotis_op::OlympicPenalty *penalty_demo = new robotis_op::OlympicPenalty();

    ros::NodeHandle nh(ros::this_node::getName());

    nh.param<bool>("keeper", keeper, false);
    nh.param<bool>("runner", runner, false);
    nh.param<bool>("dribbler", dribbler, false);
    nh.param<bool>("passer", passer, false);
    nh.param<bool>("penalty_manual", penalty_manual, false);


    if (keeper)
        desired_status = Status_Keeper;
    else
    {
        if (runner)
            desired_status = Status_Running;
        else if (dribbler)
            desired_status = Status_Dribbling;
        else if (passer)
            desired_status = Status_Passing;
        else if(penalty_manual)
            desired_status = Status_Penalty;   
    }

    init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
    led_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
    dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
    ros::Subscriber button_sub = nh.subscribe("/arduino_controller/button", 1, buttonHandlerCallback);
    ros::Subscriber base_status = nh.subscribe("/robotis/status", 1, checkBaseReady);
    ros::start();

    //set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    // wait for starting of manager
    std::string manager_name = "/alfarobi_manager";
    while (ros::ok())
    {
        ros::Duration(1.0).sleep();

        if (checkManagerRunning(manager_name) == true)
        {
            break;
            ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
        }
        ROS_WARN("Waiting for op3 manager");
    }

    // init procedure

    //node loop
    while (ros::ok())
    {
        // process
        if (apply_desired == true)
        {
            switch (desired_status)
            {
            case Status_Ready:

                if (current != NULL)
                    current->setModuleDisable();

                current = NULL;

                goInitPose();

                ROS_INFO_COND(DEBUG_PRINT, "[Go to Pose READY!]");
                break;

            case Status_Soccer:
                if (current != NULL)
                    current->setModuleDisable();

                current = soccer_demo;
                current->setModuleEnable();

                ROS_INFO_COND(DEBUG_PRINT, "[Start] Soccer");
                break;

            case Status_Keeper:
                if (current != NULL)
                    current->setModuleDisable();

                current = keeper_demo;
                current->setModuleEnable();

                ROS_INFO_COND(DEBUG_PRINT, "[Start] Keeper");
                break;

            case Status_Running:
                if (current != NULL)
                    current->setModuleDisable();

                current = running_demo;
                current->setModuleEnable();

                ROS_INFO_COND(DEBUG_PRINT, "[Start] Olympic");
                break;

            case Status_Dribbling:
                if (current != NULL)
                    current->setModuleDisable();

                current = dribbling_demo;
                current->setModuleEnable();

                ROS_INFO_COND(DEBUG_PRINT, "[Start] Olympic");
                break;

            case Status_Passing:
                if (current != NULL)
                    current->setModuleDisable();

                current = passing_demo;
                current->setModuleEnable();

                ROS_INFO_COND(DEBUG_PRINT, "[Start] Olympic");
                break;

            //penalti baru
            case Status_Penalty:
                if (current != NULL)
                    current->setModuleDisable();

                current = penalty_demo;
                current->setModuleEnable();

                ROS_INFO_COND(DEBUG_PRINT, "[Start] Olympic");
                break;

            default:
                break;
            }

            apply_desired = false;
            current_status = desired_status;
        }

        //execute pending callbacks
        ros::spinOnce();

        //relax to fit output rate
        loop_rate.sleep();
    }

    //  //exit program
    return 0;
}

void buttonHandlerCallback(const std_msgs::String::ConstPtr &msg)
{
    if (apply_desired == true)
        return;

    // in the middle of playing demo
    if (current_status != Status_Ready)
    {
        if (msg->data == "ready_pose")
        {
            // go to mode selection status
            desired_status = Status_Ready;
            apply_desired = true;
        }
        else if (msg->data == "user_long")
        {
            // it's using in op3_manager
            // torque on and going to init pose
        }
    }
    // ready to start demo
    else
    {
        if (msg->data == "soccer")
        {
            desired_status = Status_Soccer;
            apply_desired = true;
        }

        if (msg->data == "keeper")
        {
            desired_status = Status_Keeper;
            apply_desired = true;
        }

        // sound out desired status
        switch (desired_status)
        {

        case Status_Keeper:
            dxlTorqueChecker();
            break;

        case Status_Soccer:
            dxlTorqueChecker();
            break;

        default:
            break;
        }

        ROS_INFO_COND(DEBUG_PRINT, "= Start Demo Mode : %d", desired_status);
    }
}

void goInitPose()
{
    std_msgs::String init_msg;
    init_msg.data = "ini_pose";

    init_pose_pub.publish(init_msg);
}

bool checkManagerRunning(std::string &manager_name)
{
    std::vector<std::string> node_list;
    ros::master::getNodes(node_list);

    for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
    {
        if (node_list[node_list_idx] == manager_name)
        {
            ROS_WARN("Manager Found!");
            if (manager_is_ready)
            {
                ROS_WARN("Manager Ready!");
                return true;
            }
            else
            {
                ROS_WARN("Manager Not Ready!");
                return false;
            }
        }
        return true;
    }

    ROS_ERROR("Can't find op3_manager");
    return false;
}

void checkBaseReady(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
    if (msg->status_msg == "Finish Init Pose")
        manager_is_ready = true;
    else
    {
        manager_is_ready = false;
    }
}

void dxlTorqueChecker()
{
    std_msgs::String check_msg;
    check_msg.data = "check";

    dxl_torque_pub.publish(check_msg);
}
