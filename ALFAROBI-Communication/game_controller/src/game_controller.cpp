#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <game_controller/game_controller_client_t.h>
#include <game_controller/GameState.h>
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
    int exit_code = EXIT_FAILURE;

    ros::init(argc, argv, "game_controller");
    ros::NodeHandle nh;

    std::string path = ros::package::getPath("op3_manager") + "/config/GlobalConfig.yaml";

    YAML::Node doc;
    try
    {
        doc = YAML::LoadFile(path.c_str());
    } catch (const std::exception& e)
    {
        ROS_ERROR("Fail to load offset config yaml.");
        return -1;
    }

    YAML::Node general_data = doc["general"];

    int team_id = general_data["team_id"].as<int>();
    int robot_id = general_data["robot_id"].as<int>();

    ROS_INFO("Found Config from %s", path.c_str());
    ROS_WARN("Game Controller Team id : %d Robot id : %d", team_id, robot_id);

    robocup::game_controller_client_t client(team_id, robot_id); // You can set player and team numbers later

    ros::Publisher data_publisher = nh.advertise<game_controller::GameState>("alfarobi/game_controller", 10);

    ROS_WARN("Game Controller Starting");
    ros::Rate rate(2);

    while(ros::ok())
    {
        client.update(); // Receive incoming messages and sent alive signal
        if (!client.is_server_responds()) { // Checking that the server is responding
            exit_code = EXIT_SUCCESS;
//            std::cout << "Incoming message data:" << std::endl;
            const auto& data = client.get_game_ctrl_data();
            game_controller::GameState game_data;

            game_data.gameState = (int)data.state;
            game_data.secondaryState = (int)data.secondary_state;
            game_data.firstHalf = (int)data.first_half;
            game_data.secondsRemaining = (int)data.secs_remaining;
            game_data.secondary_seconds_remaining = (int)data.secondary_time;

            if((int)data.secondary_time == 0 && (int)data.state == STATE_PLAYING)
                game_data.hasKickOff = true;
            else
                game_data.hasKickOff = false;

            team_info_t team_data;
            team_info_t rival_data;

            for(int i = 0; i < 2; i++)
            {
                if((int)data.teams[i].team_number == team_id)
                {
                    team_data = data.teams[i];
                    break;
                }
                else
                {
                    rival_data = data.teams[i];
                }
            }

            game_data.singleShots = (int)team_data.single_shots;
            game_data.penaltyShot = (int)team_data.penalty_shot;

            game_data.ownScore = (int)team_data.score;
            game_data.rivalScore = (int)rival_data.score;

            robot_info_t robot_data = team_data.players[robot_id-1];
            game_data.penalty = (int)robot_data.penalty;
            game_data.penalized = (int)robot_data.penalty == 0 ? 0 : 1;

            if((int)data.state == STATE_READY)
                game_data.allowedToMove = true;
            else if ((int)data.kick_off_team == (int)team_data.team_colour && (int)data.state==STATE_PLAYING && (int)robot_data.penalty==0)
                game_data.allowedToMove = true;
            else if ((int)data.kick_off_team != (int)team_data.team_colour && (int)data.state==STATE_PLAYING
                     && game_data.hasKickOff && (int)robot_data.penalty==0)
                game_data.allowedToMove = true;
            else
                game_data.allowedToMove = false;

            game_data.secondsTillUnpenalized = (int)robot_data.secs_till_unpenalised;

            data_publisher.publish(game_data);

        } else {
            ROS_WARN("Waiting for Game Controller messages");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return exit_code;

    \
}
