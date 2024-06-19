#pragma once

#include "spl_coach_message_t.h"

#define GAMECONTROLLER_DATA_PORT       3838

#define GAMECONTROLLER_STRUCT_HEADER   "RGme"
#define GAMECONTROLLER_STRUCT_VERSION  8

#define MAX_NUM_PLAYERS             11

// SPL
#define TEAM_BLUE                   0 // cyan, blue, violet
#define TEAM_RED                    1 // magenta, pink (not red/orange)
#define TEAM_YELLOW                 2 // yellow
#define TEAM_BLACK                  3 // black, dark gray
#define TEAM_WHITE                  4 // white
#define TEAM_GREEN                  5 // green
#define TEAM_ORANGE                 6 // orange
#define TEAM_PURPLE                 7 // purple, violet
#define TEAM_BROWN                  8 // brown
#define TEAM_GRAY                   9 // lighter grey

// HL
#define TEAM_CYAN                   0
#define TEAM_MAGENTA                1
#define DROPBALL                    255

#define GAME_ROUNDROBIN             0
#define GAME_PLAYOFF                1
#define GAME_MIXEDTEAM_ROUNDROBIN   2
#define GAME_MIXEDTEAM_PLAYOFF      3

#define STATE_INITIAL               0
#define STATE_READY                 1
#define STATE_SET                   2
#define STATE_PLAYING               3
#define STATE_FINISHED              4

#define STATE2_NORMAL               0
#define STATE2_PENALTYSHOOT         1
#define STATE2_OVERTIME             2
#define STATE2_TIMEOUT              3

#define PENALTY_NONE                        0
// SPL
#define PENALTY_SPL_ILLEGAL_BALL_CONTACT    1 // ball holding / playing with hands
#define PENALTY_SPL_PLAYER_PUSHING          2
#define PENALTY_SPL_ILLEGAL_MOTION_IN_SET   3 // heard whistle too early?
#define PENALTY_SPL_INACTIVE_PLAYER         4 // fallen, inactive, local game stuck
#define PENALTY_SPL_ILLEGAL_DEFENDER        5 // own penalty area, center circle during kick-off
#define PENALTY_SPL_LEAVING_THE_FIELD       6
#define PENALTY_SPL_KICK_OFF_GOAL           7 // scored from inside center circle after kick-off
#define PENALTY_SPL_REQUEST_FOR_PICKUP      8
#define PENALTY_SPL_COACH_MOTION            9
// HL Kid Size
#define PENALTY_HL_KID_BALL_MANIPULATION    1
#define PENALTY_HL_KID_PHYSICAL_CONTACT     2
#define PENALTY_HL_KID_ILLEGAL_ATTACK       3
#define PENALTY_HL_KID_ILLEGAL_DEFENSE      4
#define PENALTY_HL_KID_REQUEST_FOR_PICKUP   5
#define PENALTY_HL_KID_REQUEST_FOR_SERVICE  6
#define PENALTY_HL_KID_REQUEST_FOR_PICKUP_2_SERVICE 7
// HL Teen Size
#define PENALTY_HL_TEEN_BALL_MANIPULATION   1
#define PENALTY_HL_TEEN_PHYSICAL_CONTACT    2
#define PENALTY_HL_TEEN_ILLEGAL_ATTACK      3
#define PENALTY_HL_TEEN_ILLEGAL_DEFENSE     4
#define PENALTY_HL_TEEN_REQUEST_FOR_PICKUP  5
#define PENALTY_HL_TEEN_REQUEST_FOR_SERVICE 6
#define PENALTY_HL_TEEN_REQUEST_FOR_PICKUP_2_SERVICE 7

#define PENALTY_SUBSTITUTE                  14
#define PENALTY_MANUAL                      15

struct robot_info_t
{
    uint8_t penalty;               // penalty state of the player
    uint8_t secs_till_unpenalised; // estimate of time till unpenalised
};

struct team_info_t
{
    uint8_t team_number;           // unique team number
    uint8_t team_colour;           // colour of the team
    uint8_t score;                 // team's score
    uint8_t penalty_shot;          // penalty shot counter
    uint16_t single_shots;         // bits represent penalty shot success
    uint8_t coach_message[SPL_COACH_MESSAGE_SIZE]; // the coach's message to the team
    robot_info_t coach;
    robot_info_t players[MAX_NUM_PLAYERS]; // the team's players
};

struct game_control_data_t
{
    char header[4];                // header to identify the structure
    uint8_t version;              // version of the data structure
    uint8_t packet_number;         // number incremented with each packet sent (with wraparound)
    uint8_t players_per_team;      // the number of players on a team
    uint8_t state;                 // state of the game (STATE_READY, STATE_PLAYING, etc)
    uint8_t first_half;            // 1 = game in first half, 0 otherwise
    uint8_t kick_off_team;         // the team number of the next team to kick off or DROPBALL
    uint8_t secondary_state;       // extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
    uint8_t drop_in_team;          // number of team that caused last drop in
    uint16_t drop_in_time;         // number of seconds passed since the last drop in. -1 (0xffff) before first dropin
    uint16_t secs_remaining;       // estimate of number of seconds remaining in the half
    uint16_t secondary_time;       // number of seconds shown as secondary time (remaining ready, until free ball, etc)
    team_info_t teams[2];
};

// data structure header
#define GAMECONTROLLER_RETURN_STRUCT_HEADER      "RGrt"
#define GAMECONTROLLER_RETURN_STRUCT_VERSION     2

#define GAMECONTROLLER_RETURN_MSG_MAN_PENALISE   0
#define GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE 1
#define GAMECONTROLLER_RETURN_MSG_ALIVE          2

struct game_control_return_data_t
{
    char header[4];
    uint8_t version;
    uint8_t team;    // team number
    uint8_t player;  // player number starts with 1
    uint8_t message; // one of the three messages defined above

    // constructor
    game_control_return_data_t()
        : version(GAMECONTROLLER_RETURN_STRUCT_VERSION),
          team(0),
          player(0),
          message(0)
    {
        *(uint32_t*) header = *(uint32_t*) GAMECONTROLLER_RETURN_STRUCT_HEADER;
        version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
//        const char* init = GAMECONTROLLER_RETURN_STRUCT_HEADER;
//        for(unsigned int i = 0; i < sizeof(header); ++i)
//            header[i] = init[i];
    }
};
