#pragma once

#include <cstdint>

#define SPL_COACH_MESSAGE_PORT           3839

#define SPL_COACH_MESSAGE_STRUCT_HEADER  "SPLC"
#define SPL_COACH_MESSAGE_STRUCT_VERSION 2
#define SPL_COACH_MESSAGE_SIZE           40

struct spl_coach_message_t
{
    char header[4];        // "SPLC"
    uint8_t version;       // SPL_COACH_MESSAGE_STRUCT_VERSION
    uint8_t team;          // team number
//    uint8_t sequence;      // sequence number of this message

    // buffer for message
    uint8_t message[SPL_COACH_MESSAGE_SIZE];

    // constructor
    spl_coach_message_t() : version(SPL_COACH_MESSAGE_STRUCT_VERSION)
    {
        *(uint32_t*) header = *(uint32_t*) SPL_COACH_MESSAGE_STRUCT_HEADER;
        version = SPL_COACH_MESSAGE_STRUCT_VERSION;
    }
};
