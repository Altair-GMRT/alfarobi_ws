/// Copyright © 2018 Arseniy Ivin <arssivka@yandex.ru>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software
/// and associated documentation files (the “Software”), to deal in the Software without
/// restriction, including without limitation the rights to use, copy, modify, merge, publish,
/// distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
/// Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or
/// substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
/// BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
/// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
///
/// \autor Arseniy Ivin <arssivka@yandex.ru>
/// \date 3/21/18

#pragma once

#include "game_control_data_t.h"
#include <chrono>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

namespace robocup {
    class game_controller_client_t {
    public:
        static constexpr std::chrono::steady_clock::duration GAMECONTROLLER_TIMEOUT = std::chrono::milliseconds(2000);
        static constexpr std::chrono::steady_clock::duration ALIVE_DELAY = std::chrono::milliseconds(1000);

        explicit game_controller_client_t(int team_num = 0, int player_num = 0);

        void reset();

        void update();

        void send_manual_penalise(bool status);

        void send_alive();

        bool is_server_responds() const ;

        bool is_server_was_found() const ;

        int get_player_number() const ;

        void set_player_number(int player_number) ;

        int get_team_number() const ;

        void set_team_number(int team_number) ;

        const game_control_data_t& get_game_ctrl_data() const ;

        ~game_controller_client_t() = default;

    private:
        void send(uint8_t message);
        void receive() ;


    private:
        game_control_data_t m_game_ctrl_data;
        boost::asio::io_service m_io;
        boost::asio::ip::udp::socket m_socket;
        boost::asio::ip::udp::endpoint m_ep;

        int m_player_number = 0;
        int m_team_number = 0;

        std::chrono::steady_clock::time_point m_last_message_received;
        std::chrono::steady_clock::time_point m_last_message_sent;


    };
}
