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

#include <game_controller/game_controller_client_t.h>
#include <iostream>

using namespace robocup;
using namespace std::chrono;
using namespace boost::asio;

constexpr std::chrono::steady_clock::duration game_controller_client_t::GAMECONTROLLER_TIMEOUT;
constexpr std::chrono::steady_clock::duration game_controller_client_t::ALIVE_DELAY;

game_controller_client_t::game_controller_client_t(int team_num, int player_num)
    : m_io(),
      m_socket(m_io, ip::udp::endpoint(ip::udp::v4(), GAMECONTROLLER_DATA_PORT)),
      m_player_number(player_num),
      m_team_number(team_num) {
    m_socket.non_blocking(true);
    m_socket.set_option(ip::udp::socket::reuse_address(true));
    m_socket.set_option(ip::udp::socket::broadcast(true));
    this->reset();
}


void game_controller_client_t::reset() {
    m_ep = ip::udp::endpoint(ip::udp::v4(), 0);
    m_game_ctrl_data = game_control_data_t();
    m_last_message_received = steady_clock::time_point();
    m_last_message_sent = steady_clock::time_point();

}


void game_controller_client_t::update() {
    this->receive();
    this->send_alive();
}


bool game_controller_client_t::is_server_responds() const  {
    auto now = steady_clock::now();
    return now - m_last_message_received > GAMECONTROLLER_TIMEOUT;
}


bool game_controller_client_t::is_server_was_found() const  {
    return m_ep.port() != 0;
}


void game_controller_client_t::send_manual_penalise(bool status) {
    auto msg = status
            ? (uint8_t) GAMECONTROLLER_RETURN_MSG_MAN_PENALISE
            : (uint8_t) GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE;
    this->send(msg);
}


void game_controller_client_t::send_alive() {
    auto now = steady_clock::now();
    bool responds_ok = now - m_last_message_received < GAMECONTROLLER_TIMEOUT;
    bool time_passed_ok = now - m_last_message_sent >= ALIVE_DELAY;
    if (responds_ok && time_passed_ok) {
        this->send(GAMECONTROLLER_RETURN_MSG_ALIVE);
    }
}


int game_controller_client_t::get_player_number() const  {
    return m_player_number;
}


void game_controller_client_t::set_player_number(int player_number)  {
    m_player_number = player_number;
}


int game_controller_client_t::get_team_number() const  {
    return m_team_number;
}


void game_controller_client_t::set_team_number(int team_number)  {
    m_team_number = team_number;
}


void game_controller_client_t::send(uint8_t message) {
    game_control_return_data_t return_packet;
    return_packet.team = (uint8_t) m_team_number;
    return_packet.player = (uint8_t) m_player_number;
    return_packet.message = message;
    if (m_team_number != 0 && this->is_server_was_found()) {
        auto buff = buffer((char*) &return_packet, sizeof(game_control_return_data_t));
        m_socket.send_to(buff, m_ep);
        m_last_message_sent = steady_clock::now();
    }
}


void game_controller_client_t::receive()  {
    bool received = false;
    size_t size = 0;
    game_control_data_t buff{};
    ip::udp::endpoint sender_ep;
    boost::system::error_code ec{};

    while ((size = m_socket.receive_from(buffer((char*)&buff, sizeof(buff)), sender_ep, 0, ec)) > 0) {
        bool team_number_ok = m_team_number != 0;
        bool size_ok = (sizeof(buff) == sizeof(game_control_data_t));
        bool header_ok = std::memcmp(&buff, GAMECONTROLLER_STRUCT_HEADER, 4) == 0 ? 1: 0;
        bool version_ok = buff.version == GAMECONTROLLER_STRUCT_VERSION;
        bool has_our_team_ok = (int)buff.teams[0].team_number == m_team_number ||
                (int)buff.teams[1].team_number == m_team_number;

        if (team_number_ok && size_ok && header_ok && version_ok && has_our_team_ok) {
            m_game_ctrl_data = buff;
            received = true;
            m_ep = ip::udp::endpoint(sender_ep.address(), GAMECONTROLLER_DATA_PORT);
        }
    }
    if (received) {
        m_last_message_received = steady_clock::now();
    }

}

const game_control_data_t& game_controller_client_t::get_game_ctrl_data() const  {
    return m_game_ctrl_data;
}
