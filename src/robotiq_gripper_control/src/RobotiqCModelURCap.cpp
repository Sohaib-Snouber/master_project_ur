#include "RobotiqCModelURCap.hpp"
#include <iostream>
#include <thread>
#include <stdexcept>

RobotiqCModelURCap::RobotiqCModelURCap(const std::string& address, int port, double timeout)
    : address_(address), port_(port), socket_timeout_(timeout)
{
    connect();
}

RobotiqCModelURCap::~RobotiqCModelURCap()
{
    disconnect();
}

void RobotiqCModelURCap::activate(bool auto_calibrate)
{
    _set_var(STA, 0);
    _set_var(STA, 1);
    std::cout << "Waiting for activation" << std::endl;
    while (!is_active())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "Activated." << std::endl;

    if (auto_calibrate)
    {
        auto_calibrate_gripper();
    }
}

bool RobotiqCModelURCap::is_active()
{
    int status = _get_var(STA);
    return status == 3;
}

void RobotiqCModelURCap::move(int position, int speed, int force)
{
    position = clip_value(_min_position, position, _max_position);
    speed = clip_value(_min_speed, speed, _max_speed);
    force = clip_value(_min_force, force, _max_force);

    std::map<std::string, int> var_dict = {
        {POS, position},
        {SPE, speed},
        {FOR, force},
        {GTO, 1}};

    _set_vars(var_dict);
    std::cout << "Moving to position: " << position << std::endl;
}

void RobotiqCModelURCap::move_and_wait_for_pos(int position, int speed, int force)
{
    move(position, speed, force);

    while (_get_var(PRE) != position)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    int cur_obj = _get_var(OBJ);
    while (cur_obj == 0) // Moving status
    {
        cur_obj = _get_var(OBJ);
    }

    int final_pos = _get_var(POS);
    std::cout << "Final position: " << final_pos << std::endl;
}

void RobotiqCModelURCap::auto_calibrate_gripper(bool log)
{
    move_and_wait_for_pos(_min_position, 64, 1);
    move_and_wait_for_pos(_max_position, 64, 1);
    move_and_wait_for_pos(_min_position, 64, 1);

    if (log)
    {
        std::cout << "Gripper auto-calibrated to [" << _min_position << ", " << _max_position << "]" << std::endl;
    }
}

void RobotiqCModelURCap::disconnect()
{
    if (socket_.is_open())
    {
        socket_.close();
    }
}

void RobotiqCModelURCap::connect()
{
    asio::ip::tcp::resolver resolver(io_context_);
    asio::connect(socket_, resolver.resolve(address_, std::to_string(port_)));
    socket_.set_option(asio::socket_base::keep_alive(true));
    socket_.set_option(asio::ip::tcp::no_delay(true));
}

int RobotiqCModelURCap::get_current_position()
{
    return _get_var(POS);
}

bool RobotiqCModelURCap::_set_vars(const std::map<std::string, int>& var_dict)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    std::string cmd = "SET";
    for (const auto& [variable, value] : var_dict)
    {
        cmd += " " + variable + " " + std::to_string(value);
    }
    cmd += "\n";

    asio::write(socket_, asio::buffer(cmd));

    return _is_ack(_read_response());
}

bool RobotiqCModelURCap::_set_var(const std::string& variable, int value)
{
    return _set_vars({{variable, value}});
}

int RobotiqCModelURCap::_get_var(const std::string& variable)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    std::string cmd = "GET " + variable + "\n";
    asio::write(socket_, asio::buffer(cmd));

    auto response = _read_response();
    auto [var_name, value_str] = _parse_response(response);

    if (var_name != variable)
    {
        throw std::runtime_error("Unexpected response: " + response);
    }

    return std::stoi(value_str);
}

std::string RobotiqCModelURCap::_read_response()
{
    char data[1024];
    size_t len = socket_.read_some(asio::buffer(data));
    return std::string(data, len);
}

std::pair<std::string, std::string> RobotiqCModelURCap::_parse_response(const std::string& response)
{
    auto split_pos = response.find(' ');
    if (split_pos == std::string::npos)
    {
        throw std::runtime_error("Invalid response format: " + response);
    }

    std::string var_name = response.substr(0, split_pos);
    std::string value_str = response.substr(split_pos + 1);
    return {var_name, value_str};
}

bool RobotiqCModelURCap::_is_ack(const std::string& data)
{
    return data == "ack";
}

int RobotiqCModelURCap::clip_value(int min_val, int val, int max_val)
{
    return std::max(min_val, std::min(val, max_val));
}
