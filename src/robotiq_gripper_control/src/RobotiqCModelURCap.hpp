#ifndef ROBOTIQ_CMODEL_URCAP_HPP
#define ROBOTIQ_CMODEL_URCAP_HPP

#include <string>
#include <map>
#include <asio.hpp>
#include <mutex>

class RobotiqCModelURCap
{
public:
    RobotiqCModelURCap(const std::string& address, int port = 63352, double timeout = 2.0);
    ~RobotiqCModelURCap();

    void activate(bool auto_calibrate = false);
    bool is_active();
    void move(int position, int speed = 255, int force = 255);
    void move_and_wait_for_pos(int position, int speed = 255, int force = 255);
    void auto_calibrate_gripper(bool log = true);
    void disconnect();
    int get_current_position();

private:
    std::string address_;
    int port_;
    double socket_timeout_;
    asio::io_context io_context_;
    asio::ip::tcp::socket socket_{io_context_};
    std::mutex command_mutex_;

    int _min_position = 0;
    int _max_position = 255;
    int _min_speed = 0;
    int _max_speed = 255;
    int _min_force = 0;
    int _max_force = 255;

    const std::string ACT = "ACT";
    const std::string GTO = "GTO";
    const std::string ATR = "ATR";
    const std::string ADR = "ADR";
    const std::string FOR = "FOR";
    const std::string SPE = "SPE";
    const std::string POS = "POS";
    const std::string STA = "STA";
    const std::string PRE = "PRE";
    const std::string OBJ = "OBJ";
    const std::string FLT = "FLT";

    void connect();
    bool _set_vars(const std::map<std::string, int>& var_dict);
    bool _set_var(const std::string& variable, int value);
    int _get_var(const std::string& variable);
    std::string _read_response();
    std::pair<std::string, std::string> _parse_response(const std::string& response);
    bool _is_ack(const std::string& data);
    int clip_value(int min_val, int val, int max_val);
};

#endif // ROBOTIQ_CMODEL_URCAP_HPP
