#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <chrono>
#include <deque>
#include <vector>
#include <numeric>

// for convenience
using json = nlohmann::json;
using namespace std::chrono;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

inline double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(x, hi));
}

//
// MessageTimes class for capturing series of time measurements and reporting
// last time delta and/or moving average time delta.
//
class MessageTimes
{
public:
    MessageTimes() {
        prev_time_ = steady_clock::time_point::min();
    }

    // Capture time now and record delta since last capture
    void Capture() {
        steady_clock::time_point cur_time = steady_clock::now();
        if (prev_time_ != steady_clock::time_point::min()) {
            duration<double> span = cur_time - prev_time_;
            deltas_.push_front(span.count());
        }
        if (deltas_.size() > 20) {
            deltas_.pop_back();
        }
        prev_time_ = cur_time;
    }

    // Report latest time delta
    double LastDelta() const {
        return deltas_.front();
    }

    // Report average of last N time delta's
    double AverageDelta(size_t count) const {
        count = std::min(count, deltas_.size());
        return accumulate(deltas_.begin(), deltas_.begin() + count, 0.0) / count;
    }

private:
    steady_clock::time_point prev_time_;
    deque<double> deltas_;
};

double calc_target_speed(double delta_t) {
    // target speed = linear function of time delta
    return std::max(20.0 - 16*delta_t, 5.0);
}

static const double MAX_TURN_ANGLE = 25.0;

// PID gain coefficients for position controller
static const double Kp_POS = 0.32;
static const double Ki_POS = 0.0002;
static const double Kd_POS = 0.54;

// PID gain coefficients for speed controller
static const double Kp_SPEED = 0.05;
static const double Ki_SPEED = 0.0005;
static const double Kd_SPEED = 0.0;

int main()
{
    uWS::Hub h;
    bool is_first_message = true;
    MessageTimes msg_times;

    // PID position controller (to control steering angle)
    PID_Controller pid_position(Kp_POS, Ki_POS, Kd_POS);

    // PID speed controller
    PID_Controller pid_speed(Kp_SPEED, Ki_SPEED, Kd_SPEED);

    h.onMessage([&](
        uWS::WebSocket<uWS::SERVER> ws, 
        char *data, 
        size_t length, 
        uWS::OpCode opCode) 
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length < 3 || data[0] != '4' || data[1] != '2') {
            return;
        }

        string s = hasData(std::string(data).substr(0, length));

        if (s == "") {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
        }

        json j = json::parse(s);
        string event = j[0].get<std::string>();

        if (event != "telemetry") {
            return;
        }

        // Track message time delta's for precise PID control
        msg_times.Capture();
        double delta_t = 1.0;
        double avg_delta_t = 1.0;

        if (!is_first_message)
        {
            delta_t = msg_times.LastDelta();  // seconds since last message
            avg_delta_t = msg_times.AverageDelta(5);
            std::cout << "Elapsed time: " << delta_t << std::endl;
        }

        // j[1] is the data JSON object
        double cte = std::stod(j[1]["cte"].get<std::string>());
        double speed = std::stod(j[1]["speed"].get<std::string>());
        double angle = std::stod(j[1]["steering_angle"].get<std::string>());
        std::cout << "CTE: " << cte << " Speed: " << speed << " Angle: " << angle << std::endl; 

        // Use time delta to scale target speed
        double target_speed = calc_target_speed(avg_delta_t);
        std::cout << "Target speed: " << target_speed << std::endl;

        // Current position & speed errors (inputs to PID controllers)
        double position_error = cte;
        double speed_error = speed - target_speed;
        
        // Compute steering & speed adjustments (except on initial message)
        double steer_adj = 0.0;
        double speed_adj = 0.0;

        if (is_first_message)
        {
            // Seed controllers with initial error measurements.
            pid_position.TrackError(position_error, delta_t);
            pid_speed.TrackError(speed_error, delta_t);
        }
        else
        {
            // Use PID controller to correct position error.
            double pos_adj = pid_position.TrackError(position_error, delta_t);
            
            // Compute steering adjustment from position correction. Simulator 
            // interprets turn angle = adjustment * (25 degrees), and simulator
            // requires adjustment clamped to [-1,1].
            steer_adj = atan2(pos_adj, target_speed/2) / deg2rad(MAX_TURN_ANGLE);
            steer_adj = clamp(steer_adj, -1.0, 1.0);

            // Use PID controller to correct speed. Simulator requires throttle 
            // correction clamped to [-1,1].
            speed_adj = pid_speed.TrackError(speed_error, delta_t);
            speed_adj = clamp(speed_adj, -1.0, 1.0);
        }

        // send steering/throttle adjustments back to simulator
        json msgJson;
        msgJson["steering_angle"] = steer_adj;
        msgJson["throttle"] = speed_adj;
        string msg = "42[\"steer\"," + msgJson.dump() + "]";
        std::cout << msg << std::endl;
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        
        is_first_message = false;
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
