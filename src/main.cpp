#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;
using std::string;

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

static const double MAX_TURN_ANGLE = deg2rad(25.0);
static const double TARGET_SPEED = 15.0;

int main()
{
    uWS::Hub h;
    PID pid_position, pid_speed;

    // Kp=0.55, Ki=0.0, Kd=1.8, TARGET_SPEED=20 is pretty close
    // Kp-0.6, Ki=0.0, Kd=2.0, TARGET_SPEED=15 made it around
    // (0.6, 0.0, 1.1-1.5), 18.0
    // (0.62, 0.0, 1.4), 15.0
    pid_position.Init(0.62, 0.0001, 1.4);

    pid_speed.Init(0.05, 0.0, 0.0);

    h.onMessage([&pid_position, &pid_speed](
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

        // j[1] is the data JSON object
        double cte_pos = std::stod(j[1]["cte"].get<std::string>());
        double speed = std::stod(j[1]["speed"].get<std::string>());
        double angle = std::stod(j[1]["steering_angle"].get<std::string>());
        std::cout << "CTE: " << cte_pos << " Speed: " << speed << " Angle: " << angle << std::endl; 

        /*
        * TODO: Calcuate steering value here, remember the steering value is
        * [-1, 1].
        * NOTE: Feel free to play around with the throttle and speed. Maybe use
        * another PID controller to control the speed!
        */

        pid_speed.UpdateError(speed - TARGET_SPEED);
        double accel_value = pid_speed.TotalError();
        accel_value = std::min(accel_value, 1.0);
        accel_value = std::max(accel_value, -1.0);

        // if (speed < TARGET_SPEED/2) {
        //     accel_value = 0.3;
        // }

        pid_position.UpdateError(cte_pos);
        double steer_value = atan2(pid_position.TotalError(), std::max(speed, TARGET_SPEED));
        steer_value /= MAX_TURN_ANGLE;
        //  steer_value = std::min(steer_value, 1.0);
        //  steer_value = std::max(steer_value, -1.0);

        // HACK: remove steering bias for now
        //steer_value -= deg2rad(1.0);

        // DEBUG
        //   std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

        json msgJson;
        msgJson["steering_angle"] = steer_value;
        msgJson["throttle"] = accel_value;
        string msg = "42[\"steer\"," + msgJson.dump() + "]";
        std::cout << msg << std::endl;
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
