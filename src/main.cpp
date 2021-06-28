#include <math.h>
#include <uWS/uWS.h>

#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "PID.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  /**
   * Initialize the pid variable.
   */

  PID pid_a;

  // Manual tuning history
  // pid_a.Init(0.1, 0.001, 1);
  // The vehicle ran out of the left lane --> increase Kp to decrease T-rise
  // pid_a.Init(0.2, 0.001, 1);
  // increase Kp to decrease T-rise
  // pid_a.Init(0.25, 0.001, 2);
  // The vehicle oscillate --> increase Kd to decrease overshoot
  // pid_a.Init(0.25, 0.001, 3);
  // The vehicle oscillate --> increase Kd to decrease overshoot
  // pid_a.Init(0.25, 0.001, 4);  // status：overshoot --> increase Kd
  // When the vehicle runs through a fast curve lane, goes out of the lane.
  // increase Kd to deal with strong changes in future data
  // pid_a.Init(0.25,0.001, 5);  // status：overshoot --> increase Kd The
  // vehicle oscillate wiht weakening intensity  --> increase Kd to decrease
  // overshoot
  // pid_a.Init(0.25, 0.001, 5.5);
  // slight overshoot
  pid_a.Init(0.25, 0.001, 6.0);

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket
    // message event. The 4 signifies a websocket message The 2
    // signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle = 0.3;

          /**
           * Calculate steering value here, remember the steering
           * value is [-1, 1].
           */

          // error term input
          pid_a.UpdateError(cte);

          steer_value = pid_a.TotalError();

          // clamping  check
          double pre_steer_value = steer_value;

          // clamping sturation limit
          steer_value = steer_value > 1 ? 1 : steer_value;
          steer_value = steer_value < -1 ? -1 : steer_value;

          /**
           * clamping - turning the integrator off
           * when the output is sturating and the error is the same sign as the
           * controller output
           */
          if (fabs(steer_value) >= 1) {
            if (((cte > 0 && steer_value > 0) ||
                 (cte < 0 && steer_value < 0)) &&
                (pre_steer_value != steer_value)) {
              // switch off
              pid_a.Init(0.25, 0.0, 6.0);
            } else {
              pid_a.Init(0.25, 0.001, 6.0);
            }
          }

          // DEBUG
          std ::cout << "CTE: " << cte << " Steering Value: " << steer_value
                     << std::endl;
          std ::cout << "CTE: " << cte << " Angle: " << angle
                     << " Speed: " << speed << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  });  // end h.onMessage

  h.onConnection([&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&](uWS::WebSocket<uWS::SERVER> ws, int code, char *message,
                        size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}