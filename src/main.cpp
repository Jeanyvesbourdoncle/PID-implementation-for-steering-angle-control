//============================================================================
// Name        : main.cpp
// Author      : Jean-Yves Bourdoncle
// Version     : v1.0
// Date		   : 23/06/2019
// Description : Creation of 1 PID controller for the angle steering and one P for the maximum speed restriction
//				 PID Controller (steering angle) with the hyperparameters (behavior (oscillation, bisas correction) of the vehicle with different speed and in the curve)
//				 P Controller (maximum speed supervision) : here 20 MPH
//				 Update Error and total Error calculation for every cycle time
//				 Prevention of a sharpe turm : streer_value = +/- 0.5 in this two extrem case (-1 <steer_value or steer_value > 1 )
//				 Debugging window for the PID Controller (steering angle) and the P Controller (respect of the max speed restriction)
//============================================================================


#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }

  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}



int main() {
  uWS::Hub h;
  PID steering_angle_pid;
  PID speed_pid;

  //Initialize the pid variable.

  steering_angle_pid.Init(0.15,0.003,1.0);
  speed_pid.Init(0.1,0.000,0.0);

  h.onMessage([&steering_angle_pid, &speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

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
          double throttle_value;

          steering_angle_pid.UpdateError(cte);
          steer_value =  steering_angle_pid.TotalError();
          if(steer_value > 1.0) steer_value = 0.5;
          else if(steer_value < -1.0) steer_value = -0.5;


          double max_speed = 20;
          double error_speed = speed - max_speed;
          speed_pid.UpdateError(error_speed);
          throttle_value = speed_pid.TotalError();

          // DEBUG PID Steering Command
          std::cout << "cycle number:" <<  steering_angle_pid.cycle <<"angle:"<<deg2rad(angle)<< "CTE: " << cte << " Steering Value: " 
            << steer_value<< std::endl;
          std::cout << "KP_ steering_angle :" << steering_angle_pid.Kp << "KI_steering_angle :" << steering_angle_pid.Ki 
            << "KD_steering_angle:" << steering_angle_pid.Kd << std::endl;

          // DEBUG PID speed limit  (depends of the CTE)
          std::cout << "cycle number:" <<  speed_pid.cycle << "CTE: " << cte << " Throttle Value: " << throttle_value << std::endl;
          std::cout << "KP_speed :" << speed_pid.Kp << "KI_speed :" << speed_pid.Ki << "KD_speed:" << speed_pid.Kd << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
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
  }); // end h.onMessage


  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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
