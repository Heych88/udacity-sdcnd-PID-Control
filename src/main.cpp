#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

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

// Belows code is copied from Andrey Glushko slack thread
void resetSimulator(uWS::WebSocket<uWS::SERVER>& ws)
{
    // reset
    std::string msg("42[\"reset\", {}]");
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}



int main()
{
  uWS::Hub h;
  
  PID steer_pid;
  // TODO: Initialize the PID variable.
  // twiddle optimal parameters Kp: -0.0698338  Ki: -0.00584  Kd: -0.475645 
  double steer_kp = steer_pid.params[0]; //0.2;  // 0.13
  double steer_kd = steer_pid.params[1]; //0; //0.0001; // 0.002
  double steer_ki = steer_pid.params[2]; //0.1;  //2.5
  steer_pid.Init(steer_kp, steer_ki, steer_kd);
    
  PID speed_pid;
  double speed_kp = 0.5;
  double speed_ki = 0.00024;
  double speed_kd = 1.1;
  speed_pid.Init(speed_kp, speed_ki, speed_kd);

  h.onMessage([&steer_pid, &speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          if((cte >= 5.0) || (cte <= -5.0) || (steer_pid.current_twiddle > 2000)) {
            if(steer_pid.use_twiddle){
            
              if(steer_pid.current_twiddle > steer_pid.best_twiddle) {
                // the parameters are better
                steer_pid.best_twiddle = steer_pid.current_twiddle;
                steer_pid.delta_params[steer_pid.current_param] *= 1.1;
              } else {
                // parameters are worse
                steer_pid.delta_params[steer_pid.current_param] -= 2 * steer_pid.delta_params[steer_pid.current_param];
                steer_pid.params[steer_pid.current_param] += steer_pid.delta_params[steer_pid.current_param];
                steer_pid.delta_params[steer_pid.current_param] *= 0.9;
              }

              if(fabs(steer_pid.delta_params[steer_pid.current_param]) < steer_pid.params_thresh[steer_pid.current_param]){
                steer_pid.current_param++;
                steer_pid.best_twiddle = 0;
                if(steer_pid.current_param >= 3){
                  steer_pid.use_twiddle = false;
                }
              }
              steer_pid.current_twiddle = 0;
              steer_pid.Kp = steer_pid.params[0];
              steer_pid.Kd = steer_pid.params[1];
              steer_pid.Ki = steer_pid.params[2];
            }
            resetSimulator(ws);
          } else {
            steer_pid.current_twiddle++;
          }
          
          std::cout << "Kp: " << steer_pid.Kp << "  Ki: " << steer_pid.Ki << "  Kd: " << steer_pid.Kd << "  bt: " << steer_pid.current_twiddle << "  ct: " << steer_pid.best_twiddle << "  cp " << steer_pid.current_param << std::endl;
          double desired_speed = 30;
          double error = desired_speed - speed;
          speed_pid.UpdateError(error);
          double throttle_value = speed_pid.output;
          
          steer_pid.UpdateError(cte);
          double steer_value = steer_pid.output;
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
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
