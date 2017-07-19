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
// https://carnd.slack.com/archives/C4Z4GFX0X/p1495663199162875?thread_ts=1495550577.287993&cid=C4Z4GFX0X
// reset the simulator when the car goes off the track or when a lap has 
// completed. Only used when using twiddle.
void resetSimulator(uWS::WebSocket<uWS::SERVER>& ws)
{
    // reset
    std::string msg("42[\"reset\", {}]");
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

// uses a hill climbing method to find good parameters.
// twiddle works as follows
// 1. init the parameters with their starting values, often all zero
// 2. run the system and observer the error
// 3A. if the first run, save the error as the best error 
// 3B. not first, compare the error with the best error and update as:
//  A. best error so far, increase the current adjustable parameter by 1.1 times
//     its current value.
//  B. worse error, decrease the current parameter by the 2 * delta size variable 
//     and also decrease the value of the delta variable.
// 4. check if the error or the delta value is below the accepted threshold level 
//    or runtime has not been exceeded.
// 5. repeat steps 2 to 4 until step 4 is no longer valid
// 6. change to the next parameter to be tuned, with the previous tuned parameters
//    and repeat steps 2 to 5 until all parameters have been tuned. 
//
// as it stands, this code is buggy and only finds workable system parameters.
// TODO: Add a sum of cte error to check for more accurate runs.
// TODO: Check for crashes when the speed goes below 1Mph
void twiddle(PID &steer_pid, double cte, uWS::WebSocket<uWS::SERVER>& ws) {
  if((cte >= 5.0) || (cte <= -5.0) || (steer_pid.current_twiddle > 3000)) {
    if(steer_pid.first_run){
      // if it is the first run, set the best error 
      steer_pid.first_run = false;
      steer_pid.best_twiddle = steer_pid.current_twiddle;
    }

    if(steer_pid.current_twiddle > steer_pid.best_twiddle) {
      // the parameters improved the error so keep improving the parameters
      steer_pid.best_twiddle = steer_pid.current_twiddle;
      steer_pid.delta_params[steer_pid.current_param] *= 1.1;
    } else {
      // parameters decreased accuracy so decrease them.
      steer_pid.delta_params[steer_pid.current_param] -= 2 * steer_pid.delta_params[steer_pid.current_param];
      steer_pid.params[steer_pid.current_param] += steer_pid.delta_params[steer_pid.current_param];
      steer_pid.delta_params[steer_pid.current_param] *= 0.9;
    }

    // check if the error or timeout has passed the threshold to tune the next
    // parameter.
    if((fabs(steer_pid.delta_params[steer_pid.current_param]) < steer_pid.params_thresh[steer_pid.current_param])
        || (steer_pid.current_twiddle > 3000)){
      steer_pid.current_param++;
      steer_pid.best_twiddle = 0;
      steer_pid.first_run = true;

      if(steer_pid.current_param >= 3){
        // all the parameters are tuned so stop twiddeling
        steer_pid.use_twiddle = false;
      }
    }
    // update the controller parameters to the latest values for the next run
    steer_pid.current_twiddle = 0;
    steer_pid.Kp = steer_pid.params[0];
    steer_pid.Ki = steer_pid.params[1];
    steer_pid.Kd = steer_pid.params[2];

    resetSimulator(ws); // reset the simulator
  } else {
    steer_pid.current_twiddle++;
  }
}

int main()
{
  uWS::Hub h;
  
  // Initialize the steering PID controller
  PID steer_pid;
  bool use_twiddel = false;
  double steer_kp, steer_kd, steer_ki;
  if(use_twiddel) {
    // set the twiddle inital parameters.
    steer_kp = steer_pid.params[0]; 
    steer_ki = steer_pid.params[1]; 
    steer_kd = steer_pid.params[2];
  } else {
    // twiddle optimal parameters Kp: -0.091  Ki: -0.01  Kd: -1.52121 
    // manual good parameters Kp: -0.13  Ki: -0.002  Kd: -2.5
    steer_kp = -0.091; // -0.13 
    steer_kd = -1.52121;  // -2.5
    steer_ki = -0.001;   // -0.002 
  }
  steer_pid.Init(steer_kp, steer_ki, steer_kd, 1, -1, use_twiddel);
    
  // Initialize the speed PID controller
  // these have been manually tuned by using the following steps
  // 1. Set all parameters to 0
  // 2. Increase only Kp and observe the speed
  // 3. Increase the value until the cars speed oscillates around the target 
  //    speed decrease if excessive oscillation
  // 4. Repeat step 3 until the speed is slightly oscillating about the  target speed
  // 5. Repeat steps 2 to 4 now with the derivative Kd parameter until the 
  //    osscilations subside/stop. The result may not be exactly the target speed, 
  //    but the goal is to stop the oscillations.
  // 6. Repeat steps 2 to 4 now with the integral Ki parameter until the speed 
  //    is the same as the target speed.
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
          if(steer_pid.use_twiddle){
            // run twiddle to find the best steering parameters
            twiddle(steer_pid, cte, ws);
            
            // twidle parameter check for DEBUG
            std::cout << "Kp: " << steer_pid.Kp << "  Ki: " << steer_pid.Ki << "  "
              "Kd: " << steer_pid.Kd << "  ct: " << steer_pid.current_twiddle << 
              "  bt: " << steer_pid.best_twiddle << "  cp " << steer_pid.current_param 
              << std::endl;
          }
          steer_pid.UpdateError(cte);
          double steer_value = steer_pid.output;
          
          // run the speed PID controller
          double desired_speed = 50; // mph
          double error = desired_speed - speed;
          speed_pid.UpdateError(error);
          double throttle_value = speed_pid.output;
          
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
