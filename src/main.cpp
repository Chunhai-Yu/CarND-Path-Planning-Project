#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // initialize with lane 1
  int lane = 1;
  double ref_vel = 0;
  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // ================prediction start================
          int prev_size = previous_path_x.size();
          // if size of previous path data not 0, set car location is at end position of previous path
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          bool carAtNearfront = false;
          bool carAtNearleft = false;
          bool carAtNearright = false;
          // Use fusion data to predict if surrounding cars near the ego car and may block the lane change
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            
            double object_d = sensor_fusion[i][6];
            int object_lane = (int)(object_d/4);;
            // check if the car in front is too close
            double object_vx = sensor_fusion[i][3];
            double object_vy = sensor_fusion[i][4];
            double object_speed = sqrt(object_vx*object_vx + object_vy*object_vy);
            double object_s = sensor_fusion[i][5];
            object_s += ((double)prev_size * 0.02 * object_speed);
            if(object_lane == lane){
              if((object_s > car_s) && (object_s < car_s + 40)){
                carAtNearfront = true;
              }
            } else {
              if((object_lane == lane - 1) && (object_s>car_s && (object_s - car_s)<40 || object_s<car_s && (car_s - object_s )<40)|| lane == 0)
                carAtNearleft = true;
              if((object_lane == lane + 1) && (object_s>car_s && (object_s - car_s)<40 || object_s<car_s && (car_s - object_s )<40)|| lane == 2)
                carAtNearright = true;
              }
          }
          //================prediction end================
          std::cout<< "current state: " <<lane << "   " << ref_vel << std::endl;
          std::cout<< "predictions: " << carAtNearleft << carAtNearright<< carAtNearfront << std::endl;
          
          //================behavior planning start================
          vector<std::pair<int, double>> planned_goalVals;//(lane, velocity)
          if(carAtNearfront){
            planned_goalVals.push_back(std::make_pair(lane, ref_vel - 0.224));// 5m/(s^2) acceleration 5 * 0.02m/s=0.224mph
            if(!carAtNearleft)
              planned_goalVals.push_back(std::make_pair(std::max(lane - 1, 0), std::min(49.5, ref_vel + 0.224)));//49.5/2.24
            if(!carAtNearright)
              planned_goalVals.push_back(std::make_pair(std::min(lane + 1, 2), std::min(49.5, ref_vel + 0.224)));
          }
          if(!carAtNearfront){
            planned_goalVals.push_back(std::make_pair(lane, std::min(49.5, ref_vel + 0.224)));
          }
          //================behavior planning end================
          
          //================cost function start================
          double lowest_cost = 1.0;
          for(int i = 0; i < planned_goalVals.size(); i++){
            int goalLane = planned_goalVals[i].first;
            double goalVel = planned_goalVals[i].second;
            // consider cost of lane change and acceleration
            double cost_laneChange = std::abs(lane - goalLane)/2; 
            double cost_Acc = 0.0;
            if(std::abs((goalVel/2.24 - car_speed/2.24)/0.02) >9.5)
              cost_Acc = 1.0;
            else
              cost_Acc = std::abs((goalVel/2.24 - car_speed/2.24)/0.02)/9.5;
            //
            double total_cost=0.5*cost_laneChange+0.5*cost_Acc;
            if(total_cost <= lowest_cost){
              lowest_cost = total_cost;
              lane = goalLane;
              ref_vel = goalVel;
            }
          }
          //================cost function end================
          std::cout<< "planned_goalVals size:<<<<<<<<<< " << planned_goalVals.size() << std::endl;
          std::cout<< "best planned_goalVal choice:  " << lane << "     " << ref_vel << std::endl;
          
          //================trajectory generation start================
          vector<double> points_x;
          vector<double> points_y;
          double pos_x = car_x;
          double pos_y = car_y;
          double angle = deg2rad(car_yaw);

          // reuse two previous points
          std::cout<< "prev_size size: " << prev_size << std::endl;
          if ( prev_size < 2 ) {   
            points_x.push_back(car_x - cos(car_yaw));
            points_x.push_back(car_x);    
            points_y.push_back(car_y - sin(car_yaw));
            points_y.push_back(car_y);
          }else{
            pos_x = previous_path_x[prev_size - 1];
            pos_y = previous_path_y[prev_size - 1];
            double pos_x2 = previous_path_x[prev_size - 2];
            double pos_y2 = previous_path_y[prev_size - 2];
            angle = atan2(pos_y-pos_y2, pos_x-pos_x2);
            points_x.push_back(pos_x2);
            points_x.push_back(pos_x);
            points_y.push_back(pos_y2);
            points_y.push_back(pos_y); 
          }
          // in Frenet add evenly 30m points ahead of the staring point
          for(int i = 1; i <4; i++){
            vector<double> next_point = getXY(car_s + 30*i, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            points_x.push_back(next_point[0]);
            points_y.push_back(next_point[1]);
          }
          // transform to car coordinates
          for(int i=0; i <points_x.size(); ++i){
            double shift_x = points_x[i] - pos_x;
            double shift_y = points_y[i] - pos_y;
            points_x[i] = (shift_x*cos(-angle) - shift_y*sin(-angle));
            points_y[i] = (shift_x*sin(-angle) + shift_y*cos(-angle));
          }
          
          // creat spline
          tk::spline s;
          std::cout<< "points_x size: " << points_x.size() << std::endl;
          for(int index=0;index<points_x.size();index++)
          {
            std::cout<< "points_x: " << points_x.at(index) << std::endl;
          }
          s.set_points(points_x, points_y);
          // Using information from the previous path ensures that there is a smooth transition from cycle to cycle
          for ( int i = 0; i < prev_size; i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0;
          
          for( int i = 1; i < 50 - prev_size; i++ ) {
            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;
            x_point = x_ref * cos(angle) - y_ref * sin(angle);
            y_point = x_ref * sin(angle) + y_ref * cos(angle);
            x_point += pos_x;
            y_point += pos_y;
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          //================trajectory generation end================


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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