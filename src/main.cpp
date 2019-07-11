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


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
#ifdef _MSC_VER
              (uWS::WebSocket<uWS::SERVER> *ws, 
#else
	          (uWS::WebSocket<uWS::SERVER> ws,
#endif
			   char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
	  // configuration
	  int lane = 1; // target line
	  double targetSpeed = 49.8;
	  double sparsePointSpacing = 30.0;
	  int sparsePointNum = 3;
	  int desiredPatchLength = 50;
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
			auto prevSize = previous_path_x.size();
			//Desired car position spaced at 30m
			vector<double> sparsePosX;
			vector<double> sparsePosY;

			//car postion will be a new refference
			double refX;
			double refY;
			double refYaw;
			double prevRefX;
			double prevRefY;
			if (prevSize < 2)
			{
				// Not enougth points use current car possition and heading
				refX = car_x;
				refY = car_y;
				refYaw = deg2rad(car_yaw);
				prevRefX = refX - cos(refYaw);
				prevRefY = refY - sin(refYaw);
			}
			else
			{
				//Use last patch points as strating point
				refX = previous_path_x[prevSize - 1];
				refY = previous_path_y[prevSize - 1];
				prevRefX = previous_path_x[prevSize - 2];
				prevRefY = previous_path_y[prevSize - 2];
				refYaw = atan2(refY - prevRefY, refX - prevRefX);
			}
			sparsePosX.push_back(prevRefX);
			sparsePosX.push_back(refX);
			sparsePosY.push_back(prevRefY);
			sparsePosY.push_back(refY);
			// add 3 points apaced 30m appart in the middle of the road for the fit
			for (int i = 1; i <= sparsePointNum; i++)
			{
				vector<double> point = getXY(car_s + i * sparsePointSpacing, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				sparsePosX.push_back(point[0]);
				sparsePosY.push_back(point[1]);
			}
			// Shift and rotate sparse points to (0,0) and 0'C
			for (int i = 0; i < sparsePosX.size(); i++)
			{
				double distX = sparsePosX[i] - refX;
				double distY = sparsePosY[i] - refY;
				sparsePosX[i] = distX * cos(-refYaw) - distY * sin(-refYaw);
				sparsePosY[i] = distX * sin(-refYaw) + distY * cos(-refYaw);
			}
			// fit the spline 
			tk::spline s;

			
		  s.set_points(sparsePosX, sparsePosY);

		  // new path points including not used points from previous path
          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  for (int i = 0; i < prevSize; ++i)
		  {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
		  }
		  // Linearize spline on 30m
		  double targetX = 30.0;
		  double targetY = s(targetX);
		  double targetDist = sqrt(targetX * targetX + targetY * targetY);
		  // 0.44704 transformes miles/h to m/s
		  double Xinc = (targetX * targetSpeed * 0.44704 * 0.02) / targetDist;
		  for (int i = 1; i <= (desiredPatchLength - prevSize); ++i)
		  {
			  double xPoint = i * Xinc;
			  double yPoint = s(xPoint);
			  double xTemp = xPoint;
			  double yTemp = yPoint; 
			  // roteate it back to original refference 
			  xPoint = xTemp * cos(refYaw) - yTemp * sin(refYaw);
			  yPoint = xTemp * sin(refYaw) + yTemp * cos(refYaw);

			  xPoint += refX;
			  yPoint += refY;
			  next_x_vals.push_back(xPoint);
			  next_y_vals.push_back(yPoint);
		  }

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
		  /*
		  std::cout << "PrevSize: " << prevSize;
		  for (int i = 0; i < next_x_vals.size(); i++)
		  {
			  std::cout << "points " << i << "[" << next_x_vals[i] << ", " << next_y_vals[i] << "]" << std::endl;
			  if (i > 0)
			  {
				  double x = next_x_vals[i] - next_x_vals[i - 1];
				  double y = next_y_vals[i] - next_y_vals[i - 1];
				  std::cout << "speed: " << sqrt(x*x + y * y) / (0.02 * 0.44704) << std::endl;
			  }

		  }
		  */
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
#ifdef _MSC_VER
		  ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef _MSC_VER
		ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
      }
    }  // end websocket if
  }); // end h.onMessage
#ifdef _MSC_VER
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif
    std::cout << "Connected!!!" << std::endl;
  });
#ifdef _MSC_VER
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, 
#else
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws,
#endif
	                     int code, char *message, size_t length) {
#ifdef _MSC_VER
	ws->close();
#else
    ws.close();
#endif
    std::cout << "Disconnected" << std::endl;
  });
#ifdef _MSC_VER
  int port = 4567;
  auto host = "127.0.0.1";
  if (h.listen(host, port)) {
#else
  int port = 4567;
  if (h.listen(port)) {
#endif
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}