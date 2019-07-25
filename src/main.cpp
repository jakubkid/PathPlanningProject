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


//#define PRINT_DEBUG             // uncomment to print debug info
#define LINE_NUM 3                // Number of lines
#define LINE_WIDTH 4.0            // Line witdth in m
#define SAMPLING_INTERVAL_S 0.02  // sampling interval in s
#define MAX_SPEED_MS 22.3         // maximum speed 22.3m/s ~50mph
#define MAX_ACC_MS2 9.5           // Maximum allowed acceleration in m/s2
#define CONF_ACC_MS2 6.0          // Max conftorable acceleration in m/s2
#define MAX_JERK_MS3 9.5           // Maximum allowed jerk in m/s3
#define CONF_JERK_MS3 6.0         // Max conftorable jerk in m/s3
#define SPEED_HIST_MS 0.2         // speed histeresis to avoid constant speed changes  in m/s
#define SPARSE_POINT_SPACING 30.0 // Spacing of sparse points in m
#define SPARSE_POINT_NUM 3        // number of Sparse points
#define DESIRED_PATCH_LEGTH 30    // number of points in patch for simulator
#define CHANGE_MIN_DIST 5.0       // minimum distance to the car in line change in m
#define CAR_MIN_DIST 15.0         // minimum distance to the car in m
#define CAR_FOLLOW_DIST 20.0      // distance where we should follow in m
#define CAR_BREAK_DIST 25.0       // Distance where we should start breaking in m
#define CAR_AVOID_DIST 40.0       // Distance where we should start overtaking the car in m
#define CAR_OVERTAKE_DIST 45.0    // Distance needed to overtake another car in m
#define CAR_IGNORE_DIST 200.0     // Distance of ignored cars because detection is nor reliable in m
#define SPEED_THRESHOLD_MS 2.0    // Threshold when it is worth to change the line in m/s
#define TIME_FOR_LINE_CHANGE_S 5  // Minimum time needed for line change in s


//check if it is safe to change the line
bool checkIfChangeIsSafe(int curLine, int destLine,double carSpeedMs,
	                     bool *trailingCarFound, double *trailingCarSpeed, double *trailingCarDist,
					     bool *procCarFound, double *procCarSpeed, double *procCarDist)
{
	if (curLine == destLine)
	{
		return true;
	}
	int inc = curLine < destLine ? 1 : -1;
	destLine += inc;
	for (int checkLine = curLine + inc; checkLine != destLine; checkLine += inc)
	{
		double trailingSpeedDiff = trailingCarSpeed[checkLine] - carSpeedMs;
		// Check trailling car
		if (trailingCarFound[checkLine] &&
		   ((trailingCarDist[checkLine] < CHANGE_MIN_DIST) || (trailingSpeedDiff > 0 && ((trailingCarDist[checkLine] / trailingSpeedDiff) < TIME_FOR_LINE_CHANGE_S))))
		{
#ifdef PRINT_DEBUG
			std::cout << "Trail car on " << checkLine << " found " << curLine << "->" << (destLine - inc) << " rejected" << std::endl;
			std::cout << "trailingCarFoundLine[checkLine] " << trailingCarFound[checkLine] << " speedDiff " << trailingSpeedDiff << " trailingCarDist[checkLine] " << trailingCarDist[checkLine] <<  std::endl;
#endif
			return false;
		}
		double procSpeedDiff = carSpeedMs - procCarSpeed[checkLine];
		//check leading car
		if (procCarFound[checkLine] &&
		   ((procCarDist[checkLine] < CHANGE_MIN_DIST) || (procSpeedDiff > 0 && ((procCarDist[checkLine] / procSpeedDiff) < TIME_FOR_LINE_CHANGE_S))))
		{
#ifdef PRINT_DEBUG
			std::cout << "Lead car on " << checkLine << " found " << curLine << "->" << (destLine - inc) << " rejected" << std::endl;
			std::cout << "procCarFoundLine[checkLine] "<< procCarFound[checkLine] << " procSpeedDiff " << procSpeedDiff << " procCarDist[checkLine] " << procCarDist[checkLine] << std::endl;
#endif
			return false;
		}
	}
	return true;
}

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
			auto prevSize = previous_path_x.size();
			json msgJson;
			double ourS; // Where we are going to be
			double targetSpeed = MAX_SPEED_MS; // target speed
			int currentLine = (int)(car_d / LINE_WIDTH); // current line
			int targetLine = currentLine; // target line (right when no cars detected)
			bool emergencyBreak = false; // break emergency
			double carSpeedMs = car_speed * 0.44704;
			if (prevSize > 0)
			{
				ourS = end_path_s;
			}
			else
			{
				ourS = car_s;
			}
/************************************************************************************************/
// Find proceeding and trailing car on each line
/************************************************************************************************/
			double procCarSpeed[LINE_NUM];                // speed of car infront per line
			double trailingCarSpeed[LINE_NUM];            // speed of trailing car per line
			double procCarDist[LINE_NUM];                 // Distance to proceeding car per line
			double trailingCarDist[LINE_NUM];             // DIstance to trailing car per line
			bool procCarFound[LINE_NUM] = { false };      // Car found in front per line
			bool trailingCarFound[LINE_NUM] = { false };  // Car found in the back per line

			for (int i = 0; i < sensor_fusion.size(); i++)
			{
				// check if car is in my lane
				double d = sensor_fusion[i][6];
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double checkCarSpeed = sqrt(vx*vx + vy * vy);
				double checkCarS = sensor_fusion[i][5];
				int checkCarLaneNum = (int)(d / LINE_WIDTH);
			    // project the car possition to the end of out path 
				checkCarS += prevSize * checkCarSpeed * SAMPLING_INTERVAL_S;
				double carDist = (checkCarS - ourS);

				if (carDist >= 0)
				{
					if (!procCarFound[checkCarLaneNum] || (carDist < procCarDist[checkCarLaneNum]))
					{
						procCarDist[checkCarLaneNum] = carDist;
						procCarSpeed[checkCarLaneNum] = checkCarSpeed > MAX_SPEED_MS ? MAX_SPEED_MS : checkCarSpeed;
						procCarFound[checkCarLaneNum] = true;
					}
				}
				else
				{
					carDist = -carDist;
					if (!trailingCarFound[checkCarLaneNum] || (carDist < trailingCarDist[checkCarLaneNum]))
					{
						trailingCarDist[checkCarLaneNum] = carDist;
						trailingCarSpeed[checkCarLaneNum] = checkCarSpeed;
						trailingCarFound[checkCarLaneNum] = true;
					}
				}
			}
/************************************************************************************************/
//Calculate optimum line
/************************************************************************************************/
			int bestLine = -1;
			// Find empty lines or lines with speeding cars or too far
			for (int checkLine = LINE_NUM - 1; checkLine >= 0; checkLine--)
			{
				if (!procCarFound[checkLine] || procCarSpeed[checkLine] >= MAX_SPEED_MS || procCarDist[checkLine] > CAR_IGNORE_DIST)
				{
					if (checkIfChangeIsSafe(currentLine, checkLine, carSpeedMs,
						trailingCarFound, trailingCarSpeed, trailingCarDist,
						procCarFound, procCarSpeed, procCarDist))
					{
						bestLine = checkLine;
						break;
					}
				}
			}
			// Find with car furtherst away and provided that it is further than ignore distance
			if (bestLine < 0)
			{
				double maxLineDist = 0;
				for (int checkLine = LINE_NUM - 1; checkLine >= 0; checkLine--)
				{
					if (procCarDist[checkLine] > CAR_OVERTAKE_DIST && procCarDist[checkLine] > maxLineDist)
					{
						if (checkIfChangeIsSafe(currentLine, checkLine, carSpeedMs,
							trailingCarFound, trailingCarSpeed, trailingCarDist,
							procCarFound, procCarSpeed, procCarDist))
						{
							maxLineDist = procCarDist[checkLine];
							bestLine = checkLine;
						}
					}
				}
			}
			// Find fastest line or line with enougth space to overtake
			if (bestLine < 0)
			{
				double maxLineSpeed = 0;
				for (int checkLine = LINE_NUM - 1; checkLine >= 0; checkLine--)
				{
					if (procCarSpeed[checkLine] > maxLineSpeed + SPEED_THRESHOLD_MS)
					{
						if (checkIfChangeIsSafe(currentLine, checkLine, carSpeedMs,
							trailingCarFound, trailingCarSpeed, trailingCarDist,
							procCarFound, procCarSpeed, procCarDist))
						{
							maxLineSpeed = procCarSpeed[checkLine];
							bestLine = checkLine;
						}
					}
				}
			}
			if (bestLine >= 0 && bestLine != currentLine)
			{
				// don't change more than 1 line at the time 
				int inc = currentLine < bestLine ? 1 : -1;
				targetLine = currentLine+inc;
			}
			else
			{
				targetLine = currentLine;
			}
/************************************************************************************************/
//Adapt speed to avoid colisions
/************************************************************************************************/
			if (procCarFound[currentLine] )
			{
				if (procCarDist[currentLine] < CAR_MIN_DIST)
				{
					targetSpeed = 0.9 * procCarSpeed[currentLine] - SPEED_HIST_MS;
				}
				else if(currentLine == targetLine)
				{
					if (procCarDist[currentLine] < CAR_FOLLOW_DIST)
					{
						targetSpeed = procCarSpeed[currentLine];
					}
					else if (procCarDist[currentLine] < CAR_BREAK_DIST)
					{
						targetSpeed = procCarSpeed[currentLine] + SPEED_HIST_MS;
					}
				}
				if (procCarDist[currentLine] <= CHANGE_MIN_DIST)
				{
					emergencyBreak = true;
				}
			}
			// check for collision on target line
			double speedTargetLine = -1;
			if (currentLine != targetLine && procCarFound[targetLine])
			{
				if (procCarDist[targetLine] < CAR_MIN_DIST)
				{
					speedTargetLine = 0.9 * procCarSpeed[targetLine] - SPEED_HIST_MS;
				}
				else if (procCarDist[targetLine] < CAR_FOLLOW_DIST)
				{
					speedTargetLine = procCarSpeed[targetLine];
				}
				else if (procCarDist[targetLine] < CAR_BREAK_DIST)
				{
					speedTargetLine = procCarSpeed[targetLine] + SPEED_HIST_MS;
				}
				if (procCarDist[targetLine] <= CHANGE_MIN_DIST)
				{
					emergencyBreak = true;
				}
			}
			// only update targetSpeed if needed and speed on target line is lower
			if (speedTargetLine > 0 && speedTargetLine < targetSpeed)
			{
				targetSpeed = speedTargetLine;
			}

/************************************************************************************************/
// Calculate desired patch based on desired line and speed
/************************************************************************************************/
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
			vector<double> lastPointFren = getFrenet(refX, refY, refYaw, map_waypoints_x, map_waypoints_y);
			double lastPointD = lastPointFren[1];
			//std::cout << "lastPointD: " << lastPointD << " car_d: " << car_d  << std::endl;
			for (int i = 1; i <= SPARSE_POINT_NUM; i++)
			{	
				// Don't turn when emergency braking is happening
				if (!emergencyBreak)
				{
					double targetLineD = (LINE_WIDTH / 2 + LINE_WIDTH * targetLine);
					if (abs(targetLineD - lastPointD) > (LINE_WIDTH / 2))
					{
						if (targetLineD > lastPointD)
						{
							targetLineD = lastPointD + (LINE_WIDTH / 2);
						}
						else
						{
							targetLineD = lastPointD - (LINE_WIDTH / 2);
						}
					}
					lastPointD = targetLineD;
				}
				vector<double> point = getXY(ourS + i * SPARSE_POINT_SPACING, lastPointD, map_waypoints_s, map_waypoints_x, map_waypoints_y);
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
			double xPointPrev = 0;
			static double curSpeed = 0;
			static double curAcc = 0;
			double maxAcc;
			double accInc;
			if (next_x_vals.size() == 0)
			{
				curSpeed = car_speed / 2.23694; // car speed converted to m/s
				curAcc = 0;
			}
			
			if (emergencyBreak)
			{
				maxAcc = MAX_ACC_MS2;
				accInc = MAX_JERK_MS3 * SAMPLING_INTERVAL_S;
				//#ifdef PRINT_DEBUG
				std::cout << "Emergency break " << std::endl;
				//#endif
			}
			else
			{
				maxAcc = CONF_ACC_MS2;
				accInc = CONF_JERK_MS3 * SAMPLING_INTERVAL_S;
			}
			for (int i = 1; i <= (DESIRED_PATCH_LEGTH - prevSize); ++i)
			{	
				double speedDiff = targetSpeed - curSpeed;
				if (abs(speedDiff) > SPEED_HIST_MS && abs(speedDiff) > maxAcc*SAMPLING_INTERVAL_S)
				{
					if (speedDiff < (curAcc*curAcc / CONF_JERK_MS3))
					{
						// start slowing accelertion down
						if (curAcc > 0)
						{
							curAcc -= accInc;
						}
						else
						{
							curAcc += accInc;
						}
					}
					if (speedDiff > 0)
					{
						if (curAcc < (maxAcc - accInc))
						{
							curAcc += accInc;
						}
						else
						{
							curAcc = maxAcc;
						}
					}
					else
					{
						if (curAcc > -(maxAcc - accInc))
						{
							curAcc -= accInc;
						}
						else
						{
							curAcc = -maxAcc;
						}
					}
					curSpeed += curAcc * SAMPLING_INTERVAL_S;
				}
				else
				{
					curSpeed = targetSpeed;
				}
				//std::cout << "curSpeed: " << curSpeed<< " curAcc: "<< curAcc << " accInc: " <<accInc<< std::endl;
				double Xinc = (targetX * curSpeed * SAMPLING_INTERVAL_S) / targetDist;

				xPointPrev = xPointPrev + Xinc;
				double xPoint = xPointPrev;
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