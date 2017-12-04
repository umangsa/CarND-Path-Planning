#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


bool is_lane_available(int lane, vector <vector <double>> sensor_fusion, double prev_size, double car_s, double &gap_ahead, double &gap_behind, double &speed){
  gap_ahead = 9999;
  gap_behind = 9999;
  double safe_gap_ahead = 30;
  double safe_gap_behind = 5;

  for(int i = 0; i < sensor_fusion.size(); i++) {
    float d = sensor_fusion[i][6];
    
    if(d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
      // car in same lane
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += (double) prev_size * 0.02 * check_speed;
      if(check_car_s > car_s) {
        double gap = check_car_s - car_s;
        if(gap < gap_ahead){
          gap_ahead = gap;
          speed = check_speed;
        }
      }
      else if(check_car_s < car_s) {
        double gap = car_s - check_car_s;
        if(gap < gap_behind)
          gap_behind = gap;
      }
      else {
        gap_ahead = 0;
        gap_behind = 0;
      }
    }
  }

  if((gap_behind >= safe_gap_behind) && (gap_ahead >= safe_gap_ahead))
    return true;
  return false;
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  int lane = 1;
  double ref_vel = 0.224;
  double target_velocity = 49.5;

  typedef enum BP_STATES {
    KEEP_LANE = 0,
    PREPARE_LANE_CHANGE_LEFT,
    PREPARE_LANE_CHANGE_RIGHT,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT
  } BP_STATES;

  BP_STATES car_state = KEEP_LANE;



  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &car_state, &target_velocity](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    // int lane = 1;
    // double ref_vel = 49.5;

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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_x.size();

            if(prev_size > 0) {
              car_s = end_path_s;
            }

            bool too_close = false;
            // double gap_ahead = 0;
            // double gap_behind = 0;

            bool right_lane_avail = false;
            bool left_lane_avail = false;
            double right_gap_ahead = 0;
            double left_gap_ahead = 0;
            double right_gap_behind = 0;
            double left_gap_behind = 0;
            double left_speed = 0;
            double right_speed = 0;
            double check_speed = 0;


            switch(car_state) {
              case KEEP_LANE:
                // avoid collision with car in front
                for(int i = 0; i < sensor_fusion.size(); i++) {
                  float d = sensor_fusion[i][6];
                  if(d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
                    // car in same lane
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];

                    check_car_s += (double) prev_size * 0.02 * check_speed;
                    if((check_car_s > car_s) && (check_car_s - car_s < 30)) {
                      too_close = true;
                    }

                  }
                }

                if(too_close) {
                  target_velocity = check_speed;
                }
                else if(ref_vel <= check_speed) {
                  ref_vel += 0.224;
                }

                if(too_close){
                  // change lane to either left or right depending on current lane
                  switch(lane) {
                    case 0:
                      if(is_lane_available(lane + 1, sensor_fusion, prev_size, car_s, right_gap_ahead, right_gap_behind, right_speed)) {
                        car_state = LANE_CHANGE_RIGHT;
                        // lane++;
                      }
                      // check_gap(lane + 1, gap_ahead, gap_behind, sensor_fusion, prev_size, car_s);
                      // if((gap_ahead > safe_gap_ahead) && (gap_behind > safe_gap_behind))
                      //   lane++;
                      else
                        ref_vel -= 0.224;
                      break;

                    case 1:

                      right_lane_avail = is_lane_available(lane + 1, sensor_fusion, prev_size, car_s, right_gap_ahead, right_gap_behind, right_speed);
                      left_lane_avail = is_lane_available(lane - 1, sensor_fusion, prev_size, car_s, left_gap_ahead, left_gap_behind, left_speed);

                      if(right_lane_avail && left_lane_avail) {
                        if(right_gap_ahead > left_gap_ahead)
                          car_state = LANE_CHANGE_RIGHT;
                        else if (right_gap_ahead < left_gap_ahead)
                          car_state = LANE_CHANGE_LEFT;
                        else {
                          //todo - check on which lane is faster
                          if(left_speed > right_speed)
                            car_state = LANE_CHANGE_LEFT;
                          else if(right_speed >= left_speed)  
                            car_state = LANE_CHANGE_RIGHT;
                        }
                      }
                      else if(right_lane_avail)
                        car_state = LANE_CHANGE_RIGHT;
                      else if(left_lane_avail)
                        car_state = LANE_CHANGE_LEFT;
                      else
                        ref_vel -= 0.224;
                      break;

                    case 2:
                      if(is_lane_available(lane - 1, sensor_fusion, prev_size, car_s, left_gap_ahead, left_gap_behind, left_speed)) {
                        car_state = LANE_CHANGE_LEFT;
                      }
                      else
                        ref_vel -= 0.224;
                      break;

                    default:
                      std::cout << "Invalid lane number " << lane << std::endl;
                  }
                }
                else if(ref_vel <= 49.5) {
                  ref_vel += 0.224;
                }
                break;
                // -- end of KEEP_LANE

              case LANE_CHANGE_RIGHT:
                lane++;
                car_state = KEEP_LANE;
                target_velocity = 49.5;
                too_close = false;
                break;
                // -- end of LANE_CHANGE_RIGHT

              case LANE_CHANGE_LEFT:
                lane--;
                car_state = KEEP_LANE;
                target_velocity = 49.5;
                too_close = false;
                break;
                // -- end of LANE_CHANGE_LEFT

              default:
                std::cout << "Invalid state " << car_state << std::endl;
              }

              vector<double> ptsx;
              vector<double> ptsy;

              // remember the reference of the car
              double ref_x = car_x;
              double ref_y = car_y;
              double ref_yaw = deg2rad(car_yaw);

              // if previous path is almost empty, current position will help to create a path
              if(prev_size < 2) {
                double previous_car_x = car_x - cos(car_yaw);
                double previous_car_y = car_y - sin(car_yaw);

                ptsx.push_back(previous_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(previous_car_y);
                ptsy.push_back(car_y);
              }
              else {
                // we have enough points from the real path
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);

                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
              }

              // In Frenet add points spaced at 30 m ahead
              vector <double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector <double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector <double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

              ptsx.push_back(next_wp0[0]);
              ptsx.push_back(next_wp1[0]);
              ptsx.push_back(next_wp2[0]);

              ptsy.push_back(next_wp0[1]);
              ptsy.push_back(next_wp1[1]);
              ptsy.push_back(next_wp2[1]);


            	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
              // double dist_inc = 0.4;
              for(int i = 0; i < ptsx.size(); i++) {
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;

                //Transform points to car co ordinates
                ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
              }



              // Co ordinates to be used in the planner
              vector<double> next_x_vals;
              vector<double> next_y_vals;


              // Add previous points. This will help smoothen transitions from old path to the new path we want to plan
              for(int i = 0; i < previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

              tk::spline s;
              s.set_points(ptsx, ptsy);
              // generate points along the spline so that we get the desired speed
              // desired lateral length of spline = number of points * sampling freq * velocity of car
              double target_x = 30.0; // desired distance on the spline
              double target_y = s(target_x);
              double target_distance = sqrt(pow(target_x, 2) + pow(target_y, 2));
              double x_add_on = 0;

              for(int i = 0; i < 50 - previous_path_x.size(); i++) {
                double N = (target_distance / (0.02 * ref_vel / 2.24)); // 2.24 to convert to meter per sec
                double x_point = x_add_on + target_x / N;
                double y_point = s(x_point);
                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                // rotate back to global co ordinates
                x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw) ;
                y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw) ;

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);

              }

            



            json msgJson;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
