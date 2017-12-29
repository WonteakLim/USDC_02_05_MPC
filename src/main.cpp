#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

Eigen::VectorXd global2local(double x_veh, double y_veh, double psi_veh, double x_pt, double y_pt)
{
    double X_veh = x_veh;
    double Y_veh = y_veh;
    double Psi_veh = psi_veh;

    double X_pt = x_pt;
    double Y_pt = y_pt;

    double x_diff = X_pt - X_veh;
    double y_diff = Y_pt - Y_veh;
    
    double x_local = x_diff*cos(Psi_veh) + y_diff*sin(Psi_veh);
    double y_local = -x_diff*sin(Psi_veh) + y_diff*cos(Psi_veh);

    Eigen::VectorXd pt_local = Eigen::VectorXd(2);

    pt_local << x_local, y_local;

    return pt_local;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
	  std::cout << "On Message" << std::endl;
      	  // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
	  double delta = j[1]["steering_angle"];
	  double a = j[1]["throttle"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // uinit conversion (mph -> m/s)
	  v = v * 0.44704;


          double steer_value;
          double throttle_value;

	  // Coordinate conversion (map->vehicle coordinate)	  
	  Eigen::VectorXd ptsx_local = Eigen::VectorXd( ptsx.size() );
	  Eigen::VectorXd ptsy_local = Eigen::VectorXd( ptsy.size() ); 

	  for( int i=0; i< (int)ptsx.size() ; i++ )
	  {
	      Eigen::VectorXd pt_local = global2local( px, py, psi, ptsx[i], ptsy[i]);

	      ptsx_local(i) = pt_local(0);
	      ptsy_local(i) = pt_local(1);
	  }
	  
	  auto coeffs = polyfit( ptsx_local, ptsy_local, 3);


	  // Curve fitting
	  double cte = polyeval( coeffs, 0);
	  double epsi = - atan( coeffs[1] );

	  // Vehicle state
	  const double dt = 0.1;
	  const double Lf = 2.67;
	  Eigen::VectorXd state = Eigen::VectorXd(6);
	  state << v*dt, 
		0, 
		v*(-delta)/Lf*dt, 
		v + a*dt, 
		cte + v*sin(epsi)*dt, 
		epsi + v*(-delta)/Lf*dt;

	  // MPC solver
	  vector<double> results = mpc.Solve( state, coeffs );
	  steer_value = results[0];
	  throttle_value = results[1];

	  std::cout << "End of calcuation" << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

	  cout << "Steer: " << steer_value * 180.0/pi();

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

	  vector<double> mpc_trj = mpc.getTrajectory();
	  int mpc_size = mpc_trj.size() / 2;
	  for( int i=0 ; i < mpc_size ; i++)
	  {	      
	      mpc_x_vals.push_back(  mpc_trj[i] );
	      mpc_y_vals.push_back(  mpc_trj[mpc_size+i] );
	  }

	  // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

	  for( int i=0; i< 20 ; i++)
	  {
	      double x_i = (double)i*5;
	      double y_i = polyeval( coeffs, x_i );

	      //Eigen::VectorXd pt_v = global2local( px, py, psi, x_i, y_i);

	      next_x_vals.push_back( x_i );
	      next_y_vals.push_back( y_i );
	  }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
