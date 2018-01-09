#include <fstream>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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

    //key for car of interest - this is used in dictionaries below
    int coi = -1;

    // car of interest (coi)
    Vehicle car(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy, "KL");

    h.onMessage(
            [&car, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](
                    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                    uWS::OpCode opCode) {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                // auto sdata = string(data).substr(0, length);
                // cout << sdata << endl;
                if (length && length > 2 && data[0] == '4' && data[1] == '2') {

                    auto s = hasData(data);

                    if (s != "") {
                        auto j = json::parse(s);

                        string event = j[0].get<string>();

                        if (event == "telemetry") {
                            // j[1] is the data JSON object

                            // dictionary with keys equal to the vehicle ids and values the vehicles that we receive information
                            // from sensor fusion including the car of interest (coi). The car of interest has a key of -1.
                            map<int, Vehicle> vehicles;

                            // dictionary with keys equal tp the vehicle ids and values the vector of vehicles each representing a
                            // hypothetical vehicle state in the prediction horizon.
                            map<int, std::vector<Vehicle> > predictions;

                            // Main car's localization Data
                            double car_x = j[1]["x"];
                            double car_y = j[1]["y"];
                            double car_s = j[1]["s"];
                            double car_d = j[1]["d"];
                            double car_yaw_deg = j[1]["yaw"];
                            // note that car of interest sensors report velocity in miles per hour
                            double car_vel_mph = j[1]["speed"];

                            // Previous path (x,y) given to the planning alg.
                            auto previous_path_x = j[1]["previous_path_x"];
                            auto previous_path_y = j[1]["previous_path_y"];

                            // Previous path's end-point s and d values
                            double end_path_s = j[1]["end_path_s"];
                            double end_path_d = j[1]["end_path_d"];

                            car.configure(car_x, car_y, car_s, car_d, car_yaw_deg, car_vel_mph);

                            // set the previous path data returned by the simulator
                            car.set_previous_path_data(previous_path_x, previous_path_y, end_path_s, end_path_d);

                            cout << "--------------------------------------------------------" << endl;
                            cout << -1 << " "
                                 << "State=" << car.state << " (x,y)=(" << car.x << "," << car.y << ")"
                                 << " (yaw_deg,v)=(" << car.yaw_deg << "," << car.v << ")"
                                 << " (s,d)=(" << car.s << " " << car.d << ")"
                                 << " lane=" << car.lane << endl;

                            // store in vehicles the coi. coi id is = -1
                            vehicles.insert(vehicles.end(), std::pair<int, Vehicle>(-1, car));

                            // Sensor fusion data, a list of all other cars on the same side of the road.
                            auto sensor_fusion = j[1]["sensor_fusion"];

                            json msgJson;

                            // instantiate other_car to keep the information for each other car seen
                            // note that the default constructor is used for other_cars as we dont need
                            // information such as waypoints etc.
                            Vehicle other_car;

                            // Loop over each other_car seen by sensor fusion
                            for (auto &i : sensor_fusion) {

                                // the corresponding sensor fusion localization data
                                int other_car_id = i[0];
                                double other_car_x = i[1];
                                double other_car_y = i[2];
                                double other_car_yaw_deg = 0.0;
                                double other_car_vx = i[3];
                                double other_car_vy = i[4];
                                double other_car_v = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);
                                double other_car_s = i[5];
                                double other_car_d = i[6];

                                // configure other_car using the sensor data
                                // note that sensors provide velocity in meters per second
                                other_car.configure(other_car_x, other_car_y, other_car_s, other_car_d,
                                                    other_car_yaw_deg,
                                                    other_car.mps2mph(other_car_v));

                                other_car.set_lane(other_car_d);

                                // print to the console the post configuration information
                                cout << other_car_id << " " << "(x,y)=(" << other_car.x << "," << other_car.y << ")"
                                     << "(yaw_deg,v)=(" << other_car.yaw_deg << "," << other_car.v << ")"
                                     << "(s,d)=(" << other_car.s <<  " " << other_car.d << ")"
                                     << "lane=" << other_car.lane << endl;

                                // sometimes we receive other_car data with negative d coordinate  - we ignore those
                                if (other_car.d > 0) {

                                    // store in vehicles each other_car object
                                    vehicles.insert(vehicles.end(), std::pair<int, Vehicle>(other_car_id, other_car));
                                }

                                // predict the trajectory for each of the other_car vehicles
                                // the prediction horizon is equal to the fraction (expressed 0.0-1.0) of the
                                // path planner trajectory or at the very least one time step beyond the current frame
                                vector<Vehicle> preds = other_car.generate_predictions(
                                        max(1, (int) (1.0 * car.horizon_points)));
                                predictions[other_car_id] = preds;
                            }

                            // given a set of predictions choose the next state
                            vector<vector<double> > trajectory = car.choose_next_state(predictions);

                            msgJson["next_x"] = trajectory[0];
                            msgJson["next_y"] = trajectory[1];

                            auto msg = "42[\"control\"," + msgJson.dump() + "]";

                            //this_thread::sleep_for(chrono::milliseconds(1000));
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                        }
                    } else {
                        // Manual driving
                        std::string msg = "42[\"manual\",{}]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }
            }

    );

// We don't need this since we're not using HTTP but if it's removed the
// program
// doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
// i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](
            uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" <<
                  std::endl;
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
