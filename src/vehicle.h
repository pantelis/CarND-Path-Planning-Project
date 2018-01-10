
#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include <queue>
#include "MovingAverage.h"

using namespace std;

/*
 * This is a modified version of the clas provided by one of the quizes. It changes several public variables
 * to make the class compatible with the simulator.
 */

class Vehicle {
public:

    double x;

    double y;

    double d;

    double s;

    double v;

    double yaw_deg;

    string state;

    std::queue<string> state_buffer;

    double a = 0.0;

    // velocity to target
    double vel_target_mph = 48.5;

    double vel_target = mph2mps(vel_target_mph);

    map<int, double> intended_lane_velocity;

    double state_cost_switch_threshold = 0.;

    // lane chnage opportunities with less velocity gain than this threshold
    // are not accounted in the cost function
    double velocity_lane_change_threshold = 0.0;

    vector<MovingAverage> lane_cost_averagers;
    vector<double> average_lane_costs;

    int lane_cost_moving_average_window = 1;

    int lanes_available = 3;

    double lane_width = 4.0;

    // this is the buffer between the car of interest and other cars (in number of points) - it impacts "keep lane" behavior.
    int preferred_buffer = 15;

    vector<bool> lane_occupancy;

    int vehicle_ahead_detection_meters = 30;

    int vehicle_ahead_too_close_meters = 30;

    double vehicle_velocity_target_distance = 30;

    int lane; // this represents the current lane and is set from the set_lane() method

    double max_acceleration;

    double Ts = 0.02;

    vector<double> previous_path_x;
    vector<double> previous_path_y;
    int prev_path_size;
    double end_path_s;
    double end_path_d;

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    int horizon_points = 50;

    map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};


    Vehicle();

    Vehicle(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
            vector<double> map_waypoints_dx, vector<double> map_waypoints_dy, string state="KL");

    virtual ~Vehicle();

    void configure(double x, double y, double s, double d, double car_yaw_deg, double vel_mph);

    void set_previous_path_data(vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);

    void set_lane(double d_coord);

    //void set_intended_lane(double d);

    vector<vector< double> > choose_next_state(map<int, vector<Vehicle>> predictions);

    // Full FSM  - not used in this implementation
    vector<string> successor_states();

    // Reduced FSM without prepare lane change states
    vector<string> reduced_successor_states();

    vector<vector<double> > generate_trajectory(string state, int new_lane, map<int, vector<Vehicle>> predictions);

    double get_lane_velocity(map<int, vector<Vehicle>> predictions, int new_lane);

    vector<vector<double> > keep_lane_trajectory(int new_lane, map<int, vector<Vehicle>> predictions);

    vector<vector<double> > lane_change_trajectory(int new_lane, map<int, vector<Vehicle>> predictions);

    vector<vector<double> > prep_lane_change_trajectory(int new_lane, map<int, vector<Vehicle>> predictions);


    double position_at(int t);

    bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, Vehicle & rVehicle, int new_lane);

    bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, Vehicle & rVehicle, int new_lane);

    void get_lanes_occupancy(map<int, vector<Vehicle>> predictions);

    vector<Vehicle> generate_predictions(int prediction_horizon = 2);

    vector<vector<double> > generate_spline(const int target_lane, const double velocity);

    /// Transform from Frenet s,d coordinates to Cartesian x,y - not a linear transformation
    vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

    /// Transform from Cartesian x,y, theta to Frenet s,d coordinates
    vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

    int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

    int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

    inline double distance(double x1, double y1, double x2, double y2){
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    inline double center_of_lane(int lane, double lane_width_meters) {
        // Returns the d coordinate that corresponds to the center of the input lane whose attribute is lane_width_meters
        // Assumes that lane 0 is starting at d=0;
        double d = (lane_width_meters / 2.) + lane_width_meters * lane;
        return d;
    }

    void lane_cost_averaging(int new_lane, double min_cost);

    // Cost related methods Functions
    vector<int> lane_cost_ranking();

    bool lane_is_feasible(int new_lane);

    int decide_best_state_lane(vector<int> lane_rankings);

    double calculate_cost(const int new_lane);

    double goal_distance_cost();

    double inefficiency_cost(const double new_lane_vel);

    double occupant_experience_cost(const int intended_lane);

    double collision_cost(const int new_lane);

    struct weighted_cost_functions {
        double time_diff_weight = 1;
        double s_diff_weight = 1;
        double d_diff_weight = 1;
        double efficiency_weight = 1;
        double max_jerk_weight = 1;
        double total_jerk_weight = 1;
        double collision_weight = 100;
        double buffer_weight = 1;
        double max_accel_weight = 1;
        double total_accel_weight = 1;
    };

    // helper methods
    /// Returns the M_PI constant
    inline constexpr double pi() const { return M_PI; }

    inline double deg2rad(double x) { return x * pi() / 180.; }
    inline double rad2deg(double x) { return x * 180. / pi(); }

    /// Converts miles per hours to meters per second
    inline double mph2mps(double miles_per_hour) { return miles_per_hour * 0.44704; }

    /// Converts meters per second to miles per hour
    inline double mps2mph(double meters_per_second) { return meters_per_second * 2.23694; }

    /// Logistic Function
    inline double logistic(double x) {
        //  A function that returns a value between 0 and 1 for x in the
        // range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

        return (2.0 / (1 + exp(-x))) - 1.0;
    }
};

#endif //PATH_PLANNING_VEHICLE_H
