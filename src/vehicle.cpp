#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include "spline/src/spline.h"

Vehicle::Vehicle() {}

Vehicle::Vehicle(vector<double> map_x, vector<double> map_y, vector<double> map_s,
                 vector<double> map_dx, vector<double> map_dy, string car_state) {

    x = 0;
    y = 0;
    s = 0;
    d = 0;
    yaw_deg = 0;
    // the instantaneous velocity
    v = 0;
    state = car_state;
    map_waypoints_x = map_x;
    map_waypoints_y = map_y;
    map_waypoints_s = map_s;
    map_waypoints_dx = map_dx;
    map_waypoints_dy = map_dy;

    // the public variable a holds the instantaneous acceleration
    // a = 0.0;
    max_acceleration = 10.;
    lane_width = 4.0;

    vel_target = mph2mps(vel_target_mph);

    // configure the moving average filter for lane cost calculation
    MovingAverage lane_score_moving_average(lane_cost_moving_average_window);

    for (int i = 0; i < lanes_available; i++) {
        lane_cost_averagers.push_back(lane_score_moving_average);
        average_lane_costs.emplace_back(0.0);
    }
}

Vehicle::~Vehicle() {}


void Vehicle::set_previous_path_data(vector<double> previous_x, vector<double> previous_y, double end_s, double end_d) {

    // size of the previous path given to the planning alg will always be < 50
    // as the car simulator consumed a number > 1 points during a cycle.

    previous_path_x = previous_x;
    previous_path_y = previous_y;
    prev_path_size = (int) previous_path_x.size();

    end_path_s = end_s;
    end_path_d = end_d;


}

void Vehicle::set_lane(double d_coord) {

    lane = 0;

    if (0.0 <= d_coord && d_coord < lane_width) {
        lane = 0;
    } else if (lane_width <= d_coord && d_coord < 2.0 * lane_width) {
        lane = 1;
    } else if (2.0 * lane_width <= d_coord && d_coord < 3.0 * lane_width) {
        lane = 2;
    }
    else
        lane=999; // invalid lane translation from d coordinate
}


void Vehicle::configure(double car_x, double car_y, double car_s, double car_d, double car_yaw_deg, double car_vel_mph) {

    // Called after we receive the localization data to configure a vehicle.

    x = car_x;

    y = car_y;

    s = car_s;

    d = car_d;

    yaw_deg = car_yaw_deg;

    v = mph2mps(car_vel_mph);

    set_lane(d);
}

vector<string> Vehicle::successor_states() {

//    Provides the possible next states given the current state for the FSM
//    discussed in the course, with the exception that lane changes happen
//    instantaneously, so LCL and LCR can only transition back to KL.

    vector<string> states;

    // Note: If state is "LCL" or "LCR", then just return "KL"
    states.emplace_back("KL");

    if (state == "KL") {
        states.emplace_back("PLCL");
        states.emplace_back("PLCR");
    } else if (state == "PLCL") {
        // assumes that lane 0 is the leftmost lane
        if (lane != 0) {
            states.emplace_back("PLCL");
            states.emplace_back("LCL");
        }
    } else if (state == "PLCR") {
        // assumes that lanes_available-1 is the right most lane
        if (lane != lanes_available - 1) {
            states.emplace_back("PLCR");
            states.emplace_back("LCR");
        }
    }

    return states;
}

vector<string> Vehicle::reduced_successor_states() {

//    Provides the possible next states given the current state for the FSM
//    discussed in the course, with the exception that lane changes happen
//    instantaneously, so LCL and LCR can only transition back to KL.

    vector<string> states;

    // If state is "LCL" or "LCR", then just return "KL"
    states.emplace_back("KL");

    if (state == "KL") {
        if (lane != 0)
            states.emplace_back("LCL");
        if (lane != lanes_available - 1)
            states.emplace_back("LCR");
    }

    return states;
}

vector<vector<double> > Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {


//    INPUT: The predictions dictionary.
//    OUTPUT: The (lowest cost) trajectory for the vehicle corresponding to the next vehicle state.
//
//    1. successor_states() - Uses the current state to return a vector of possible successor states for the finite
//       state machine.
//    2. generate_trajectory(string state, terget lane, map<int, vector<Vehicle>> predictions) - Returns a vector of Vehicle objects
//       representing a vehicle trajectory, given a state, target lane and predictions. Note that trajectory vectors
//       might have size 0 if no possible trajectory exists for the state.
//    3. calculate_cost(target lane)

    vector<vector<double> > final_trajectory;

    // populates the lane occupancy vector for other lanes
    get_lanes_occupancy(predictions);

    // only consider states which can be reached from current FSM state.
    vector<string> possible_successor_states = reduced_successor_states();

    vector<int> lane_rankings;

    //set_lane(d);

    for (auto &state_iter : possible_successor_states) {

        int new_lane = min(max(0, lane + lane_direction[state_iter]), lanes_available - 1);

        double max_lane_velocity = get_max_lane_velocity(predictions, new_lane);

        // set the intended lane velocity for cost calculations
        intended_lane_velocity[new_lane] = max_lane_velocity;

        // generate a rough idea of what trajectory we would follow IF we chose this state.
        vector<vector<double> > traj = generate_trajectory(state_iter, new_lane, predictions);

        // calculate the "cost" associated with that trajectory.
        double trajectory_cost = calculate_cost(new_lane);

        // averaging window is a int parameter >=1 .
        lane_cost_averaging(new_lane, trajectory_cost);
    }

    // rank lanes according to average cost
    lane_rankings = lane_cost_ranking();

    cout << "--------------- COSTS -------------" << endl;
    for (auto l : lane_rankings)
        std::cout << l << ' ' << average_lane_costs[l] << " | ";
    cout << endl;
    cout << "-----------------------------------" << endl;

    // given the rankings which lane and associated state is the best
    int best_lane = decide_best_state_lane(lane_rankings);

    lane = best_lane;

    final_trajectory = generate_trajectory(state, best_lane, predictions);

    return final_trajectory;

}

int Vehicle::decide_best_state_lane(vector<int> lane_rankings) {

    // decide best lane policy i.e. the best lane is the lane with minimal
    // sliding window average  cost

    for (auto &l: lane_rankings) {
        if (lane_is_feasible(l))
        {
            cout << "BEST LANE = " << l << " with cost = " << average_lane_costs[l] << endl;
            return l;
        }
    }

    // if all lanes are not feasible e.g. when all lanes are occupied,
    // then just maintain the current lane and state
    return (lane);
}

bool Vehicle::lane_is_feasible(int new_lane){

    bool feasible = true;

    // new lane should be not occupied and should not be spanning more than 1 lanes
    if ( (abs(new_lane - lane) == 2) || (lane_occupancy[new_lane]) )
    {
        feasible = false;
    }

    // avoid changing lanes before the car reaches a threshold speed
    if ((new_lane != lane) && (v < mph2mps(feasible_lane_change_velocity_threshold_mph)))
    {
        feasible = false;
    }

    return feasible;
}

void Vehicle::lane_cost_averaging(int new_lane, double min_cost) {
    // averaging lane costs to avoid instantaneous lane changes
    // moving average filter is initialized in the constructor.
    average_lane_costs[new_lane] = lane_cost_averagers[new_lane].next(min_cost);

}

vector<int> Vehicle::lane_cost_ranking() {

    // rank the lanes according to their average lane costs
    // first element of ranked_lane_indeces is the lowest cost lane index
    std::vector<int> ranked_lane_indeces(average_lane_costs.size());
    std::size_t n(0);
    std::generate(std::begin(ranked_lane_indeces), std::end(ranked_lane_indeces), [&] { return n++; });

    std::sort(std::begin(ranked_lane_indeces),
              std::end(ranked_lane_indeces),
              [&](double i1, double i2) { return average_lane_costs[i1] < average_lane_costs[i2]; });

    return ranked_lane_indeces;

}

vector<vector<double> >
Vehicle::generate_trajectory(string successor_state, int new_lane, map<int, vector<Vehicle>> predictions) {

    // Given a possible next state, generate the appropriate trajectory to realize it.

    vector<vector<double> > trajectory;
    if (successor_state == "KL") {
        trajectory = keep_lane_trajectory(new_lane, predictions);
    } else if (successor_state == "LCL" || successor_state == "LCR") {
        trajectory = lane_change_trajectory(new_lane, predictions);
    } else if (successor_state == "PLCL" || successor_state == "PLCR") {
        trajectory = prep_lane_change_trajectory(new_lane, predictions);
    }

    return trajectory;
}

double Vehicle::get_max_lane_velocity(map<int, vector<Vehicle> > predictions, int new_lane) {

    // Gets maximum velocity allowed for a given lane given the presence of other vehicles.
    double max_lane_velocity;
    Vehicle vehicle_ahead;

    bool exists_vehicle_ahead = get_vehicle_ahead(predictions, vehicle_ahead, new_lane);

    if (exists_vehicle_ahead) {
        if ((vehicle_ahead.s > s) && (vehicle_ahead.s < s + vehicle_ahead_too_close_meters)) {

            max_lane_velocity = vehicle_ahead.v;
            cout << "Max velocity for lane " << new_lane << " is limited by vehicle ahead with velocity "
                 << max_lane_velocity << endl;

        } else
            max_lane_velocity = vel_target;

    } else {
        max_lane_velocity = vel_target;
        cout << "Max velocity for lane " << new_lane << " = " << max_lane_velocity << endl;
    }

    return max_lane_velocity;

}

vector<vector<double> > Vehicle::keep_lane_trajectory(int new_lane, map<int, vector<Vehicle>> predictions) {

    /// Generate a keep-lane trajectory.

    // find the best velocity to use
    double increment = 3.0 * max_acceleration * Ts;
    double decrement = 4.0 * max_acceleration * Ts;
    double velocity = v;
    if (v >= intended_lane_velocity[new_lane]) {

        // cout << "Decrement=" << increment << endl;
        velocity = v - decrement;

    } else if (v <= intended_lane_velocity[new_lane]) {

        // cout << "Increment=" << increment << endl;
        velocity = v + increment;

    }

    cout << "KL TRAJECTORY : lane " << new_lane << "  " << " Velocity = " << velocity << endl;
    vector<vector<double> > traj = generate_spline(new_lane, velocity);

    return traj;
}

vector<vector<double> > Vehicle::prep_lane_change_trajectory(int new_lane, map<int, vector<Vehicle>> predictions) {

    // Generate a trajectory preparing for a new_lane change.

    double max_lane_velocity = get_max_lane_velocity(move(predictions), new_lane);

    intended_lane_velocity[new_lane] = max_lane_velocity;

    vector<vector<double> > traj = generate_spline(new_lane, max_lane_velocity);

    return traj;
}

vector<vector<double> > Vehicle::lane_change_trajectory(int new_lane, map<int, vector<Vehicle>> predictions) {

    /// Generate a new_lane change trajectory.

    // find the best velocity to use
    double increment = 3.0 * max_acceleration * Ts;
    double decrement = 3.0 * max_acceleration * Ts;

    double velocity = v;
    if (v > intended_lane_velocity[new_lane]) {

        // cout << "Decrement=" << increment << endl;
        velocity = v - decrement;

    } else if (v <= intended_lane_velocity[new_lane]) {

        // cout << "Increment=" << increment << endl;
        velocity = v + increment;
    }

    cout << lane << " -> " << new_lane << " with velocity = " << velocity << endl;

    vector<vector<double> > traj = generate_spline(new_lane, velocity);

    return traj;
}

double Vehicle::position_at(int h) {
    // Ts is the sampling rate of the simulator
    // h is the prediction horizon
    // prediction is considering a=0 in this version. Acceleration measurements if provided can be used.
    double t = h * Ts;
    return s + v * t + a * t * t / 2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, Vehicle &rVehicle, int lane) {

    // Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    // rVehicle is updated if a vehicle is found.

    // detect the vehicle behind using the same detection distance as detecting the vehicle ahead
    double s_behind_detection = s - vehicle_ahead_detection_meters;
    bool found_vehicle = false;

    Vehicle temp_vehicle;
    for (auto &prediction : predictions) {

        // pick the first element of the predicted vehicle trajectory that represents the current frame
        temp_vehicle = prediction.second[0];
        if (temp_vehicle.lane == lane && temp_vehicle.s < s && temp_vehicle.s > s_behind_detection) {
            s_behind_detection = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

void Vehicle::get_lanes_occupancy(map<int, vector<Vehicle>> predictions) {

    // sets lane_occupancy vector if a vehicle is found in the other lanes that can cause collision
    // if the car changes lanes.

    lane_occupancy = {false, false, false};

    Vehicle temp_vehicle;
    for (auto &prediction : predictions) {

        // pick the first element of the predicted vehicle trajectory that represents the current frame
        temp_vehicle = prediction.second[0];
        temp_vehicle.set_lane(temp_vehicle.d);
        if ((temp_vehicle.s > s - preferred_buffer) && (temp_vehicle.s < s + preferred_buffer)) {
            lane_occupancy[temp_vehicle.lane] = true;
        }
    }

    cout << "Lane occupancy = " << lane_occupancy[0] << " " << lane_occupancy[1] << " " << lane_occupancy[2] << endl;

    if (lane_occupancy[lane] && temp_vehicle.s < s) {
        cout << "COLISION WARNING WITH VEHICLE COMMING FROM BEHIND ON LANE " << lane << endl;
    }
    else if (lane_occupancy[lane] && temp_vehicle.s > s) {
        cout << "COLISION WARNING WITH VEHICLE IN FRONT ON LANE " << lane << endl;
    }
}


bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, Vehicle &rVehicle, int new_lane) {

    // Returns a true if a vehicle, in the target lane, is found ahead of the current vehicle, false otherwise. The passed reference
    // rVehicle is updated if a vehicle is found.

    double s_ahead_detection = s + vehicle_ahead_detection_meters;
    bool found_vehicle = false;

    Vehicle temp_vehicle;
    for (auto &prediction : predictions) {

        // pick the last element of the predicted vehicle trajectory that represents the current frame
        temp_vehicle = prediction.second[0];

        //cout << temp_vehicle.s << " " << temp_vehicle.new_lane << endl;
        if (temp_vehicle.lane == new_lane && temp_vehicle.s > s && temp_vehicle.s < s_ahead_detection) {

            s_ahead_detection = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int prediction_horizon) {

//    Generates predictions for other vehicles on the road to be used in trajectory generation for the vehicle of interest.
//    Predictions are limited to only the s coordinate and velocity.
//    In this version the predicted lane is also the same i.e. we are myopic to predicted changes of the lane.

    vector<Vehicle> predictions;
    for (int i = 0; i < prediction_horizon; i++) {

        double next_s = position_at(i);

        double next_v_mps = 0;
        if (i < prediction_horizon - 1) {
            next_v_mps = (position_at(i + 1) - next_s) / Ts;
        }

        Vehicle predicted;

        // the prediction is limited to the s-coordinate and velocity only
        predicted.configure(0., 0., next_s, d, yaw_deg, mps2mph(next_v_mps));
        predictions.emplace_back(predicted);
    }
    return predictions;

}

double Vehicle::inefficiency_cost(const double new_lane_vel) {

    // Cost becomes lower for new lanes that offer higher velocity gains past a threshold.
    double cost = 0.0;
    if (new_lane_vel > v + velocity_lane_change_threshold) {
        cost = logistic(v - new_lane_vel);
    }
    return cost;
}

double Vehicle::occupant_experience_cost(const int new_lane) {

    double cost = 0.0;

    if ((lane == 0 && new_lane == 1) || (lane == 2 && new_lane == 1)) {
        // favour transitions into the center lane
        cost = logistic(-1.0);
    } else if ((lane == 0 && new_lane == 2) || (lane == 2 && new_lane == 0)) {
        // dont favor transitions that span 2 lanes
        cost = logistic(1.0);
    }
    else if ((lane == 1 && new_lane == 0) || (lane == 1 && new_lane == 2)) {
        // slightly penalize transitions from center lane i.e. introduce bias towards center lane.
        cost = logistic(0.2);
    }
    return cost;
}

double Vehicle::collision_cost(const int new_lane) {

    if (lane_occupancy[new_lane]) {
        return logistic(pow(10., 3.));
    } else {
        return 0.0;
    }
}

double Vehicle::calculate_cost(const int new_lane) {

    // Sum weighted cost functions to get total cost for trajectory.

    weighted_cost_functions weights;

    double cost = weights.efficiency_weight * inefficiency_cost(intended_lane_velocity[new_lane]);
    cost += weights.d_diff_weight * occupant_experience_cost(new_lane);
    cost += weights.collision_weight * collision_cost(new_lane);

    return cost;

}

vector<vector<double> > Vehicle::generate_spline(const int target_lane, const double velocity) {

    double car_lane_center = center_of_lane(target_lane, 4.0);

    if (prev_path_size > 0)
    {
        s = end_path_s;
    }

    // create a list of widely spaced (x,y) anchor points evenly spaced at 30m
    // these anchor points will be interpolated with a spline.
    vector<double> anchor_points_x;
    vector<double> anchor_points_y;

    // the vector of values returned by the planning alg.
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // state variables
    double pos_x;
    double pos_y;
    double yaw_rad;

    // cout << prev_path_size << " " << end_path_s << " " << s << endl;

    // if the previous path is almost empty i.e. typically when we just start out,
    if (prev_path_size < 2) {

        // the current state will be where the car is (current localization (x,y) location)
        // and with an angle tangent to the yaw_deg angle of the car.
        pos_x = x;
        pos_y = y;

        yaw_rad = deg2rad(yaw_deg);

        double previous_x = x - cos(yaw_rad);
        double previous_y = y - sin(yaw_rad);

        // append the previous coords and the current coords
        anchor_points_x.push_back(previous_x);
        anchor_points_x.push_back(pos_x);

        anchor_points_y.push_back(previous_y);
        anchor_points_y.push_back(pos_y);
    } else {
        // if we have received previous path data, the state will be at the previous path endpoint and
        // with an angle tangent to the angle determined by last two path points.
        pos_x = previous_path_x[prev_path_size - 1];
        pos_y = previous_path_y[prev_path_size - 1];

        double pos_x2 = previous_path_x[prev_path_size - 2];
        double pos_y2 = previous_path_y[prev_path_size - 2];
        yaw_rad = atan2(pos_y - pos_y2, pos_x - pos_x2);

        // append the previous state and the current state
        anchor_points_x.push_back(pos_x2);
        anchor_points_x.push_back(pos_x);

        anchor_points_y.push_back(pos_y2);
        anchor_points_y.push_back(pos_y);

    }

    // After the stitching to the previous_path data (the endpoint and the point before it)
    // we consider three target trajectory points into the future using the waypoint data.
    // The points are at a distance of 30m, 60m and 90m relative to the current car_s coordinate
    // We use the Frenet coordinates for these points as its easier to specify and we convert them to
    // global X, Y coordinates
    vector<double> next_point_30 = getXY(s + 30, car_lane_center, map_waypoints_s,
                                         map_waypoints_x, map_waypoints_y);
    vector<double> next_point_60 = getXY(s + 60, car_lane_center, map_waypoints_s,
                                         map_waypoints_x, map_waypoints_y);
    vector<double> next_point_90 = getXY(s + 90, car_lane_center, map_waypoints_s,
                                         map_waypoints_x, map_waypoints_y);

    anchor_points_x.push_back(next_point_30[0]);
    anchor_points_y.push_back(next_point_30[1]);
    anchor_points_x.push_back(next_point_60[0]);
    anchor_points_y.push_back(next_point_60[1]);
    anchor_points_x.push_back(next_point_90[0]);
    anchor_points_y.push_back(next_point_90[1]);

    // up to this point we have 5 anchor points specified and we need to interpolate between these 5 points

    // First, we need to change the coordinate system from global to local
    // with the current car_yaw to be 0 degrees.
    for (int i = 0; i < anchor_points_x.size(); i++) {
        double shift_x = anchor_points_x[i] - pos_x;
        double shift_y = anchor_points_y[i] - pos_y;

        anchor_points_x[i] = shift_x * cos(0 - yaw_rad) - shift_y * sin(0 - yaw_rad);
        anchor_points_y[i] = shift_x * sin(0 - yaw_rad) + shift_y * cos(0 - yaw_rad);

    }

    // we then create a spline object
    tk::spline spl;

    // add the anchor points to the spline
    spl.set_points(anchor_points_x, anchor_points_y);

    // The simulator returns the previous path data so that the planning alg can use them to create
    // smooth transitions.
    for (int i = 0; i < prev_path_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Every 20ms the car will go to the next interpolated point.
    // We want the car to hit a speed target after an Eucliean distance of vehicle_velocity_target_distance = 30m.
    // d = N_points * 0.02s * vel

    double target_x = vehicle_velocity_target_distance;
    double target_y = spl(target_x);
    double target_distance = sqrt(target_x * target_x + target_y * target_y);


    double x_add_on = 0.; // local coordinate system
    // Since we have already appended in all the previous path data (prev_path_size) we just
    // need to append the new data (horizon_points-prev_path_size) starting from the endpoint and spaced dist_inc appart.
    for (int i = 0; i < horizon_points - prev_path_size; i++) {
        double N = (target_distance / (Ts * velocity));
        double x_point = x_add_on + (target_x / N);
        double y_point = spl(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Change the coordinate system from local to global
        // with the current car_yaw from 0 to its original value.
        x_point = x_ref * cos(yaw_rad) - y_ref * sin(yaw_rad);
        y_point = x_ref * sin(yaw_rad) + y_ref * cos(yaw_rad);

        x_point += pos_x;
        y_point += pos_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
    vector<vector<double> > next_values;
    next_values.emplace_back(next_x_vals);
    next_values.emplace_back(next_y_vals);

    return next_values;
}

// Transform from Frenet s,d coordinates to Cartesian x,y
// not a linear transformation
vector<double> Vehicle::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x,
                              const vector<double> &maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % (int) maps_x.size();

    double heading_rad = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading_rad);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading_rad);

    double perp_heading_rad = heading_rad - pi() / 2;

    double x = seg_x + d * cos(perp_heading_rad);
    double y = seg_y + d * sin(perp_heading_rad);

    return {x, y};

}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// the important input parameters here are x, y and theta.
// map_x and map_y are considered given as constant vectors as they are read from a file.
vector<double>
Vehicle::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000. - maps_x[prev_wp];
    double center_y = 2000. - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};

}

// the closest waypoint in terms of Euclidean distance.
int Vehicle::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

// The difference between next waypoint and closest waypoint is that the next waypoint is the
// one that is in next waypoint in front of you even if you have just passed another closer waypoint.
int
Vehicle::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = min(2 * pi() - angle, angle);

    if (angle > pi() / 4) {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size()) {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}