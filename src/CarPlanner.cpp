#include "../include/motion_planning/CarPlanner.h"
#include <math.h>

using namespace motion_planning;

/**
 * Private Functions
*/

// Determines if an object is in front of the car
// Returns true if the object is in front of the car
bool CarPlanner::is_in_front(const motion_planning::RoadObject& object) const {
    // Assuming the relative coords act as if the car is travelling towards positive x
    // Check to make sure object is within 2 meters of the car's y-coordinate
    return object.loc_x > 0 && abs(object.loc_y) < 2;
}

// Determines if an object is in the lane directly left of the car
// Returns true if the object is in lane directly left of the car
bool CarPlanner::is_directly_left(const motion_planning::RoadObject& object) const {
    // Max distance you can be is 6 assuming you are in center and they are on edge
    return object.loc_y > 0 && object.loc_y < 6;
}

// Determines if an object is in the lane directly right of the car
// Returns true if the object is in lane directly right of the car
bool CarPlanner::is_directly_right(const motion_planning::RoadObject& object) const {
    return object.loc_y < 0 && object.loc_y > -6;
}

RoadObject CarPlanner::get_lane_object() const {
    // Keep track of the closest object that is in front of the car
    // We can't simply return the first object we find in front of the car
    // Because a car might've changed lanes and be in between us
    RoadObject ret;
    double min_dist = std::numeric_limits<double>::max();
    ret.object_id = 0;

    for (auto const& it: objects_) {
        if (is_in_front(it.second)) {
            if (ret.object_id == 0) {
                ret = it.second;
            } else {
                double curr_dist = get_distance_to_object(it.second);
                if (curr_dist < min_dist) {
                    ret = it.second;
                    min_dist = curr_dist;
                }
            }
        }
    }

    return ret;
}

// Gets the distance from the car to object
// Returns the distance to the object
double CarPlanner::get_distance_to_object(const RoadObject& object) const {
    return sqrt(object.loc_x * object.loc_x + object.loc_y * object.loc_y);
}

// Determines if the distance is too small
// Returns true if the distance is less than the velocity squared
// over twice the stopping acceleration
bool CarPlanner::is_too_close(double distance) const {
    return distance < (autonomy_.vel * autonomy_.vel / (2 * MAX_DECEL));
}

// Gets the vehicle that is behind/equal with the car
// `left` specifies if we are looking in the left lane or right lane.
// Returns a RoadObject with object_id = 0 if no object is found
RoadObject CarPlanner::get_lag_vehicle(bool left) const {
    RoadObject ret;
    double min_dist = std::numeric_limits<double>::max();
    ret.object_id = 0;

    for (auto const& it : objects_) {
        if (is_directly_left(it.second) && it.second.loc_x < 2 && left) {
            if (ret.object_id == 0) {
                ret = it.second;
            } else {
                double curr_dist = get_distance_to_object(it.second);
                if (curr_dist < min_dist) {
                    ret = it.second;
                    min_dist = curr_dist;
                }
            }
        } else if (is_directly_right(it.second) && it.second.loc_x < 2 && !left) {
            if (ret.object_id == 0) {
                ret = it.second;
            } else {
                double curr_dist = get_distance_to_object(it.second);
                if (curr_dist < min_dist) {
                    ret = it.second;
                    min_dist = curr_dist;
                }
            }
        }
    }

    return ret;
}

// Gets the vehicle that is ahead/equal with the car
// `left` specifies if we are looking in the left lane or right lane.
// Returns a RoadObject with object_id = 0 if no object is found
RoadObject CarPlanner::get_ahead_vehicle(bool left) const {
    RoadObject ret;
    double min_dist = std::numeric_limits<double>::max();
    ret.object_id = 0;

    for (auto const& it : objects_) {
        if (is_directly_left(it.second) && it.second.loc_x > -2 && left) {
            if (ret.object_id == 0) {
                ret = it.second;
            } else {
                double curr_dist = get_distance_to_object(it.second);
                if (curr_dist < min_dist) {
                    ret = it.second;
                    min_dist = curr_dist;
                }
            }
        } else if (is_directly_right(it.second) && it.second.loc_x > -2 && !left) {
            if (ret.object_id == 0) {
                ret = it.second;
            } else {
                double curr_dist = get_distance_to_object(it.second);
                if (curr_dist < min_dist) {
                    ret = it.second;
                    min_dist = curr_dist;
                }
            }
        }
    }

    return ret;
}

// Gets the distance from the car to the lane given by `lane`
// Returns the distance to the lane
double CarPlanner::get_distance_to_lane(int lane) const {
    // Convert the current long and lat to meters relative to the starting location
    double x = 111139 * (autonomy_.loc_x - origin_x_);
    double y = 111139 * (autonomy_.loc_y - origin_y_);

    double lane_offset = left_lane_offset_ + lane * LANE_WIDTH;

    return abs(left_lane_x_ * x + left_lane_y_ * y + lane_offset);
}

void CarPlanner::keep_lane() {
    RoadObject lane_object = get_lane_object();
    if (lane_object.object_id != 0) {
        double dist_to_obj = get_distance_to_object(lane_object);
        if (is_too_close(dist_to_obj)) {
            state_ = State::stop;
        } else if (dist_to_obj < KEEP_LANE_DISTANCE_THRESHOLD) {
            state_ = State::prep_lane_change;
        }
    }
}

void CarPlanner::prep_lane_change() {
    RoadObject lane_object = get_lane_object();

    if (lane_object.object_id != 0) {
        double dist_to_obj = get_distance_to_object(lane_object);
        double safe_distance = 2 * autonomy_.vel;
        
        if (is_too_close(dist_to_obj)) {
            state_ = State::stop;
        } else if (dist_to_obj < safe_distance) {
            state_ = State::follow_leader;
        } else {
            // Determines if we've found a lane to go into
            bool good = false;
            // Check if there is a left lane
            if (current_lane_ > 0) {
                RoadObject left_lane_lagger = get_lag_vehicle(true);
                double lagger_dist = get_distance_to_object(left_lane_lagger);
                RoadObject left_lane_leader = get_ahead_vehicle(true);

                bool lagger_good = left_lane_lagger.object_id == 0 || lagger_dist > SAFE_LANE_CHANGE_DISTANCE;
                bool leader_good = left_lane_leader.object_id == 0 || left_lane_leader.vel > lane_object.vel * W_L;
                if (lagger_good && leader_good) {
                    good = true;
                    target_lane_ = current_lane_ - 1;
                    // Do math to figure out how far to move
                    state_ = State::change_left;
                }
            }
            // Check if there is a right lane
            if (!good && current_lane_ < NUM_LANES - 1) {
                RoadObject right_lane_object = get_lag_vehicle(false);
                double dist = get_distance_to_object(right_lane_object);
                RoadObject right_lane_leader = get_ahead_vehicle(false);

                bool lagger_good = right_lane_object.object_id == 0 || dist > SAFE_LANE_CHANGE_DISTANCE;
                bool leader_good = right_lane_leader.object_id == 0 || right_lane_leader.vel > lane_object.vel * W_R;
                if (lagger_good && leader_good) {
                    good = true;
                    target_lane_ = current_lane_ + 1;
                    // Do math to figure out how far to move
                    state_ = State::change_right;
                }
            }
        }

    } else {
        state_ = State::keep_lane;
    }
}

void CarPlanner::lane_change() {
    // Check if we've reached the target lane
    double dist_to_lane = get_distance_to_lane(target_lane_);

    if (dist_to_lane > D_MIN && dist_to_lane < D_MAX) {
        state_ = State::keep_lane;
    }
}

void CarPlanner::follow_leader() {
    RoadObject lane_object = get_lane_object();

    if (autonomy_.vel < FREE_FLOW_SPEED) {
        state_ = State::prep_lane_change;
    } else if (lane_object.object_id == 0) {
        state_ = State::keep_lane;
    }
}

void CarPlanner::stop() {
    RoadObject lane_object = get_lane_object();
    if (lane_object.object_id == 0) {
        state_ = State::keep_lane;
    } else {
        state_ = State::prep_lane_change;
    }
}

// Initializes the origin of the coordinates
void CarPlanner::intialize_origin(const Autonomy& autonomy) {
    origin_set = true;
    left_lane_x_ = cos(autonomy.heading + M_PI / 2);
    left_lane_y_ = sin(autonomy.heading + M_PI / 2);
    left_lane_offset_ = -LANE_WIDTH; // Center of left lane is 4 meters to the left of the car
    origin_x_ = autonomy.loc_x;
    origin_y_ = autonomy.loc_y;
}

/**
 * Public Functions
*/
CarPlanner::CarPlanner(): state_(State::keep_lane), objects_(), autonomy_() {

}

void CarPlanner::update(const RoadObject& object) {
    objects_[object.object_id] = object;
    // 0 is the object id reserved for many functions
    // as a "NULL" type of return value, so increment each given object id
    objects_[object.object_id].object_id++;
}

void CarPlanner::update(const Autonomy& autonomy) {
    if (!origin_set) {
        intialize_origin(autonomy);
    }
    autonomy_ = autonomy;
    autonomy_.vel *= 1000 / 3600;
}

State CarPlanner::get_state() const {
    return state_;
}

void CarPlanner::plan() {
    switch(state_) {
        case State::keep_lane:
            keep_lane();
            break;
        case State::change_left:
        case State::change_right:
            lane_change();
            break;
        case State::follow_leader:
            follow_leader();
            break;
        case State::stop:
            stop();
            break;
        case State::prep_lane_change:
            prep_lane_change();
            break;
        default:
            keep_lane();
            break;
    }
}