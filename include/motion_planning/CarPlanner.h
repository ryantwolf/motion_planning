#pragma once
#include <motion_planning/RoadObject.h>
#include <motion_planning/Autonomy.h>
#include <vector>

enum State { keep_lane, change_left, change_right, follow_leader, stop, prep_lane_change };

class CarPlanner {
    private:
        const double KEEP_LANE_DISTANCE_THRESHOLD = 60.0;
        const int NUM_LANES = 3;
        const double LANE_WIDTH = 4.0;
        const double SAFE_LANE_CHANGE_DISTANCE = 15.0;
        const double W_R = 1.0;
        const double W_L = 1.0;
        const double D_MIN = 0.3;
        const double D_MAX = 4.0;
        const double FREE_FLOW_SPEED = 27;
        const double MAX_DECEL = 1;


        State state_;
        // Lane number from 0-2, where 0 is the left lane and 2 is the right lane
        int current_lane_ = 1, target_lane_ = 1;
        bool origin_set = false;
        // Parameters for determining an equation for the line that denotes the left lane
        double left_lane_x_ = 0, left_lane_y_ = 0, left_lane_offset_ = -LANE_WIDTH;
        // The longitude and latitude of the car's first recorded position
        double origin_x_ = 0, origin_y_ = 0;
        // The car's current information
        motion_planning::Autonomy autonomy_;
        // List of all objects on the road, indexed by their ID
        std::map<uint8_t, motion_planning::RoadObject> objects_;

        // Determines if an object is in front of the car
        // Returns true if the object is in front of the car
        bool is_in_front(const motion_planning::RoadObject& object) const;

        // Determines if an object is in the lane directly left of the car
        // Returns true if the object is in lane directly left of the car
        bool is_directly_left(const motion_planning::RoadObject& object) const;

        // Determines if an object is in the lane directly right of the car
        // Returns true if the object is in lane directly right of the car
        bool is_directly_right(const motion_planning::RoadObject& object) const;

        // Gets the closest object in front of the car
        // Returns a RoadObject with object_id = 0 if no object is found
        motion_planning::RoadObject get_lane_object() const;

        // Gets the distance from the car to object
        // Returns the distance to the object
        double get_distance_to_object(const motion_planning::RoadObject& object) const;

        // Determines if the distance is too small
        // Returns true if the distance is less than the velocity squared over twice the acceleration
        bool is_too_close(double distance) const;

        // Gets the vehicle that is behind/equal with the car
        // `left` specifies if we are looking in the left lane or right lane.
        // Returns a RoadObject with object_id = 0 if no object is found
        motion_planning::RoadObject get_lag_vehicle(bool left) const;
        
        // Gets the vehicle that is ahead/equal with the car
        // `left` specifies if we are looking in the left lane or right lane.
        // Returns a RoadObject with object_id = 0 if no object is found
        motion_planning::RoadObject get_ahead_vehicle(bool left) const;

        // Gets the distance from the car to the lane given by `lane`
        // Returns the distance to the lane
        double get_distance_to_lane(int lane) const;


        // All of the state handlers
        void keep_lane();
        void prep_lane_change();
        void lane_change();
        void follow_leader();
        void stop();

        // Initializes the origin of the coordinates
        void intialize_origin(const motion_planning::Autonomy& autonomy);
    public:
        CarPlanner();
        void update(const motion_planning::RoadObject& object);
        void update(const motion_planning::Autonomy& autonomy);
        void plan();
        State get_state() const;
};