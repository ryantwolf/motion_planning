#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/motion_planning/CarPlanner.h"
#include "motion_planning/RoadObject.h"
#include "motion_planning/Autonomy.h"

CarPlanner car_planner;

void autonomy_callback(const motion_planning::Autonomy& autonomy) {
    car_planner.update(autonomy);
}
void road_object_callback(const motion_planning::RoadObject& road_object) {
    car_planner.update(road_object);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "planner");

    ros::NodeHandle n;

    ros::Subscriber autonomy_sub = n.subscribe("/topic2", 1000, autonomy_callback);
    ros::Subscriber road_object_sub = n.subscribe("/topic1", 1000, road_object_callback);

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        car_planner.plan();
        State state = car_planner.get_state();
        switch (state) {
            case State::keep_lane:
                ROS_INFO("KEEP LANE");
                break;
            case State::prep_lane_change:
                ROS_INFO("PREP LANE CHANGE");
                break;
            case State::change_left:
                ROS_INFO("CHANGE LEFT");
                break;
            case State::change_right:
                ROS_INFO("CHANGE RIGHT");
                break;
            case State::follow_leader:
                ROS_INFO("FOLLOW LEADER");
                break;
            case State::stop:
                ROS_INFO("STOP");
                break;
            default:
                ROS_INFO("UNKNOWN STATE");
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}