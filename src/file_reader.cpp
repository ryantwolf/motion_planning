#include "ros/ros.h"
#include "motion_planning/RoadObject.h"
#include "motion_planning/Autonomy.h"
#include <fstream>


int main(int argc, char **argv) {
    ros::init(argc, argv, "file_reader");

    ros::NodeHandle n;

    ros::Publisher obj_pub = n.advertise<motion_planning::RoadObject>("topic1", 1000);
    ros::Publisher auto_pub = n.advertise<motion_planning::Autonomy>("topic2", 1000);
    // Relative paths are weird in ROS, so we need to use an absolute path
    std::ifstream obj_in("/home/ryan/catkin_ws/src/motion_planning/topic1.txt");
    std::ifstream auto_in("/home/ryan/catkin_ws/src/motion_planning/topic2.txt");
    motion_planning::RoadObject road_object;
    motion_planning::Autonomy autonomy;

    // Get rid of the header line in the file
    std::string line;
    std::getline(auto_in, line);
    std::getline(obj_in, line);

    ros::Rate loop_rate(10);
    // ROS_INFO("Started");
    while(ros::ok()) {
        std::getline(auto_in, line);
        if (auto_in.eof()) {
            break;
        }
        // ROS_INFO(line.c_str());
        std::istringstream iss(line);
        std::string token;
        int i = 0;
        while(std::getline(iss, token, ',')) {
            // ROS_INFO(token.c_str());
            switch (i) {
                case 0:
                    autonomy.loc_x = std::stod(token);
                    break;
                case 1:
                    autonomy.loc_y = std::stod(token);
                    break;
                case 2:
                    autonomy.vel = std::stod(token);
                    break;
                case 3:
                    autonomy.accel = std::stod(token);
                    break;
                case 4:
                    autonomy.heading = std::stod(token);
                    break;
            }

            i++;
        }

        auto_pub.publish(autonomy);

        std::getline(obj_in, line);
        if (obj_in.eof()) {
            break;
        }

        std::istringstream iss2(line);
        std::string token2;
        int j = 0;
        while(std::getline(iss2, token2, ',')) {
            switch (j) {
                case 0:
                    road_object.object_id = (uint8_t) std::stoi(token2);
                    break;
                case 1:
                    road_object.object_type = token2;
                    break;
                case 2:
                    road_object.loc_x = std::stod(token2);
                    break;
                case 3:
                    road_object.loc_y = std::stod(token2);
                    break;
                case 4:
                    road_object.vel = std::stod(token2);
                    break;
                case 5:
                    road_object.accel = std::stod(token2);
                    break;
            }
            j++;
        }

        obj_pub.publish(road_object);

        ros::spinOnce();
        loop_rate.sleep();
    }

    auto_in.close();
    obj_in.close();

    return 0;
}