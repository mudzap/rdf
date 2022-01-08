#include "rdf_ur/rdf_robot.h"

rdf::robot::robot(const std::string& robot_name) :
    uuid(boost::uuids::random_generator()()),
    robot_nh(ros::NodeHandle(robot_name.c_str())) {
}