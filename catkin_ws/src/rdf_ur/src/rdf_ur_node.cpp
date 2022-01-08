/*  
 *  APPLICATION ENTRY POINT
 *      Define ros node
 *      Init memory, perhaps robot controllers/drivers (modifiable by args? or done in program?)
 *
 *  Single node? How will it interface with other, not defined ROS nodes?
 * 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "rdf_ur/rdf_robot.h"

#include <vector>
#include <stdio.h>

int main (int argc, char** argv) {
    ros::init(argc, argv, "rdf_ur_node");

    ros::NodeHandle main_nh;
    rdf::robot robot("beeper");

    ros::spin();
    return 0;
}