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

int main (int argc, char** argv) {
    ros::init(argc, argv, "rdf_ur_node");

    ros::NodeHandle main_nh;

    ros::spin();
    return 0;
}