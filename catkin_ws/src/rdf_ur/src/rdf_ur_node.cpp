/*  
 *  APPLICATION ENTRY POINT
 *      Define ros node
 *      Init memory, perhaps robot controllers/drivers (modifiable by args? or done in program?)
 *
 *  Single node? How will it interface with other, not defined ROS nodes?
 * 
 */

#ifndef _ROS_SPINNER_THREADS_
#define _ROS_SPINNER_THREADS_ 2
#endif

#define _RDF_LOG_NAME_ "rdf"

#include <ros/ros.h>

#include "rdf_ur/rdf_robot.h"
#include "rdf_ur/rdf_plan.h"
#include "rdf_ur/rdf_state.h"

#include <string>
#include <vector>
#include <stdio.h>

int main (int argc, char** argv) {
    std::string node_name = "rdf_ur_node";
    ROS_INFO_STREAM_NAMED(_RDF_LOG_NAME_, "Initializing ROS node" << node_name << "...");
    ros::init(argc, argv, node_name);
    ros::NodeHandle main_nh;

    ROS_INFO_STREAM_NAMED(_RDF_LOG_NAME_, "Creating ROS async spinner with " << _ROS_SPINNER_THREADS_ << " threads...");
    ros::AsyncSpinner spinner(_ROS_SPINNER_THREADS_);
    ROS_INFO_STREAM_NAMED(_RDF_LOG_NAME_, "Starting ROS spinner...");
    spinner.start();

    ROS_INFO_STREAM_NAMED(_RDF_LOG_NAME_, "Starting application...");
    rdf::robot robot("beeper");
    rdf::robot state(nh);
    rdf::plan_interface plan("manipulator");

    plan.move_l(0.3, 0.3, 0.3, 0.3, 0.3, 0.3);
    plan.move_l(0.5, 0.5, 0.3, 0.5, 0.5, 0.5);

    ros::waitForShutdown();

    return 0;
}