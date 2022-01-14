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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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
    rdf::state state(main_nh);
    rdf::plan_interface plan("manipulator");

    plan.move_j(0.5, 0.7, 0.5, 0.0, 1.57, 0.0);
    plan.move_j(-0.5, 0.7, 0.5, 0.0, 1.57, 0.0);
    
    rdf::joint_tf_map tfs = state.get_robot_joint_tfs();
    for(const auto& tf: tfs) {
        const Eigen::Vector3d pos = tf.second.position;
        const Eigen::Vector3d rot = Eigen::Matrix3d(tf.second.orientation).eulerAngles(0,1,2);
        Eigen::IOFormat CleanFmt(2, 0, ", ", ", ", "[", "]");
        std::cout   << tf.first << "\n"
                    << "XYZ: " << pos.format(CleanFmt) << "\n"
                    << "RPY: " << rot.format(CleanFmt) 
                    << std::endl;
    }

    ROS_INFO_STREAM_NAMED(_RDF_LOG_NAME_, "Awaiting shutdown...");
    ros::waitForShutdown();

    return 0;
}