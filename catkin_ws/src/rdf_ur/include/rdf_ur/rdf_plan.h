#pragma once

/*
 *  PLAN
 *  Base class for other robot moveit planning groups
 * 
 * 
 * 
 * 
 * 
 */

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#define _RDF_PLAN_LOG_NAME_ "rdf::plan"

namespace rdf {

    enum class plan_error {
        OK,
        PLAN_FAIL,
        EXEC_FAIL
    };

    class plan_interface {

        public:
            plan_interface(const std::string& planning_group);

            plan_error move_j(double x, double y, double z, double r, double p, double yaw);
            plan_error move_j(double x, double y, double z);
            plan_error move_l(double x, double y, double z, double r, double p, double yaw);
            plan_error move_l(double x, double y, double z);
            // subscribe once to tf or tf_static (for initial pos)

        private:
            const std::string planning_group;                                               // Set of joints
            moveit::planning_interface::MoveGroupInterface move_group;                      // Interface to move_group
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;    // Defs collision objs
            const robot_state::JointModelGroup* joint_model_group;                        // Stores planning group
            moveit::core::RobotStatePtr current_state_ptr;

            std::vector<double> initial_joint_group_positions;

    };

};