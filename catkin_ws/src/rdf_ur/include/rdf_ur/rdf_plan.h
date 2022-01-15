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

    /** @enum plan_error
     * @brief Error codes enum for planning
     */
    enum class plan_error {
        OK,
        PLAN_FAIL,
        EXEC_FAIL
    };

    /** @class plan_interface
     * @brief Class for planning/programming robot by common methods such as move_j, move_l
     */
    class plan_interface {

        public:
            /** 
             * @brief plan_interface constructor
             * @param planning_group    Robot's planning group
             */
            plan_interface(const std::string& planning_group);

            /**
             * @fn plan_error move_j(double x, double y, double z, double r, double p, double yaw);
             * @brief Plans robot to move by joint interpolation
             * 
             * @param x     X position of end effector
             * @param y     Y position of end effector
             * @param z     Z position of end effector
             * @param r     Roll of end effector
             * @param p     Pitch of end effector
             * @param yaw   Yaw of end effector
             * @return plan_error 
             */
            plan_error move_j(double x, double y, double z, double r, double p, double yaw);
            /**
             * @overload plan_error move_j(double x, double y, double z);
             * @brief Plans robot to move by joint interpolation, doesn't change end effector orientation
             */
            plan_error move_j(double x, double y, double z);

            /**
             * @fn plan_error move_l(double x, double y, double z, double r, double p, double yaw);
             * @brief Plans robot to move by linear interpolation
             * 
             * @param x     X position of end effector
             * @param y     Y position of end effector
             * @param z     Z position of end effector
             * @param r     Roll of end effector
             * @param p     Pitch of end effector
             * @param yaw   Yaw of end effector
             * @return plan_error 
             */
            plan_error move_l(double x, double y, double z, double r, double p, double yaw);
            /**
             * @overload plan_error move_l(double x, double y, double z);
             * @brief Plans robot to move by linear interpolation, doesn't change end effector orientation
             */
            plan_error move_l(double x, double y, double z);

        private:
            const std::string planning_group;                                               // Set of joints
            moveit::planning_interface::MoveGroupInterface move_group;                      // Interface to move_group
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;    // Defs collision objs
            const robot_state::JointModelGroup* joint_model_group;                        // Stores planning group
            moveit::core::RobotStatePtr current_state_ptr;

            std::vector<double> initial_joint_group_positions;

    };

};