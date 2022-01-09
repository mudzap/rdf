#include "rdf_ur/rdf_plan.h"

namespace rdf {

    plan_interface::plan_interface(const std::string& planning_group) :
        planning_group(planning_group),
        move_group(moveit::planning_interface::MoveGroupInterface(planning_group)) {
        
        current_state_ptr = move_group.getCurrentState();
        joint_model_group = current_state_ptr->getJointModelGroup(planning_group);
        current_state_ptr->copyJointGroupPositions(joint_model_group, initial_joint_group_positions);
        //ROS_INFO_STREAM("Initial joint group positions for " << planning_group << ": " << initial_joint_group_positions);

    }

    plan_error plan_interface::move_l(double x, double y, double z, double roll, double pitch, double yaw) {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        geometry_msgs::Pose p;
        p.orientation = tf2::toMsg(q);
        p.position.x = x;
        p.position.y = y;
        p.position.z = z;

        move_group.setPoseTarget(p);
        move_group.move();

        return plan_error::OK;
    }

    plan_error plan_interface::move_l(double x, double y, double z) {
        return plan_error::OK;
    }



};