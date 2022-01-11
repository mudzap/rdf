 #pragma once

#include <unordered_map>
#include <string>

#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <ros/ros.h>

#include <Eigen/Dense.h>

/* Digital twins
 *
 *  /tf: Joint transforms (can be prefixed)
 *  /joint_states: Joint states
 * 
 *  If sim is active: ????
 *  If physical robot is active: Acquire robot state
 *  
 *  To differentiate sim from physical robot:
 *  - /ur_hardware_interface/joints exists (or /ur_hardware_interface for that matter)
 *  - /ur_hardware_interface/robot_mode > -1?
 *  - Is it possible to implement for all robots universally?
 * 
 *  URDF: Contained in parameter /robot_description (nice!)
 *  - joint_state_publisher reads this parameter, automagically
 * 
 *  Should we automagically connect to a physical robot, and if not available, just sim?
 * 
 *  For now, lets just give the user the robot state for any reason they might want it.
 */

namespace rdf {

   // Abstracts sensor_msgs::JointState (SoA -> AoS)
   struct joint_state {
      Eigen::Array3f position;
      Eigen::Array3f velocity;
      Eigen::Array3f effort;
   };

   class state {
        
         public:
            state(int refresh_rate_hint = -1);
            virtual std::unordered_map<std::string, joint_state> get_robot_state(); // Returns map of joint states
            virtual joint_state get_joint_state(const std::string& j_name);         // Returns joint state (just use map!)

         private:
            bool is_sim;

            virtual bool is_robot_simulated();
            virtual void set_robot_state_pubs();

            std::unordered_map<std::string, joint_state> joint_states // Not critical, maybe it is alright to use string?
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);

   };
    
};
