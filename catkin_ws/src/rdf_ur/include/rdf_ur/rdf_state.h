 #pragma once

#include <unordered_map>
#include <string>
#include <functional>

#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <ros/ros.h>
#include <ros/master.h>

#include <Eigen/Dense.h>
#include <Eigen/Geometry.h>

/* Digital twins
 *
 *  /tf: Joint transforms (can be prefixed)
 *  /joint_states: Joint states
 * 
 *  If sim is active: ????
 *  If physical robot is active: Acquire robot state
 *  
 *  To differentiate sim from physical robot:
 *  - /ur_hardware_interface exists
 *  - /ur_hardware_interface/robot_mode > -1?
 *  - Is it possible to implement for all robots universally?
 *  - /ur_hardware_interface name space can be edited in launch file (see: ur_control.launch)
 * 
 *  URDF: Contained in parameter /robot_description (nice!)
 *  - joint_state_publisher reads this parameter, automagically
 * 
 *  Should we automagically connect to a physical robot, and if not available, just sim?
 * 
 *  For now, lets just give the user the robot state for any reason they might want it.
 */

namespace rdf {

   // Abstracts sensor_msgs::JointState (SoA -> AoS), utilized with map
   struct joint_state {
      double position;
      double velocity;
      double effort;
   };

   // Abstracts tf2_msgs::TFMessage to utilize with map
   struct joint_transform {
      Eigen::Vector3d position;
      Eigen::Quaternion velocity;
   };

   class state {
        
         public:
            state(const int refresh_rate_hint = -1);
            virtual std::unordered_map<std::string, joint_state> get_robot_joint_state(); // Returns map of joint states
            virtual joint_state get_joint_state(const std::string& j_name);         // Returns joint state (just use map!)
            virtual std::unordered_map<std::string, joint_state> get_robot_transforms();
            virtual joint_state get_joint_transform(const std::string& j_name);         

         private:
            bool is_sim;
            

            virtual bool is_robot_simulated();
            void setup_state_subscribers();

            std::unordered_map<std::string, joint_state> joint_states; // Not critical, maybe it is alright to use string?
            std::unordered_map<std::string, joint_transform> joint_transforms;
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);

   };
    
};
