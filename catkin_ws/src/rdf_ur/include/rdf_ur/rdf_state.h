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

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/function.hpp>

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

#define _RDF_STATE_LOG_NAME_ "rdf::state"

/** @namespace rdf
 * @brief Encompasses all functions and classes provided by rdf
 */
namespace rdf {

   /** @struct joint_state
    * @brief Contains joint state necessary for control.
    * @var joint_state::position
    * Contains translation, or rotation of a linear or rotary joint respectively
    * @var joint_state::velocity
    * Contains linear or rotational velocity of a linear or rotary joint respectively
    * @var joint_state::effort
    * Contains force or torque of a linear or rotary joint respectively
    */
   struct joint_state;

   /** @struct joint_tf
    * @brief Contains a joint's transform.
    * @var joint_tf::position
    * Vector containing the euclidean position of the joint's origin
    * @var joint_tf::orientation
    * Quaternion containing the joint's orientation in respect to its reference frame
    */
   struct joint_tf;

   /** @brief Unordered map of joint states, the joint's name is utilized as the map's key */
   typedef std::unordered_map<std::string, joint_state> joint_state_map;
   /** @brief Unordered map of joint transforms, the joint's name is utilized as the map's key  */
   typedef std::unordered_map<std::string, joint_tf> joint_tf_map;

   struct joint_state {
      double position;
      double velocity;
      double effort;
   };

   struct joint_tf {
      Eigen::Vector3d position;
      Eigen::Quaternion<double> orientation;
   };

   class state {
        
         public:
            state(ros::NodeHandle nh);
            joint_state_map get_robot_joint_states(); // Returns map of joint states
            joint_state get_joint_state(const std::string& j_name);         // Returns joint state (just use map!)
            joint_tf_map get_robot_joint_tfs();
            joint_tf get_joint_tf(const std::string& j_name);         

         private:
            bool is_sim;
            ros::Subscriber tf_sub;
            ros::Subscriber js_sub;

            virtual bool is_robot_simulated();
            void setup_state_subscribers(ros::NodeHandle nh);

            joint_state_map joint_states; // Not critical, maybe it is alright to use string?
            joint_tf_map joint_tfs;
            //tf2_ros::Buffer tfBuffer;
            //tf2_ros::TransformListener tfListener(tfBuffer);

   };
    
};
