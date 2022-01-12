#include "rdf_ur/rdf_state.h"

namespace rdf {

    state::state(ros::NodeHandle nh) {
        ROS_INFO_STREAM_NAMED(_RDF_STATE_LOG_NAME_, "Determining if robot is simulated...");
        is_sim = is_robot_simulated();
        ROS_INFO_STREAM_NAMED(_RDF_STATE_LOG_NAME_, "Setting up subscribers...");   
        setup_state_subscribers(nh);
    }

    void state::setup_state_subscribers(ros::NodeHandle nh) {
        /* Options:
         * - /tf: Can be namespaced in launch file (for multiple robots)
         * - /joint_states
         */
        boost::function<void(const tf2_msgs::TFMessageConstPtr&)> tf_cb = 
        [&](const tf2_msgs::TFMessageConstPtr& tf2){
            for (auto& tf_stamped : tf2->transforms) {
                const auto& tf = tf_stamped.transform;
                const auto& name = tf_stamped.header.frame_id;
                
                Eigen::Vector3d pos;
                Eigen::Quaternion<double> quat;  
                tf2::fromMsg (tf.translation, pos);
                tf2::fromMsg (tf.rotation, quat);

                const joint_transform j_tf = {pos, quat};
                this->joint_transforms[name] = j_tf;
            }
        };
        tf_sub = nh.subscribe("/tf", 3, tf_cb);
            
        boost::function<void(const sensor_msgs::JointStateConstPtr&)> js_cb =
        [&](const sensor_msgs::JointStateConstPtr& js){
            for (size_t it = 0; it < js->name.size(); it++) {
                this->joint_states[js->name[it]] = {js->position[it], js->velocity[it], js->effort[it]};
            }
        };
        js_sub = nh.subscribe("/joint_states", 3, js_cb);

    }

    bool state::is_robot_simulated() {
        // Searches if any node contains "hardware_interface" in its name
        ros::V_string nodes;
        ros::master::getNodes(nodes);
        for (auto &str: nodes) {
            if(str.find("hardware_interface") != std::string::npos) {return false;}
        }
        return true;
    }

    /*
    std::unordered_map<std::string, joint_state> state::get_robot_joint_state() {
    }
    joint_state state::get_joint_state(const std::string& j_name) {
    }
    std::unordered_map<std::string, joint_state> get_robot_transforms() {
    }
    joint_state get_joint_transform(const std::string& j_name) {
    } 
    */
    
};
