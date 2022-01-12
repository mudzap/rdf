#include "rdf_ur/rdf_state.h"

namespace rdf {

    state::state(const int refresh_rate_hint = -1) {
        is_sim = is_robot_simulated();
        setup_state_subscribers()
    }

    void state::setup_state_subscribers(const int refresh_rate_hint = -1) {
        /* Options:
         * - /tf: Can be namespaced in launch file (for multiple robots)
         * - /joint_states
         */
        ros::Subscriber sub = n.subscribe("/tf", 3,
            [this](tf2_msgs::TFMessage tf2){
                for (auto& tf_stamped : tf2) {
                    const auto& tf = tf_stamped.transform;
                    const auto& name = tf_stamped.header.frame_id;
                    
                    Eigen::Vector3d pos;
                    Eigen::Quaternion quat;  
                    tf2::fromMsg (tf.translation, pos)
                    tf2::fromMsg (tf.rotation, quat)

                    const joint_transform j_tf = {tf.position, tf.rotation};
                    this->joint_transforms[name] = j_tf;
                }
            });
        ros::Subscriber sub = n.subscribe("/joint_states", 3,
            [this](sensor_msgs::JointState js){
                for (size_t it = 0; it < js.name.size(); it++) {
                    this->joint_states[js.name[it]] = {js.position[it], js.velocity[it], js.effort[it]}
                }
            });

    }

    bool state::is_robot_simulated(const int refresh_rate_hint = -1) {
        // Searches if any node contains "hardware_interface" in its name
        ros::V_string nodes;
        ros::master::getNodes(nodes);
        for (auto &str: nodes) {
            if(str.find("hardware_interface") != string::npos) {return false;}
        }
        return true;
    }
    
};
