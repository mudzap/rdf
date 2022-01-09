#include "rdf_ur/rdf_robot.h"

namespace rdf {
    robot::robot(const std::string& robot_name) :
        uuid(boost::uuids::random_generator()()),
        robot_nh(ros::NodeHandle(robot_name.c_str())),
        start_time(ros::Time::now()) {

        pubs[robot_pub::LIFETIME] = robot_nh.advertise<std_msgs::Float64>("lifetime", 1);

        timer = robot_nh.createTimer(ros::Duration(0.1),
            [this](const ros::TimerEvent& event){
                std_msgs::Float64 msg;
                msg.data = ros::Time::now().toSec() - this->start_time.toSec();
                this->pubs[robot_pub::LIFETIME].publish(msg);
            }
        );

        if (on_init() != robot_error::OK) {
            ROS_ERROR_STREAM_NAMED(_RDF_ROBOT_LOG_NAME_, "Failed to initialize robot " << robot_name);   
        }

    }

    robot_error robot::on_init() {
        #define DEFAULT_URDF "/root/catkin_ws/src/rdf_ur/urdf/default.urdf"
        if (!model.initFile(DEFAULT_URDF)){
            ROS_ERROR_NAMED(_RDF_ROBOT_LOG_NAME_, "Failed to parse urdf file");
            return robot_error::FAIL_URDF_IMPORT;
        }

        return robot_error::OK;
    }
};