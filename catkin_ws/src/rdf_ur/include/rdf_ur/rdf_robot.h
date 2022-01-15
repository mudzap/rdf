#pragma once

/** @file rdf_robot.h
 *  Header file for robot class.
 */ 

#define _RDF_ROBOT_LOG_NAME_ "rdf::robot"

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>

#include <urdf/model.h>

#include <functional>
#include <unordered_map>
#include <string>

#include <boost/uuid/uuid.hpp>           
#include <boost/uuid/uuid_generators.hpp>

namespace rdf {

    /** @enum robot_sub
     * @brief Enum for standards subscribers for created robots
     */
    enum class robot_sub {
    };

    /** @enum robot_pub
     * @brief Enum for standards publishers for created robots
     */
    enum class robot_pub {
        LIFETIME,
        CAMERA
    };

    /** @enum robot_error
     * @brief Error codes for robot creation
     */
    enum class robot_error {
        OK,
        FAIL_URDF_IMPORT
    };


    /** @class robot
     *  @brief Class for creation of robot from an urdf file
     */
    class robot {
        
        public:
            robot(const std::string& robot_name);
            const boost::uuids::uuid uuid;
            const ros::Time start_time;

        private:
            virtual robot_error on_init();
            urdf::Model model;
            
            ros::NodeHandle robot_nh;
            ros::Timer timer;
            std::unordered_map<robot_pub, ros::Subscriber> subs;
            std::unordered_map<robot_pub, ros::Publisher> pubs;

    };

};