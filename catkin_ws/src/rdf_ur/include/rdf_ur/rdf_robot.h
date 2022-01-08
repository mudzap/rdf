#pragma once

#include <ros/ros.h>
//#include <urdf/

#include <functional>
#include <unordered_set>
#include <string>

#include <boost/uuid/uuid.hpp>           
#include <boost/uuid/uuid_generators.hpp>

namespace rdf {

    class robot {
        
        public:
            robot(const std::string& robot_name);
            const boost::uuids::uuid uuid;

        private:
            
            ros::NodeHandle robot_nh;

    };

};