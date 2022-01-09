 #pragma once

 #include <tf2_ros/transform_listener.h>

 namespace rdf {
     class state {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
     };
    
};
