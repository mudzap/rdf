FROM ros:melodic-ros-base-bionic

# utilities
RUN apt-get update -qq && \
	apt-get install vim nano git -y -qq --no-install-recommends \
	&& rm -rf /var/lib/apt/lists*

# install ur5 packages
RUN mkdir -p ur_ws/src
WORKDIR /ur_ws
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
RUN git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
RUN apt update -qq && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -y
# build
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
	catkin_make -DCMAKE_BUILD_TYPE=Release"
	
# install robotiq packages
WORKDIR /
RUN mkdir -p robotiq_ws/src
WORKDIR /robotiq_ws
RUN git clone https://github.com/Danfoa/robotiq_2finger_grippers.git src
RUN apt update -qq && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -y
# build
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
	catkin_make -DCMAKE_BUILD_TYPE=Release"

# update entrypoint
WORKDIR /
RUN sed '/exec "$@"/i source /ur_ws/devel/setup.bash --extend' -i ros_entrypoint.sh
RUN sed '/exec "$@"/i source /robotiq_ws/devel/setup.bash --extend' -i ros_entrypoint.sh

# other deps
RUN apt-get update && \
	apt-get install \ 
	ros-melodic-rqt-joint-trajectory-controller \
	ros-melodic-urdf-tutorial \
	-y -qq --no-install-recommends \
	&& rm -rf /var/lib/apt/lists*

  
