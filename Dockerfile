FROM ros:melodic-ros-base-bionic

# install ros perception & robot packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-perception=1.4.1-0* \
    ros-melodic-robot=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

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
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release"

# update entrypoint
WORKDIR /
RUN sed '/exec "$@"/i source /ur_ws/devel/setup.bash' -i ros_entrypoint.sh

# action deps
RUN apt-get update && \
	apt-get install \ 
	ros-melodic-rqt-joint-trajectory-controller \
	libgtest-dev \
	doxygen \
	ros-melodic-rosdoc-lite \
	-y -qq --no-install-recommends \
	&& rm -rf /var/lib/apt/lists*

# make dummy package for other dependencies (bit hackish)
RUN mkdir -p dummy_ws/src
WORKDIR /dummy_ws/src
COPY ./catkin_ws/src/dependencies .
RUN /bin/bash -c "source /ur_ws/devel/setup.bash && xargs catkin_create_pkg dummy_pkg < dependencies"
WORKDIR /dummy_ws
RUN /bin/bash -c "sudo apt-get update -qq && \
	source /ur_ws/devel/setup.bash && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -y \
	&& rm -rf /var/lib/apt/lists*"
WORKDIR /
RUN rm -rf dummy_ws
  
