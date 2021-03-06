FROM ros:melodic-ros-base-bionic

# utilities
RUN apt-get update -qq && \
	apt-get install vim nano git -y -qq --no-install-recommends \
	&& rm -rf /var/lib/apt/lists*

# install ur5 packages
RUN mkdir -p ur_ws/src
WORKDIR /ur_ws
RUN git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git
RUN apt update -qq && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -y
# build
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
	catkin_make -DCMAKE_BUILD_TYPE=Release"
	
# update entrypoint
WORKDIR /
RUN sed '/exec "$@"/i source /ur_ws/devel/setup.bash --extend' -i ros_entrypoint.sh

# other deps
RUN apt-get update && \
	apt-get install \ 
	ros-melodic-rqt-joint-trajectory-controller \
	ros-melodic-urdf-tutorial \
	-y -qq --no-install-recommends \
	&& rm -rf /var/lib/apt/lists*
