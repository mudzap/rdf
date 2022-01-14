. ./common
echo "rosrun '$@' in container: ${RDF_CONTAINER_TAG}"
docker run -v ~/.ros:/root/.ros -v $(pwd)/catkin_ws:/root/catkin_ws --rm -it --net=host ${RDF_CONTAINER_TAG} bin/bash -c "\
source /root/catkin_ws/devel/setup.bash; \
rosrun '$@'; \
"
