. ./common
echo "Running container: ${RDF_CONTAINER_TAG}"
docker run \
	-v $(pwd)/catkin_ws:/root/catkin_ws \
	-v ~/.ros:/root/.ros \
	--rm -it --net=host \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--env=DISPLAY \
	${RDF_CONTAINER_TAG} bin/bash
