. ./common
echo "Running container: ${RDF_CONTAINER_TAG}"
docker run -v $(pwd)/catkin_ws:/root/catkin_ws -v ~/.ros:/root/.ros --rm -it --net=host ${RDF_CONTAINER_TAG} bin/bash
