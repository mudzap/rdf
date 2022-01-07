. ./common
echo "Building catkin packages with container: ${RDF_CONTAINER_TAG}"
docker run -v $(pwd)/catkin_ws:/root/catkin_ws --rm -it ${RDF_CONTAINER_TAG} bin/bash -c "\
cd /root/catkin_ws; \
catkin_make; \
"
