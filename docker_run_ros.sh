. ./common
echo "Running ROSCORE in container: ${RDF_CONTAINER_TAG}"
docker run -v ~/.ros:/root/.ros --rm -it --net=host ${RDF_CONTAINER_TAG} roscore
