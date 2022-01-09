. ./common
echo "Building container: ${RDF_CONTAINER_TAG}_gui"
docker build -t ${RDF_CONTAINER_TAG}_gui -f Dockerfile_gui .
