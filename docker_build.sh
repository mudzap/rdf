. ./common
echo "Building container: ${RDF_CONTAINER_TAG}"
docker build -t ${RDF_CONTAINER_TAG} .
