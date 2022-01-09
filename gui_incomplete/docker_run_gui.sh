. ./common

echo "Running container: ${RDF_CONTAINER_TAG}_gui"

xhost +local:docker

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it \
    	-v=$(pwd)/catkin_ws:/root/catkin_ws \
    	-v=${HOME}/.ros:/root/.ros \
    	-e="DISPLAY=$DISPLAY" \
    	-e="QT_X11_NO_MITSHM=1" \
    	-v="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    	-v="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    	-e="XAUTHORITY=$XAUTH" \
    	-v="$XAUTH:$XAUTH" \
    	--runtime=nvidia \
    	--rm -it --net=host \
    	${RDF_CONTAINER_TAG}_gui bin/bash
    	
xhost -local:docker

#xhost +local:docker
#docker run \
#	-v=$(pwd)/catkin_ws:/root/catkin_ws \
#	-v=${HOME}/.ros:/root/.ros \
#	-v=/tmp/.X11-unix:/tmp/.X11-unix \
#	-e="DISPLAY" \
#	-e="QT_X11_NO_MITSHM=1" \
#	-e="XAUTHORITY=$XAUTH" \
#	-v="$XAUTH:$XAUTH" \
#	--device=/dev/dri \
#	--group-add video \
#	${RDF_CONTAINER_TAG}_gui bin/bash
#xhost -local:docker
