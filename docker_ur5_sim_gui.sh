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
	--privileged \
    	${RDF_CONTAINER_TAG}_gui /bin/bash -c " \
echo 'Executing \"roscore\" in container named ${RDF_CONTAINER_NAME}_gui'
roscore &
echo 'Awaiting 5 seconds while roscore inits'
sleep 5

echo ' '
echo 'Executing \"roslaunch ur_gazebo ur5_bringup.launch limited:=true\" in container named ${RDF_CONTAINER_NAME}_gui'
roslaunch ur_gazebo ur5_bringup.launch limited:=true &
echo 'Awaiting 6 seconds while ur5 sim inits'
sleep 6

echo ' '
echo 'Executing \"roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true\" in container named ${RDF_CONTAINER_NAME}_gui' &
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true"
    	
xhost -local:docker

