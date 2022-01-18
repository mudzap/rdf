. ./common

sleep 10 && gzclient &

echo "Running container: ${RDF_CONTAINER_TAG}, with name ${RDF_CONTAINER_NAME}"
docker run \
	-v $(pwd)/catkin_ws:/root/catkin_ws \
	-v ~/.ros:/root/.ros \
	--name ${RDF_CONTAINER_NAME} \
	--rm -it --net=host ${RDF_CONTAINER_TAG} /bin/bash -c " \
echo 'Executing \"roscore\" in container named ${RDF_CONTAINER_NAME}'
roscore &
echo 'Awaiting 5 seconds while roscore inits'
sleep 5

echo ' '
echo 'Executing \"roslaunch ur_gazebo ur5_bringup.launch limited:=true\" in container named ${RDF_CONTAINER_NAME}'
roslaunch ur_gazebo ur5_bringup.launch limited:=true &
echo 'Awaiting 6 seconds while ur5 sim inits'
sleep 6

echo ' '
echo 'Executing \"roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true\" in container named ${RDF_CONTAINER_NAME}' &
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true"
