# rdf

## Ver

- https://moveit.ros.org/documentation/plugins/#movegroupcapability
- https://github.com/ros-planning/moveit/blob/master/moveit_ros/move_group/default_capabilities_plugin_description.xml
- https://github.com/ros-planning/moveit/blob/master/moveit_ros/move_group/src/default_capabilities/cartesian_path_service_capability.cpp

## TODO

- Enrobustecer simulacion (El programa debe indicar falla en planeacion!)
- Permitir creación de simulaciones con URDF (rdf_robot.cpp)
- Documentación y pruebas unitarias (doxygen, gtest, rostest)

## Baja prioridad

- Camara
  - Subscribible, o accesible de /dev/
- Bindings de python
  - melodic soporta hasta python2, 3 con algunas modificaciones, es preferible python3 por librerias disponibles en esta version.
