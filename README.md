# Dockerfile:

Dockerfile de trabajo, en este se teiene preparado los robots de UR y los grippers de Robotiq, la compilación y preparación del entrypoint tambien se realiza aqui (lo correcto es proveer el entrypoint uno mismo, pero es de uso personal, asi que no lo cambie)

# Dockerfile\_gui:

Dockerfile con preparación necesaria para utilizar programas con aceleración 3d, como lo son Gazebo. Requiere de nvidia-docker. El contenedor generado se deriva de aquel creado en la dockerfile. Solamente compatible con NVIDIA.

# Dockerfile-fix:

Dockerfile de trabajo 2, en este se hicieron pruebas con otro paquete alternativo del gripper.

# common:

En este archivo se define el nombre del contenedor y version, para automatizar los siguientes scripts

# docker\_build.sh:

En este archivo se construye el docker con los argumentos correctos de forma automatica.

# docker\_build\_gui.sh

Este script construye el contenedor de la dockerfile que permite el uso de interfaces graficas.

# docker\_run.sh

Este script ejecuta el contenedor construido en Dockerfile. Por defecto, monta el directorio catkin\_ws, que es en donde realizo, pruebo y diseño las practicas. El contenedor sin capacidades de interfaz grafica los utilizo para comprobar cosas con herramientas de la terminal como rosnode, rostopic.

# docker\_run\_gui.sh

Este script ejecuta el conteendor construido con Dockerfile\_gui. Lo utilizo para ejecutar programas en Gazebo/RViz.

# docker\_ur5...

Este script lo utilizaba para hacer pruebas con el UR5 y MoveIt, automatizaba todo el proceso tedioso de comenzar los nodos adecuados. No se utiliza desde que volvio irrelevante MoveIt para el desarrollo de las practicas (creo nomas lo utilice la primera semana antes de vacacionesw y durante la primera semana).

# ur5\_install\_script.sh

Este script busca automatizar la instalación de los paquetes de UR en una instalación normal de Ubuntu. Requiere de mas pruebas, solamente he podido realizar una hasta el momento y requiero ajustar algunas cosas de permisos de administrador.

# instrucciones\_git.md

Este archivo describe como utilizar git de forma simple, su objetivo era enseñar a los otros integrantes del equipo a utilizarlo. Ya no se requirio despues de el trabajo cambio el enfoque a desarrollar practicas.

# catkin\_ws/

Este directorio contiene casi todo lo el codigo que he hecho con excepción de algunos que borre por accidente, tiene una variedad de paquetes que posiblemente ahorita no se necesiten.

