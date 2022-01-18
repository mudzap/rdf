# Instrucciones ejecución de simulación UR5 y Docker

## Instrucciones UR5

Para simular el UR5, ejecutar los siguientes comandos en forma secuencial:

```
roscore
roslaunch ur_gazebo ur5_bringup.launch limited:=true
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true
```

roscore puede ejecutarse en el host, en caso de realizarse la simulación dentro de un contenedor Docker, sin embargo, es opcional, al crear roslaunch un master de forma automatica.

**El script de utilidad "docker_startup_ur5_sim.sh" se encarga de todo lo anterior, incluyendo la ejecución de un cliente local de gazebo en el host.**

## Instrucciones Docker

### Creación de contenedor

El contenedor debe de ser construido antes de su ejecución, si observamos el script de utilidad "docker_build.sh", observaremos el siguiente comando:

`docker build "$@" -t ${RDF_CONTAINER_TAG} .`

Del cual simplemente se tomara:

`docker build -t CONTAINER_TAG .`

Una explicación breve:
- `docker run`: Construye imagenes Docker en base a un contexto y a una Dockerfile. El contexto se refiere al conjunto de archivos disponibles durante la construcción, y la Dockerfile es un archivo que define una serie de comandos necesarios para construir la imagen.
- `-t CONTAINER_TAG`: Define una etiqueta 
- `.`: Define al contexto como el directorio actual, del cual también buscara la Dockerfile.

Para mas información: https://docs.docker.com/engine/reference/commandline/build/

### Dockerfile

Si se observa el contenido de esta, se observara una serie de comandos, estos son los mismos (con algunas modificaciones pertinentes a Docker) que aquellos que se utilizarian al momento 

**Puede ignorar lo anterior y simplemente construir la imagen con "docker_build.sh".**

### Ejecución de contenedor

Por motivos de conveniencia, se pueden correr estos dentro de docker de la siguiente forma:
`docker run -v $(pwd)/catkin_ws:/root/catkin_ws -v ~/.ros:/root/.ros --rm -it --net=host CONTAINER_TAG bin/bash`

Una explicación breve:
- `docker run`: Ejecuta procesos en un contenedor aislado de docker, un contenedor se define como "unidad estandar de software que contiene todo el codigo y dependencias de este", en resumen, es un ambiente virtual (en este caso definido en el Dockerfile), que permite ejecutar programas en el ambiente de ROS Melodic, incluso sin tener Ubuntu 18.04.
- `-v $(pwd)/catkin_ws:/root/catkin_ws`: Monta el directorio del huesped `$(pwd)/catkin_ws` (Directorio catkin_ws, en el directorio actual), al directorio del contenedor `/root/catkin_ws`. Esto permite acceder a los archivos del huesped desde el contenedor. En este caso, es util para realizar la compilación y creación de paquetes catkin en el ambiente de desarrollo de ROS Melodic, asi como ejecutarlos dentro de este.
- `-v ~/.ros:/root/.ros`: Lo mismo, pero tiene el objetivo de permitir a masters de ROS dentro del contenedor a acceder a los logs de ROS del huesped.
- `--rm`: Elimina el contenedor automaticamente al salir de este.
- `-it`: Combinacion de las banderas `-i` y `-t`, mantiene el pipe STDIN abierta y asigna una pseudo-tty.
- `--net=host`: Comparte los recursos de red con el huesped, esto es util para conectar un servidor gazebo y varios nodos de ROS dentro del contenedor con el huesped. No se considera seguro.
- `CONTAINER_TAG`: Etiqueta unica del contenedor asignada durante su construcción. Esta se define por conveniencia en `common`.
- `bin/bash`: Comando a ejecutar al comenzar el contenedor. En este caso comienza una terminal bash interactiva.

Para mas información: https://docs.docker.com/engine/reference/run/

**Para ejecutar una terminal interactiva, se mantiene el script de utilidad "docker_run.sh", que no requiere argumentos y considera todo lo necesario.**

## Instrucciones Gazebo

Gazebo consiste de un cliente y servidor, esto es util para la ejecución de la simulación, ya que pueden existir problemas en comunicarse con un display server, en especial con aceleración de hardware dentro de un contenedor.

Al ejecutar `roslaunch ur_gazebo ur5_bringup.launch limited:=true`, se comenzara un servidor de gazebo (también se ejecuta el cliente, pero este falla), al cual nos podemos conectar a través del comando `gzclient`, incluso desde fuera del contenedor.

En el caso de ejecutar la simulación en el mismo host, no sera necesario esto.

**Como mencionado anteriormente, script de utilidad "docker_startup_ur5_sim.sh" se encarga de esto.**