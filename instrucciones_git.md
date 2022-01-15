# Instrucciones git

git es un sistema de control de versión. Se puede utilizar desde el sitio web, o de la terminal o otra herramienta con GUI. Es universalmente utilizado y permite colaboración entre varios desarrolladores.

## Resumen

- `git clone`: Clona un repositorio a una carpeta dentro de tu directorio. En este caso realizarias uno de los dos comandos:
  - `git clone git@github.com:mudzap/rdf.git`
  - `git clone https://github.com/mudzap/rdf.git`
- `git commit`: Realiza un commit, o sea, especifica un punto con los cambios actuales. Para realizar un commit es necesario poner un mensaje de commit y especificar los archivos de los cuales se esta haciendo el commit, por ejemplo, si implementaste algo en el archivo "rdf_ur.h", puedes ejecutar lo siguiente:
  - `git commit -am "Mensaje de commit"`: Realiza un commit de todos los archivos, incluyendo rdf_ur.h.
  - `git commit -m "Mensaje de commit" rdf_ur.h`: Realiza un commit de rdf_ur.h.
- `git add`: Añade archivos de los cuales llevara seguimiento, cada que se cree un archivo relevante al repositorio, es necesario añadir este. Supongamos implementaste una clase "foo" en el archivo "rdf_foo.cpp", puedes ejecutar lo siguiente:
  - `git add rdf_foo.cpp`: Añade rdf_foo.cpp a los archivos de los cuales lleva seguimiento el repositorio.
  - `git add .`: Añade todos los archivos en el directorio actual.
- `git push`: Empuja cambios del ultimo commit a una rama. Supongamos que quiere subir el ultimo commit realizado al repositorio, entonces ejecutas:
  - `git push origin master`: Empuja cambios al repositorio de origen (Si clonaste el repositorio, automaticamente sera el correcto, de otra forma habra que configurarlo), en la rama master. Esto no es buena practica, pero para no hacerlos batallar digo no se preocupen.
  - `git push origin ivan/feature`: Empuja cambios al repositoiro de origen a la rama ivan/feature. Esto seria lo ideal (Crear una rama por separado, hacer los cambios en esta, y realizar un merge posteriormente a través de un pull request).
- `git pull`: Toma cambios del repositorio. Se usa identicamente que `push`, indicando la rama y el repositorio. 
- `git checkout`: Cambia de rama, tipicamente utilizaras:
  - `git checkout ivan/feature`: Cambia a la rama ivan/feature.
  - `git checkout -b ivan/feature`: Cambia a la rama ivan/feature, si no existe, la creara.
- `git branch`: Permite administrar ramas. Tipicamente utilizaras lo siguiente:
  - `git branch -d ivan/feature`: Borra la rama ivan/feature. (Util despues de hacer un merge).
  - `git branch ivan/feature`: Crea la rama ivan/feature (Pero no cambia a esta).
  - `git branch`: Muestra la rama actual en la que estas trabajando asi como las ramas disponibles en tu repositorio.
- `git rm`: Deja de rastrear archivos (Lo contrario a `git add`).
- `git merge`: Hace un "merge" (unifica cambios), de una rama con otra.

## Otros

- `git rebase`: Modifica una rama en base a los commits de otra, es util cuando la rama master recibe cambios, y no puedes integrar tu rama a esta, requiriendo un pull, o un rebase primero.
