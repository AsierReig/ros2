# ROS2
Repositorio para cargar un modelo entrenado de YOLO en un nodo de ROS2 utilizando la cámara Orbbec Astra.

## Nodo YOLO
Se ha creado un nodo desde cero para conectarse con los topic de la cámara. Para ello, se han seguido los siguientes pasos:
### 1. Crear paquete
Para crear un paquete tenemos que estar en el directorio de trabajo **/ros2_ws/src** previamente creado (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) y ejecutar desde terminal:
```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 nombre_paquete
```
### 2. Crear nodo Subscriber
 Descargar ejemplo de nodo suscriptor de ROS2, dentro de la ruta **ros2_ws/src/nombre_paquete/nombre_paquete**:
```bash
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
Nos encontraremos un archivo con el nombre ***susbcriber_member_function.py***, el cual podemos renombrar y editar su contenido. En nuestro caso, nos suscribimos a dos topics de la cámara Orbbec Astra, tales como **/camera/color/image_raw** y **/camera/depth/image_raw**, los cuales nos permiten visulaizar las imágenes de la cámara y su profundidad. Una vez suscrito, podemos cargar nuestro modelo YOLO entrenado y pasarle cada imagen de la cámara para que lance el programa y detecte el objeto que queramos. Además, se recoge información sobre la clase, confianza y coordenada 3D de cada objeto detectado en otro topic creado llamado **/detected_objects**, el cual se explicará más adelante.

### 3. Añadir dependencias
En la ruta **ros2_ws/scr/nombre_paquete**, se encuentran los archivos creados previamente para nosotros: ***setup.py, setup.cfg y package.xml***
Dentro de `*package.xml* debemos completar las tags y añadir las siguientes dependencias que el paquete necesita cuando ejecuta el nodo:
```bash
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

### 4. Añadir puntos de entrada
En el archivo *setup.py* debemos completar las tags para que coincidan con las de *package.xml* y añadir la siguiente línea:
```bash
entry_points={
        'console_scripts': [
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```
Revisar el archivo *setup.cfg*, que debería tener este aspecto:
```bash
[develop]
script_dir=$base/lib/py_pubsub
[install]
install_scripts=$base/lib/py_pubsub
```

### 5. Construir y compilar
Retrocedemos a la ruta **/ros_ws** y ejecutamos *rosdep* para revisar si nos falta alguna dependencia.
```bash
rosdep install -i --from-path src --rosdistro humble -y
```
Para construir el paquete ejecutamos:
```bash
colcon build --packages-select nombre_paquete
```
Para compilar el paquete:
```bash
source install/setup.bash
```

Ahora podemos lanzar el nodo de Yolo:
```bash
ros2 run nombre_paquete YOLO_subscriber
```



