import rclpy    # Biblioteca de ROS para Python
from rclpy.node import Node     # Base para crear nodos en ROS2
from sensor_msgs.msg import Image   # Tipo de mensaje que se espera (una imagen)
import cv2      # Para convertir las imágenes de ROS2 a OpenCV
from cv_bridge import CvBridge  # Biblioteca para procesar imágenes en Python
from message_filters import ApproximateTimeSynchronizer, Subscriber # Librería necesaria para sincronizar dos topic's
from ultralytics import YOLO # Librería modelo detección de objetos
from interfaces.msg import DetectedObject, DetectedObjects # Mensajes de mi paquete
import numpy as np # Librería mmatrices y vectores
import time # Funciones relacionadas con el tiempo

class YOLOSubscriber(Node):

    def __init__(self):
        super().__init__('YOLO_subscriber') # Nombre del nodo

        # Subscriptores para RGB y profundidad
        self.rgb_subscription = Subscriber(
            self, # Necesario porque el constructor espera este argumento
            Image, # Tipo de mensaje que esperas
            '/camera/color/image_raw') # Nombre del topic

        self.depth_subscription = Subscriber(
            self,
            Image, 
            '/camera/depth/image_raw') # Nombre del topic
        
        # Sincronizador de mensajes
        self.sync = ApproximateTimeSynchronizer([self.rgb_subscription, self.depth_subscription], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.listener_callback)

        # Publisher de detecciones
        self.detection_pub = self.create_publisher(DetectedObjects, '/detected_objects', 10)

        self.subscriptions  # prevent unused variable warning
        self.last_time = time.time()  # Guardar tiempo inicial
        self.br = CvBridge() # Instancia para convertir de ROS a OpenCV
        self.frame = None   # Inicializar el frame como None para evitar errores antes de llegar la primera Imagen

         # Cargar el modelo YOLO entrenado
        self.model = YOLO('/home/openheimer/Repositorios/rai_yolo_object_detection/Modelo/best.pt')

        # Matriz intrínseca de la cámara (calibración)
        self.K = np.array([[570.3422047415297, 0, 319.5],  # Se puede obtener del topic camera/depth/camera_info
                           [0, 570.3422047415297, 239.5],
                           [0, 0, 1]])

    def listener_callback(self, rgb_msg, depth_msg): 

        # Este es el *callback* que se ejecuta cada vez que se recibe un mensaje
        #self.frame = msg.data

        # Convertir mensajes ROS a OpenCV
        current_frame = self.br.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_frame = self.br.imgmsg_to_cv2(depth_msg, "passthrough")

        # Ejecutar detección con el modelo YOLO
        results = self.model(current_frame)

        # Crear mensaje para objetos detectados
        detected_objects_msg = DetectedObjects()
        
        for box in results[0].boxes:
            # try:
                # For bounding box
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0]) 

                # For classname
                # List with the classes of the model
                classNames =  ['Bottle','Cup','Spoon']
                cls = int(box.cls)
                current_cls = classNames[cls]

                # For id object
                #id = box.id
                
                # For confidence
                confidence = float(box.conf)

                # Calcular el centro del bounding box
                center_x = (x_min + x_max) // 2  # Division entera
                center_y = (y_min + y_max) // 2

                # Obtener profundidad en el centro
                z = depth_frame[center_y, center_x] / 1000

                # Proyectar a coordenadas 3D
                coords_3d = np.linalg.inv(self.K) @ np.array([center_x * z, center_y * z, z])

                 # Crear un mensaje DetectedObject
                obj_msg = DetectedObject()
                obj_msg.label = str(current_cls)
                obj_msg.confidence = confidence
                obj_msg.bbox = [float(x_min), float(y_min), float(x_max), float(y_max)]
                obj_msg.position = [coords_3d[0], coords_3d[1], coords_3d[2]]

                # Agregar el objeto al mensaje principal
                detected_objects_msg.objects.append(obj_msg)
        
                # Mostrar información
                #self.get_logger().info(f'Objeto detectado: Clase: {str(current_cls)}, Confianza: {confidence}, Posición 3D: {coords_3d}')

        # Publicar los objetos detectados
        self.detection_pub.publish(detected_objects_msg)

        # Mostrar información en el log
        for obj in detected_objects_msg.objects:
            self.get_logger().info(f"Objeto detectados: {obj.label}, Confianza: {obj.confidence}, Posición 3D: {obj.position}")

            # except:
            #     self.get_logger().error("ERROR in callback")
            
        # Calcular FPS
        start = time.time()
        fps = 1.0 / (start - self.last_time)
        self.last_time = start
        
        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (440,440), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.imshow("Detection in real time", annotated_frame)   # Muestra la imagen en una ventana
        cv2.waitKey(1)  # Refresca la ventana en cada frame
        
        
def main(args=None):
    rclpy.init(args=args)    # Inicializa el sistema de ROS2

    YOLO_subscriber = YOLOSubscriber()  # Instancia del nodo suscriptor

    rclpy.spin(YOLO_subscriber) # Mantiene el nodo en ejecución

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    YOLO_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
