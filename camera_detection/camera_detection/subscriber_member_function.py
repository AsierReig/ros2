import rclpy    # Biblioteca de ROS para Python
from rclpy.node import Node     # Base para crear nodos en ROS2
from sensor_msgs.msg import Image   # Tipo de mensaje que se espera (una imagen)
import cv2      # Para convertir las imágenes de ROS2 a OpenCV
from cv_bridge import CvBridge  # Librería para procesar imágenes en Python
from ultralytics import YOLO
import time
import math

class YOLOSubscriber(Node):

    def __init__(self):
        super().__init__('YOLO_subscriber') # Nombre del nodo
        # Crea el suscriptor que escucha el topic
        self.subscription = self.create_subscription(
            Image, # Tipo de mensaje que esperas
            '/camera/color/image_raw', # Nombre del topic
            self.listener_callback, # Funcion que se ejecuta cuando recibe un mensaje
            10) # Cola de mensajes
        self.subscription  # prevent unused variable warning
        self.br = CvBridge() # Instancia para convertir de ROS a OpenCV
        self.frame = None   # Inicializar el frame como None para evitar errores antes de llegar la primera Imagen

         # Cargar el modelo YOLO entrenado
        self.model = YOLO('/home/openheimer/Repositorios/rai_yolo_object_detection/Modelo/best.pt')


    def listener_callback(self, msg):

        start = time.time()

        # Este es el *callback* que se ejecuta cada vez que se recibe un mensaje
        self.frame = msg.data

        self.get_logger().info('Recibiendo imagen')

        # Convertir la imagen recibida en un formato usable por OpenCV
        current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")

        # Ejecutar detección con el modelo YOLO
        results = self.model(current_frame)

        # Dibujar las bounding boxes y etiquetas sobre la imagen
        boxes = results[0].boxes

        for box in boxes:
            try:
                #For bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1),int(y1),int(x2),int(y2)
                bbox = x1, y1, x2, y2

                #For classname
                cls = int(box.cls[0])
                #List with the classes of the model
                classNames =  ['Bottle','Cup','Spoon'] 
                curr_class = classNames[cls]

                #For id object
                id = box.id.int().item()
                
                # cv2.imshow('Detected object id: '+ str(id) + ' & Class: ' + str(curr_class), current_frame[bbox[1]:bbox[3], bbox[0]:bbox[2],:])
                
                # print(bbox, cls, id)

            except:
                pass
        
        end = time.time()
        fps = math.ceil(1 / (end - start))
        
        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        cv2.putText(annotated_frame, 'FPS: '+ str(fps), (440,440), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 3)
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
