import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge


# En el publisher
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # self.publisher_ = self.create_publisher(Image, 'camera/image_raw', qos_profile)        
        # Usar CompressedImage en lugar de Image
        self.publisher_ = self.create_publisher(
            CompressedImage, 
            'camera/image_compressed', 
            qos_profile
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publica cada 0.1s (10 Hz)
        # self.timer = self.create_timer(0.2, self.timer_callback)  # Publica cada 0.2 segundos (5 Hz)

        self.bridge = CvBridge()

        # Configuración de la cámara
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)  # Cambia el índice si usas otra cámara
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo acceder a la cámara')
            exit(1)

        # Configurar la cámara a MJPEG y establecer una resolución específica
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPEG
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Ancho de la imagen
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Alto de la imagen
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # FPS (opcional, si es necesario)

        # Verificar si la configuración se aplicó correctamente
        if (self.cap.get(cv2.CAP_PROP_FOURCC) != cv2.VideoWriter_fourcc(*'MJPG') or
                self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) != 640 or
                self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) != 480):
            self.get_logger().warn('La configuración de la cámara puede no haberse aplicado correctamente')

        # Imprimir información de la cámara para verificar
        print("Cámera abierta correctamente")
        print("Ancho:", self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        print("Alto:", self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print("FPS:", self.cap.get(cv2.CAP_PROP_FPS))
        print("FOURCC:", self.cap.get(cv2.CAP_PROP_FOURCC))


        # # cap = cv2.VideoCapture(2)
        # if self.cap.isOpened():
        #     print("Cámera abierta correctamente")
        #     print("Ancho:", self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        #     print("Alto:", self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #     print("FPS:", self.cap.get(cv2.CAP_PROP_FPS))
        #     print("FOURCC:", self.cap.get(cv2.CAP_PROP_FOURCC))
        # else:
        #     print("No se pudo abrir la cámara.")
        #     self.get_logger().error('No se pudo acceder a la cámara')
        #     exit(1)
        # # Forzar formato MJPEG y resolución
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPEG
        # # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Ancho de la imagen
        # # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Alto de la imagen
        # # self.cap.set(cv2.CAP_PROP_FPS, 30)  # FPS (opcional, si es necesario)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Considerar usar formato comprimido
            resized_image = cv2.resize(frame, (320, 240))
            msg = self.bridge.cv2_to_compressed_imgmsg(resized_image) 
            self.publisher_.publish(msg)




            # Convierte la imagen OpenCV a un mensaje ROS 2
            # ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            # self.publisher_.publish(ros_image)
            self.get_logger().info('Publicando frame de la cámara...')
        else:
            self.get_logger().warning('No se pudo capturar el frame')

    def __del__(self):
        self.cap.release()  # Libera la cámara al finalizar el nodo

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
