
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from yolov5 import YOLOv5  # Asegúrate de que el módulo de YOLOv5 esté disponible

# Cargar el modelo YOLOv5 (puedes usar "yolov5s", "yolov5m", "yolov5l", "yolov5x")
model = YOLOv5("yolov5s.pt", device="cuda" if torch.cuda.is_available() else "cpu")

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convierte el mensaje ROS a una imagen de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Realiza la detección con YOLO
            results = model.predict(cv_image)
            
            # Muestra los resultados de la detección
            annotated_frame = results.render()[0]  # Dibuja los cuadros de los objetos detectados

            # Muestra la imagen con las anotaciones en una ventana de OpenCV
            cv2.imshow('Camera Feed with YOLO', annotated_frame)
            cv2.waitKey(1)  # Espera 1 ms para refrescar la ventana

        except Exception as e:
            self.get_logger().error(f'Error al convertir la imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Cierra la ventana de OpenCV al finalizar

if __name__ == '__main__':
    main()
