import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
import numpy as np
from PIL import Image

import os
os.environ["PYTORCH_ENABLE_MPS_FALLBACK"] = "1"

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image_compressed',
            self.image_callback,
            qos_profile
        )
        self.bridge = CvBridge()

        # Configurar ventana OpenCV
        cv2.namedWindow('Depth Estimation', cv2.WINDOW_NORMAL)

        # Cargar el modelo de estimación de profundidad
        self.image_processor = AutoImageProcessor.from_pretrained("LiheYoung/depth-anything-small-hf")
        self.depth_model = AutoModelForDepthEstimation.from_pretrained("LiheYoung/depth-anything-small-hf")

        self.get_logger().info("Modelo de estimación de profundidad cargado correctamente.")

    def image_callback(self, msg):
        try:
            # Convertir la imagen ROS a OpenCV
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            
            # Convertir la imagen OpenCV a PIL
            pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

            # Procesar la imagen con el modelo
            inputs = self.image_processor(images=pil_image, return_tensors="pt")
            with torch.no_grad():
                outputs = self.depth_model(**inputs)

            # Postprocesar el resultado para ajustarlo al tamaño original
            post_processed_output = self.image_processor.post_process_depth_estimation(
                outputs,
                target_sizes=[(pil_image.height, pil_image.width)],
                # interpolation="bilinear"  # Cambiar de 'bicubic' a 'bilinear'
            )


            # Obtener la profundidad predicha
            predicted_depth = post_processed_output[0]["predicted_depth"]
            print("PREDICTED DEPTH: ", predicted_depth)
            print("PREDICTED DEPTH SHAPE: ", predicted_depth.shape)

            depth = predicted_depth * 255 / predicted_depth.max()
            depth = depth.detach().cpu().numpy()

            # Mostrar el mapa de profundidad con OpenCV
            depth_colored = cv2.applyColorMap(depth.astype("uint8"), cv2.COLORMAP_JET)
            cv2.imshow('Depth Estimation', depth_colored)

            # Finalizar si se presiona 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Finalizando nodo...")
                self.destroy_node()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error al procesar la imagen: {e}')

    def __del__(self):
        cv2.destroyAllWindows()


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


if __name__ == '__main__':
    main()
