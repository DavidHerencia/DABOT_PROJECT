import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
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
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.bridge = CvBridge()
        cv2.namedWindow('Depth Estimation', cv2.WINDOW_NORMAL)
        
        self.image_processor = AutoImageProcessor.from_pretrained("LiheYoung/depth-anything-small-hf")
        self.depth_model = AutoModelForDepthEstimation.from_pretrained("LiheYoung/depth-anything-small-hf")
        
        # Parámetros de control actualizados
        self.STOP_DISTANCE = 1.5  # Reducido para no detenerse tan pronto
        self.SLOW_DISTANCE = 0.90  # Reducido para mejor control
        self.MAX_SPEED = 0.18   # Aumentado para mayor velocidad forward
        self.MIN_SPEED = 0.05
        self.angular_speed = 0.5        # Reducido para giros más suaves
        self.angular_speed_obstacle = 0.5   # Reducido para evitar giros bruscos
        self.similarity_threshold = 1.5  # Reducido para mejor detección de diferencias

    # Mejorar el cálculo del speed_factor
    def calculate_speed_factor(self, distance):
        if distance > self.STOP_DISTANCE:
            return 0.0
        elif distance > self.SLOW_DISTANCE:
        # Función exponencial suavizada para mejor control
        # Normalizar la distancia entre 0 y 1
            normalized_dist = (distance - self.STOP_DISTANCE) / (self.SLOW_DISTANCE - self.STOP_DISTANCE)
        # Aplicar curva exponencial suave
            return (1 - math.exp(-3 * normalized_dist)) * 0.7
        else:
        # Limitar la velocidad máxima al 70% para mejor control
            return 0.7

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            
            inputs = self.image_processor(images=pil_image, return_tensors="pt")
            with torch.no_grad():
                outputs = self.depth_model(**inputs)
            
            post_processed_output = self.image_processor.post_process_depth_estimation(
                outputs,
                target_sizes=[(pil_image.height, pil_image.width)],
            )
            
            predicted_depth = post_processed_output[0]["predicted_depth"]
            depth = predicted_depth.detach().cpu().numpy()
            
            # Navigation logic
            height, width = depth.shape
            left_region = depth[:, :width//3]
            center_region = depth[:, width//3:2*width//3]
            right_region = depth[:, 2*width//3:]
            
            DEPTH_SCALE = 0.1  # Factor de conversión a metros

            min_depths = {
            'left': np.median(left_region[left_region > 0]) * DEPTH_SCALE,
            'center': np.median(center_region[center_region > 0]) * DEPTH_SCALE,
            'right': np.median(right_region[right_region > 0]) * DEPTH_SCALE
            }

            
            # Factor de velocidad basado en la distancia mínima general
            speed_factor = self.calculate_speed_factor(min(min_depths.values()))
            
            print("MIN DISTANCES - Left: {:.2f}, Center: {:.2f}, Right: {:.2f}".format(
                min_depths['left'], min_depths['center'], min_depths['right']))
            print("SPEED FACTOR: ", speed_factor)
            
            cmd = Twist()
            
            # Lógica de navegación mejorada
            # if min_depths['center'] > self.STOP_DISTANCE or min_depths['left'] > self.STOP_DISTANCE or min_depths['right'] > self.STOP_DISTANCE:
            if min_depths['center'] > self.STOP_DISTANCE:
                # Objeto muy cerca al frente - detenerse y girar hacia el lado más libre
                cmd.linear.x = 0.0
                if min_depths['right'] < min_depths['left']:
                    cmd.angular.z = -self.angular_speed_obstacle * 0.8
                    print("ROTATING RIGHT - BLOCKED CENTER")
                else:
                    cmd.angular.z = self.angular_speed_obstacle * 0.8
                    print("ROTATING LEFT - BLOCKED CENTER")
            elif min_depths['center'] > self.SLOW_DISTANCE:
                # Objeto cercano al frente - reducir velocidad y girar suavemente
                cmd.linear.x = self.MAX_SPEED * speed_factor * 0.5
                if min_depths['right'] < min_depths['left']:
                    cmd.angular.z = -self.angular_speed_obstacle * 0.30
                    print("TURNING RIGHT SLOWLY - OBJECT AHEAD")
                else:
                    cmd.angular.z = self.angular_speed_obstacle * 0.30
                    print("TURNING LEFT SLOWLY - OBJECT AHEAD")
            else:
                # Camino libre al frente - ajustar dirección basado en obstáculos laterales
                cmd.linear.x = self.MAX_SPEED * speed_factor
                if min_depths['right'] > self.SLOW_DISTANCE:
                    cmd.angular.z = self.angular_speed_obstacle * 0.45
                    print("ADJUSTING LEFT - RIGHT SIDE CLOSE")
                elif min_depths['left'] > self.SLOW_DISTANCE:
                    cmd.angular.z = -self.angular_speed_obstacle * 0.45
                    print("ADJUSTING RIGHT - LEFT SIDE CLOSE")
                else:
                    cmd.angular.z = 0.0
                    print("GOING STRAIGHT")

            self.cmd_vel_pub.publish(cmd)
            
            # Visualization
            depth_display = depth * 255 / depth.max()
            depth_colored = cv2.applyColorMap(depth_display.astype("uint8"), cv2.COLORMAP_JET)
            cv2.imshow('Depth Estimation', depth_colored)
            
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