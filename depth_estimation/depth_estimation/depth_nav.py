import rclpy
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
        
        self.min_safe_depth = 1.0
        self.linear_speed = 0.15
        self.angular_speed = 0.5
        self.similarity_threshold = 0.1  # Umbral para considerar profundidades similares

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
            
            avg_depths = {
                'left': np.mean(left_region),
                'center': np.mean(center_region),
                'right': np.mean(right_region)
            }
            
            cmd = Twist()
            if avg_depths['center'] > self.min_safe_depth:
                cmd.linear.x = self.linear_speed
                if abs(avg_depths['right'] - avg_depths['left']) < self.similarity_threshold:
                    cmd.angular.z = 0.0
                    print("MOVING FORWARD")
                elif avg_depths['right'] < avg_depths['left']:
                    cmd.angular.z = -0.25
                    print("MOVING TO THE RIGHT")
                else:
                    cmd.angular.z = 0.25
                    print("MOVING TO THE LEFT")
            else:
                cmd.linear.x = 0.0
                if avg_depths['right'] < self.min_safe_depth:
                    cmd.angular.z = -self.angular_speed
                elif avg_depths['left'] < self.min_safe_depth:
                    cmd.angular.z = self.angular_speed
                else:
                    cmd.angular.z = self.angular_speed  # Gira en el lugar si no hay espacio seguro
            
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