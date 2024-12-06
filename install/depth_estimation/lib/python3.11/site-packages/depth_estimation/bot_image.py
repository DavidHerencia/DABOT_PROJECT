import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist

class BotImage(Node):
    def __init__(self):
        super().__init__('image_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(
            CompressedImage, 
            'camera/image_compressed', 
            qos_profile
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo acceder a la cámara')
            exit(1)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'JPEG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.timer = self.create_timer(0.1, self.timer_callback)

        # Inicializar velocidades
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            resized_image = cv2.resize(frame, (320, 240))
            msg = self.bridge.cv2_to_compressed_imgmsg(resized_image, dst_format='jpeg')
            self.publisher_.publish(msg)
            self.get_logger().info('Publicando frame de la cámara...')
        else:
            self.get_logger().warning('No se pudo capturar el frame')

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.get_logger().info(f'Recibiendo comando de velocidad: linear={self.linear_velocity}, angular={self.angular_velocity}')
        self.move_bot()

    def move_bot(self):
        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = self.angular_velocity
        self.cmd_vel_pub.publish(cmd)

    def __del__(self):
        self.cap.release()

def main(args=None):    
    rclpy.init(args=args)
    node = BotImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()