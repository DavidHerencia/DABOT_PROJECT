import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FrameViewer(Node):
    def __init__(self):
        super().__init__('frame_viewer')
        self.subscription = self.create_subscription(Image, '/camera_frames', self.listener_callback, 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('Frame en tiempo real', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FrameViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
