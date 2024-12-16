import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')

        # Declare parameters for the camera topic name and frame rate
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('frame_rate', 20)
        self.declare_parameter('video_device', '/dev/video0')
        self.camera_topic = self.get_parameter('camera_topic').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.video_device = self.get_parameter('video_device').value

        self.publisher_ = self.create_publisher(Image, self.camera_topic, self.frame_rate)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.video_device)

        if not self.cap.isOpened():
            self.get_logger().error('Unable to open camera')
            exit()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
