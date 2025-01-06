import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Declare a parameter for the topic name
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('width', 1080)
        self.declare_parameter('height', 720)

        # Set up the subscription
        self.subscription = self.create_subscription(Image, self.get_param('camera_topic'), self.image_callback, 1)
        self.subscription  # Prevent unused variable warning
        
        # Initialize CvBridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()
        self.get_logger().info(f"Subscribed: {self.get_param('camera_topic')}")

    def get_param(self, name):
        return self.get_parameter(name).value

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize the image
            width = self.get_param('width')
            height = self.get_param('height')
            cv_image = cv2.resize(cv_image, (width, height))

            # Display the image
            cv2.imshow('Camera Feed', cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
