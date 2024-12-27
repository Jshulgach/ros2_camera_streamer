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

        # Set up the subscription
        topic = self.get_parameter('camera_topic').value
        self.subscription = self.create_subscription(Image, topic, self.image_callback, 1)
        self.subscription  # Prevent unused variable warning
        
        # Initialize CvBridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()
        self.get_logger().info(f"Subscribed to topic: {topic}")

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
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
