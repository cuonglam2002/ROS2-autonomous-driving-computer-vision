
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(0)
        print("success conect to camera")

    def publish_image(self):
        while True:
            ret, img = self.capture.read()
            if ret:
                img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.publisher.publish(img_msg)
                
                

            else:
                self.get_logger().error("Could not grab a frame!")
                break

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    try:
        image_publisher.publish_image()
    except KeyboardInterrupt:
        print("Shutting down!")
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
