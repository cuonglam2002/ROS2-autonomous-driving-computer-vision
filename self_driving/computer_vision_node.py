import cv2
from geometry_msgs.msg import Twist
import datetime
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from .drive_BOT import Car
import rclpy
from std_msgs.msg import String  # Import message type for traffic sign

class Video_feed_in(Node):
    def __init__(self):

        super().__init__('video_subscriber')
        self.publisher_img_read = self.create_publisher(Image, '/imgread', 10)  # Create a publisher for the processed image
        self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(String, 'traffic_sign', self.traffic_sign_callback, 10)
        timer_period = 0.1;self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        
        
        # self.out = cv2.VideoWriter('/home/an/ros2_ws/src/self_driving/self_driving/output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

        self.velocity = Twist()
        self.bridge   = CvBridge() # converting ros images to opencv data
        self.Car = Car()
        self.p_err = 0
        self.vel =''
        # ngay_gio_hien_tai = datetime.datetime.now()
        # ten_tap_tin = '/home/an/ros2_ws/src/self_driving/self_driving/output{}.avi'.format(ngay_gio_hien_tai.strftime("%Y-%m-%d_%H-%M-%S"))
        # Tạo video writer với tên tập tin đã được định dạng
        # self.out = cv2.VideoWriter(ten_tap_tin, cv2.VideoWriter_fourcc('M','J','P','G'), 30, (320,240))
    def traffic_sign_callback(self, msg):
        # Xử lý dữ liệu nhận được từ topic 'traffic_sign'
        self.vel =  msg.data
        
        
    def send_cmd_vel(self):
        self.publisher.publish(self.velocity)
        
    def process_data(self, data): 
        
        
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
        # self.out.write(frame)# write the frames to a video
        # Display the image
       
        # cv2.waitKey(1)  # You need this line to keep the window open

        # text, frame = det_traffic_light(frame)
        # cv2.imshow('Image', frame)
        # self.get_logger().info(text)
        text = ""

        self.p_err,Angle,Speed,img = self.Car.drive_car(frame, self.p_err, self.vel)

        self.velocity.angular.z = Angle
        self.velocity.linear.x = Speed    
        
        # self.velocity.linear.x = self.vel    

        # width = 600
        # height = 400
        # dim = (width, height)
        
        # # resize image
        # img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        # self.out.write(img)# write the frames to a video
        # self.publisher.publish(self.velocity)
        cv2.imshow("Frame",img)
        cv2.waitKey(1)

        
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher_img_read.publish(img_msg)
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
	main()