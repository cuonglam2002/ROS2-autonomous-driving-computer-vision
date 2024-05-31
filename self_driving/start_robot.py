import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class startrobot(Node):
    def __init__(self):
        super().__init__('angle_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(2.0, self.publish_angle)
        self.angle = 0
        self.counter = 0  # Initialize counter to keep track of iterations

    def publish_angle(self):
        if self.counter == 0:  # Send pi/3 angle
            self.angle = math.pi / 20
        elif self.counter <= 2:  # Send -pi/3 twice
            self.angle = -math.pi / 20
        elif self.counter <= 4:  # Send pi/3 twice
            self.angle = math.pi / 20
        elif self.counter == 5:  # Send pi/3 twice
            self.angle = -math.pi / 20
        elif self.counter == 6:  # Send pi/3 twice
            self.angle = 0.0
        elif self.counter == 7:  # End the node
            self.timer.cancel()  # Stop the timer
            self.destroy_node()  # End the node
            
        twist_msg = Twist()
        twist_msg.angular.z = self.angle
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg)

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = startrobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
