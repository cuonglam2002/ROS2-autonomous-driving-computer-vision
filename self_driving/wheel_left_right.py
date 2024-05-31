#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import serial
from threading import Lock
from geometry_msgs.msg import Twist


class MinimalSubscriber(Node):
    r = 0.066 #radius off wheel
    b = 0.287 #length of body
    def __init__(self):
        super().__init__('vel_wheel_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',       
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.velocity=Twist()
        
        self.declare_parameter('serial_port', value="/dev/ttyUSB1")
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=57600)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value

        if (self.debug_serial_cmds):
            print("Serial debug enabled")
            
        self.mutex = Lock() #  synchronization primitive used to control access to a shared resource by multiple threads

        # Open serial comms
        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.connection}")



    def listener_callback(self, msg):
        self.velocity = msg
        
        V_linear = self.velocity.linear.x         
        w_angular = self.velocity.angular.z
        # V_linear = 0.1       
        # w_angular = -0.5
        V = V_linear
        W = w_angular
        W_wheel_left = round(((V - W * MinimalSubscriber.b / 2) / MinimalSubscriber.r) * 19.0986, 4)
        W_wheel_right = round(((V + W * MinimalSubscriber.b / 2) / MinimalSubscriber.r) * 19.0986, 4)
        self.get_logger().info(f"v {W_wheel_left} {W_wheel_right}")
        self.send_command(f"v {W_wheel_left} {W_wheel_right}")
        # self.send_command(f"v 20 20")
        # print(W_wheel_right, W_wheel_left)
        
    # def motor_command_callback(self, motor_command):
    #     self.send_command(f"v {int(motor_command.linear)} {int(motor_command.angular)}")

    # Utility functions
    def send_command(self, cmd_string):
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.connection.write(cmd_string.encode("utf-8"))
            if(self.debug_serial_cmds):
                print("Sent: " + cmd_string)
            # c = ''
            # value = ''
            # while c != '\r':
            #  c = self.connection.read(1).decode("utf-8")
            #  if (c == ''):
            #     print("Error: Serial timeout on command: " + cmd_string)
            #     return ''
            # value += c

            # value = value.strip('\r')

            # if (self.debug_serial_cmds):
            #     print("Received: " + value)
            # return value
        finally:
            self.mutex.release()

    def close_connection(self):
        self.send_command("v 0 0")
        self.connection.close()

    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try: 
        rclpy.spin(minimal_subscriber)
    finally:
        minimal_subscriber.close_connection()
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()