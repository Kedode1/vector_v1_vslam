#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class ArduinoController(Node):

    def __init__(self):
        super().__init__('arduino_controller')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        self.port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        #Serial setup, if it's the only port used it is by default /dev/ttyACM0, if not use rpi terminal to make sure by doing >> ls /dev/tty*
        self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
        time.sleep(2)  # Give Arduino time to reset
        self.subscription = self.create_subscription(Twist,'/bicycle_controller/cmd_vel',self.callback,10)
        self.encoder_pub = self.create_publisher(Pose,'/encoder_readings',10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.instantaneous_distance_mm = 0
        self.distance_mm = 0

    def callback(self, msg):

        linear_x = msg.linear.x  # Forward/Backward speed
        angular_z = msg.angular.z  # Steering angle

        # Map linear_x and angular_z to motor speed and steering commands
        motor_speed = int(round(linear_x)) # Scale to -10 to 10
        steering_angle = int(angular_z)  # Scale to -10 to 10

        # Create command string
        command = f"M:{motor_speed},S:{steering_angle}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f'Sent command: {command.strip()}')

    
    def timer_callback(self):

        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            self.get_logger().info(f'Received from Arduino: {line}')
            command_separation = line.split(',')
            self.distance_mm = float(command_separation[0].split(':')[1].strip())
            self.instantaneous_distance_mm = float(command_separation[1].split(':')[1].strip())
            msg = Pose()
            msg.x = self.distance_mm
            msg.y = self.instantaneous_distance_mm
            self.encoder_pub.publish(msg)
            

    def destroy_node(self):
        
        """Graceful shutdown"""
        self.ser.write(b'M:0,S:95\n')
        self.ser.close()
        self.get_logger().info("Serial closed. Node stopped.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()