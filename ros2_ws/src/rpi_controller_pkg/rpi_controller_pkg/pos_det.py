#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class pos_calculation(Node):
    def __init__(self):
        super().__init__('position_calculation')
        self.angle_prev = 0.0
        self.steering_angle_prev = 0.0
        self.wheel_radius_mm = 32.0  # Example wheel radius in mm
        self.omega = 0.0
        self.v = self.omega * self.wheel_radius_mm
        
        self.l = 400 
        self.lh = 200

        self.x = 0.0
        self.y = 0.0

        self.sub_encoder_readings = self.create_subscription(Pose,'/encoder_readings',self.callback,10)
        self.steering_sub = self.create_subscription(Twist,'/bicycle_controller/cmd_vel',self.steering_callback,10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def steering_callback(self, msg):
        steering_angle = msg.angular.z
        self.steering_angle_prev = steering_angle

    def destroy_node(self):
        self.get_logger().info("Node stopped.")
        super().destroy_node()

    def callback(self, msg):
        instantaneous_distance_mm = msg.y

        dt = 0.01  # Assuming a fixed time step of 10ms

        # Calculate linear velocity (v) in mm/s
        v = instantaneous_distance_mm / (dt)  # rad/s

        # Simple steering angle estimation (this would depend on your vehicle's geometry)
        steering_angle = self.steering_angle_prev  # Placeholder for actual steering angle calculation

        # Calculate angular velocity (omega) in rad/s
        wheelbase_mm = 250.0  # Example wheelbase in mm

        if steering_angle != 0:
            # turning_radius_mm = wheelbase_mm / (steering_angle * 0.01745)  # Convert degrees to radians
            turning_radius_mm = self.wheel_radius_mm 
            self.omega = v / turning_radius_mm  # rad/s
        else:
            self.omega = 0.0

        self.get_logger().info(f'Linear Velocity (v): {v:.2f} mm/s, Angular Velocity (omega): {self.omega:.4f} rad/s')

        self.angle_prev = self.angle_prev + (self.omega * dt)
        self.steering_angle_prev = steering_angle

    def timer_callback(self):
        dt = 0.01
        x_dot = (self.v) * math.cos(self.angle_prev) - (self.v * self.steering_angle_prev * (1/ self.l)) * self.lh * math.sin(self.angle_prev)
        y_dot = (self.v) * math.sin(self.angle_prev) + (self.v * self.steering_angle_prev * (1/ self.l)) * self.lh * math.cos(self.angle_prev)
        
        self.x += x_dot * dt
        self.y += y_dot * dt

def main(args=None):
    rclpy.init(args=args)
    node = pos_calculation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()