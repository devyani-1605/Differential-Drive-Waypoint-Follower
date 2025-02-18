#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf_transformations import euler_from_quaternion

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Declare PID parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp_linear', 0.5),
                ('ki_linear', 0.0),
                ('kd_linear', 0.1),
                ('kp_angular', 1.0),
                ('ki_angular', 0.0),
                ('kd_angular', 0.2),
                ('distance_threshold', 0.1)
            ]
        )
        
        # Declare waypoint parameters (Ensure they are recognized)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoint_1_x', 0.0),
                ('waypoint_1_y', 0.0),
                ('waypoint_2_x', 0.0),
                ('waypoint_2_y', 0.0)
            ]
        )
        
        # Read waypoints from parameters
        self.waypoints = []
        index = 1
        while True:
            param_x = f'waypoint_{index}_x'
            param_y = f'waypoint_{index}_y'
            if self.has_parameter(param_x) and self.has_parameter(param_y):
                x = self.get_parameter(param_x).value
                y = self.get_parameter(param_y).value
                self.waypoints.append((x, y))
                index += 1
            else:
                break
        
        if not self.waypoints:
            self.get_logger().error('No waypoints provided!')
            return
        
        # PID controllers
        self.linear_pid = PIDController(
            self.get_parameter('kp_linear').value,
            self.get_parameter('ki_linear').value,
            self.get_parameter('kd_linear').value
        )
        self.angular_pid = PIDController(
            self.get_parameter('kp_angular').value,
            self.get_parameter('ki_angular').value,
            self.get_parameter('kd_angular').value
        )
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_waypoint_index = 0
        self.reached_all_waypoints = False
        
        # Control loop timer
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
    def odom_callback(self, msg):
        if self.reached_all_waypoints:
            return
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            
    def control_loop(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            return
        
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        angle_error = target_angle - self.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        angular_vel = self.angular_pid.update(angle_error)
        linear_vel = self.linear_pid.update(distance)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_vel)
        
        if distance < self.get_parameter('distance_threshold').value:
            self.get_logger().info(f'Waypoint reached: ({target_x}, {target_y})')
            self.current_waypoint_index += 1
            self.linear_pid.reset()
            self.angular_pid.reset()
            
        def stop_robot(self):
            """ Publishes zero velocity to stop the robot """
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
            
      
            
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()
        
    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        
    def update(self, error, dt=0.1):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        
        return (self.kp * error + 
                self.ki * self.integral + 
                self.kd * derivative)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

