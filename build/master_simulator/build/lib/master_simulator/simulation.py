#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, PoseStamped, Point, Quaternion
import numpy as np
from simulator.auv_dynamics import dynamics
import tf_transformations

class SystemModel(Node):
    def __init__(self):
        super().__init__('system_model')

        # Initialize state variables
        self.x = np.zeros(12)
        self.forces = np.zeros(3)
        self.moments = np.zeros(3)
        self.dt = 0.1  # Time step for Euler integration

        self.tau_RB = np.zeros(6)

        # Subscribe to forces and moments
        self.force_subscriber = self.create_subscription(
            Wrench,
            'forces_topic',
            self.forces_callback,
            10
        )

        # Create a timer to update the simulation state
        self.timer = self.create_timer(self.dt, self.update_state)

        # Publish the state
        self.state_publisher = self.create_publisher(PoseStamped, 'state_topic', 10)

    def forces_callback(self, msg):
        # Handle forces and moments from the message
        self.forces = np.array([msg.force.x, msg.force.y, msg.force.z])
        self.moments = np.array([msg.torque.x, msg.torque.y, msg.torque.z])
        self.tau_RB = np.concatenate([self.forces, self.moments])

    def simulation(self):
        self.x_dot = dynamics(self.x, self.tau_RB)

    def euler_integration(self):
        self.x = self.x + self.x_dot * self.dt

    def update_state(self):
        self.simulation()
        self.euler_integration()

        # Euler to quaternions
        roll   = self.x[3]
        pitch  = self.x[4]
        yaw    = self.x[5]
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        # Publish the updated state
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position = Point(x=self.x[0], y=self.x[1], z=self.x[2])
        pose_msg.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.state_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    system_model = SystemModel()

    rclpy.spin(system_model)

    system_model.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()