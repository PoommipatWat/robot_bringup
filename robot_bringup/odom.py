import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, TransformStamped
import tf2_ros
import numpy as np

class Odom(Node):
    def __init__(self):
        super().__init__('odom')
        self.publisher_ = self.create_publisher(Odometry, 'odom/raw', 10)
        self.timer = self.create_timer(0.01, self.odom_callback)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel/vel',
            self.sub_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.last_time = self.get_clock().now()
        self.pos = Pose2D()
        self.tf_boardcast = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("Odom node has been started")
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

    def sub_callback(self, msg):
        # Extract velocities
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = -msg.angular.z

    def odom_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        
        # Update position and heading
        delta_x = (self.vx * np.cos(self.pos.theta) - self.vy * np.sin(self.pos.theta)) * dt
        delta_y = (self.vx * np.sin(self.pos.theta) + self.vy * np.cos(self.pos.theta)) * dt
        delta_th = self.vth * dt
        
        self.pos.x += delta_x
        self.pos.y += delta_y
        self.pos.theta += delta_th
        
        # Normalize heading after update
        self.pos.theta = self.normalize_angle(self.pos.theta)
        
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.pos.x
        odom.pose.pose.position.y = self.pos.y
        odom.pose.pose.position.z = 0.0
        quaternion = self.quaternion_from_euler(0, 0, self.pos.theta)
        odom.pose.pose.orientation.w = quaternion[0]
        odom.pose.pose.orientation.x = quaternion[1]
        odom.pose.pose.orientation.y = quaternion[2]
        odom.pose.pose.orientation.z = quaternion[3]

        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001
        
        self.publisher_.publish(odom)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pos.x
        t.transform.translation.y = self.pos.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = quaternion[0]
        t.transform.rotation.x = quaternion[1]
        t.transform.rotation.y = quaternion[2]
        t.transform.rotation.z = quaternion[3]
        
        #self.tf_boardcast.sendTransform(t)
        
        self.last_time = current_time

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr
        
        return q

def main():
    rclpy.init()
    node = Odom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()