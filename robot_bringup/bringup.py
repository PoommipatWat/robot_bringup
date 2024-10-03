import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
import numpy as np

class Omni_kinematics:
    def __init__(self, R, l1, l2):
        self.R = R  # wheel radius in meters
        self.l1 = l1  # distance between wheel axis and body center (x-axis) in meters
        self.l2 = l2  # distance between wheel axis and body center (y-axis) in meters
        self.update_matrix()

    def update_matrix(self):
        self.forward_matrix = (1/self.R) * np.array([
            [1, -1, -(self.l1 + self.l2)],
            [1, 1, (self.l1 + self.l2)],
            [1, 1, -(self.l1 + self.l2)],
            [1, -1, (self.l1 + self.l2)]
        ])
        
        self.inverse_matrix = (self.R/4) * np.array([
            [1, 1, 1, 1],
            [-1, 1, 1, -1],
            [-(1/(self.l1 + self.l2)), (1/(self.l1 + self.l2)), -(1/(self.l1 + self.l2)), (1/(self.l1 + self.l2))]
        ])

    def forward_kinematics(self, vx, vy, omega):
        # vx, vy in m/s, omega in rad/s
        body_velocity = np.array([vx, vy, omega])
        wheel_angular_velocities = np.dot(self.forward_matrix, body_velocity)
        # Convert to RPM
        wheel_rpm = wheel_angular_velocities * (60 / (2 * np.pi))
        return wheel_rpm.tolist()  # Return as a list

    def inverse_kinematics(self, w1, w2, w3, w4):
        # w1, w2, w3, w4 in rad/s
        wheel_angular_velocities = np.array([w1, w2, w3, w4])
        body_velocity = np.dot(self.inverse_matrix, wheel_angular_velocities)
        return body_velocity.tolist()  # Return as a list [vx, vy, omega]

class Bringup(Node):
    def __init__(self):
        super().__init__('bringup')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel/rpm', 10)
        self.publisher2_ = self.create_publisher(Twist, 'cmd_vel/vel', 10)
        self.timer_ = self.create_timer(0.01, self.timer_callback)
        self.subscriptions_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.subscriptions_2 = self.create_subscription(Twist, 'rpm', self.cmd_vel_callback2, qos_profile)

        self.rpm = Twist()
        self.wheel_radius = 0.082  # 82 mm radius
        self.l1 = 0.13  # Distance from center to wheel axis in x direction
        self.l2 = 0.13  # Distance from center to wheel axis in y direction
        self.max_rpm = 165.0
        self.factor_rpm = 0.85
        self.omni = Omni_kinematics(R=self.wheel_radius, l1=self.l1, l2=self.l2)
        self.cmd_vel = Twist()
        self.get_logger().info("Robot Bringup node has been started with updated kinematics")

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def cmd_vel_callback2(self, msg):
        # Convert RPM to rad/s
        w1 = -1 * msg.linear.x * (2 * np.pi / 60)
        w2 = msg.linear.y * (2 * np.pi / 60)
        w3 = -1 * msg.angular.x * (2 * np.pi / 60)
        w4 = msg.angular.y * (2 * np.pi / 60)

        vx, vy, w = self.omni.inverse_kinematics(w1, w2, w3, w4)

        # Apply scaling factor to correct for 10x overestimation
        scaling_factor = 1.0
        self.rpm.linear.x = vx * scaling_factor
        self.rpm.linear.y = vy * scaling_factor
        self.rpm.angular.z = w * scaling_factor

        self.get_logger().info(f"Input RPM: {msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.angular.x:.2f}, {msg.angular.y:.2f}")
        self.get_logger().info(f"Calculated velocities: vx={self.rpm.linear.x:.2f}, vy={self.rpm.linear.y:.2f}, w={self.rpm.angular.z:.2f}")

    def limit_rpm(self, rpm):
        max_allowed_rpm = self.max_rpm * self.factor_rpm
        limited_rpm = [min(max(r, -max_allowed_rpm), max_allowed_rpm) for r in rpm]
        return limited_rpm

    def timer_callback(self):
        rpm = self.omni.forward_kinematics(self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.angular.z)
        limited_rpm = self.limit_rpm(rpm)
        
        rpm_msg = Twist()
        rpm_msg.linear.x = -1*limited_rpm[0]
        rpm_msg.linear.y = limited_rpm[1]
        rpm_msg.angular.x = -1*limited_rpm[2]
        rpm_msg.angular.y = limited_rpm[3]
        self.publisher_.publish(rpm_msg)

        vel_msg = Twist()
        vel_msg.linear.x = self.rpm.linear.x
        vel_msg.linear.y = self.rpm.linear.y
        vel_msg.angular.z = self.rpm.angular.z
        self.publisher2_.publish(vel_msg)

def main():
    rclpy.init()
    node = Bringup()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()