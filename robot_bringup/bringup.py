import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
import numpy as np

class Omni_kinematics:
    def __init__(self, Distance, Radius, initial_theta=0) -> None:
        self.Distance = Distance
        self.initial_theta = initial_theta
        self.Radius = Radius

    def inverse_omni(self, vx, vy, w, max_rpm, filter=True):
        instance = np.array([
            [-np.sin(self.initial_theta), np.cos(self.initial_theta), self.Distance],
            [-np.sin(np.pi/4 - self.initial_theta), -np.cos(np.pi/4 - self.initial_theta), self.Distance],
            [np.sin(np.pi/4 + self.initial_theta), -np.cos(np.pi/4 + self.initial_theta), self.Distance],
            [np.sin(self.initial_theta), -np.cos(self.initial_theta), self.Distance]
        ])
        v1, v2, v3, v4 = instance @ np.array([vx, vy, w])

        omega1 = v1 / self.Radius
        omega2 = v2 / self.Radius
        omega3 = v3 / self.Radius
        omega4 = v4 / self.Radius

        rpm1 = omega1 * 60 / (2 * np.pi)
        rpm2 = omega2 * 60 / (2 * np.pi)
        rpm3 = omega3 * 60 / (2 * np.pi)
        rpm4 = omega4 * 60 / (2 * np.pi)

        if filter:
            max_rpm_value = max(abs(rpm1), abs(rpm2), abs(rpm3), abs(rpm4))
            if max_rpm_value > max_rpm:
                scale_factor = max_rpm / max_rpm_value
                rpm1 *= scale_factor
                rpm2 *= scale_factor
                rpm3 *= scale_factor
                rpm4 *= scale_factor

        return rpm1, rpm2, rpm3, rpm4

    def forward_omni(self, rpm1, rpm2, rpm3, rpm4):
        omega1 = rpm1 * 2 * np.pi / 60
        omega2 = rpm2 * 2 * np.pi / 60
        omega3 = rpm3 * 2 * np.pi / 60
        omega4 = rpm4 * 2 * np.pi / 60

        v1 = omega1 * self.Radius
        v2 = omega2 * self.Radius
        v3 = omega3 * self.Radius
        v4 = omega4 * self.Radius

        instance = np.array([
            [-np.sin(self.initial_theta), np.cos(self.initial_theta), self.Distance],
            [-np.sin(np.pi/4 - self.initial_theta), -np.cos(np.pi/4 - self.initial_theta), self.Distance],
            [np.sin(np.pi/4 + self.initial_theta), -np.cos(np.pi/4 + self.initial_theta), self.Distance],
            [np.sin(self.initial_theta), -np.cos(self.initial_theta), self.Distance]
        ])
        vx, vy, w = np.linalg.pinv(instance) @ np.array([v1, v2, v3, v4])
        return vx, vy, w

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
        self.subscriptions

        self.subscriptions2 = self.create_subscription(
            Twist, 
            'rpm', 
            self.cmd_vel_callback2, 
            qos_profile
        )
        self.subscriptions2
        self.rpm = Twist()

        self.wheel_diameter = 0.082
        self.wheel_distance = 182.0/1000.0
        self.max_rpm = 165
        self.factor_rpm = 0.85

        self.omni = Omni_kinematics(self.wheel_distance, self.wheel_diameter/2)

        self.cmd_vel = Twist()

        self.get_logger().info("Robot Bringup node has been started")

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def cmd_vel_callback2(self, msg):
        vx, vy, w = self.omni.forward_omni(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x)
        self.rpm.linear.x = vx
        self.rpm.linear.y = vy
        self.rpm.angular.z = w

    def timer_callback(self):
        rpm = self.omni.inverse_omni(self.cmd_vel.linear.x, self.cmd_vel.linear.y, -self.cmd_vel.angular.z, self.max_rpm*self.factor_rpm, filter=True)
        rpm_msg = Twist()
        rpm_msg.linear.x = rpm[0]
        rpm_msg.linear.y = rpm[1]
        rpm_msg.linear.z = rpm[2]
        rpm_msg.angular.x = rpm[3]  # Added fourth wheel RPM
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