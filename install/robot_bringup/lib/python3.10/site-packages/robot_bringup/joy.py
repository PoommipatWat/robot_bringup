import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import numpy as np

class Omni_kinematics:
    def __init__(self, Distance, Radius, initial_theta=0) -> None:
        self.Distance = Distance
        self.initial_theta = initial_theta
        self.Radius = Radius

    def inverse_omni(self, vx, vy, w, max_rpm, filter=True): # max_rpm for 2.5 m/s at 41 mm self.Radius
        instance = np.array([[-np.sin(self.initial_theta), np.cos(self.initial_theta), self.Distance],
                            [-np.sin(np.pi/3 - self.initial_theta), -np.cos(np.pi/3 - self.initial_theta), self.Distance],
                            [np.sin(np.pi/3 + self.initial_theta), -np.cos(np.pi/3 + self.initial_theta), self.Distance]])
        v1, v2, v3 = instance @ np.array([vx, vy, w])

        # Convert linear velocities to angular velocities (radians per second)
        omega1 = v1 / self.Radius
        omega2 = v2 / self.Radius
        omega3 = v3 / self.Radius

        # Convert angular velocities to revolutions per minute (rpm)
        rpm1 = omega1 * 60 / (2 * np.pi)
        rpm2 = omega2 * 60 / (2 * np.pi)
        rpm3 = omega3 * 60 / (2 * np.pi)

        if filter:
            if abs(rpm1) > max_rpm or abs(rpm2) > max_rpm or abs(rpm3) > max_rpm:
                scale_factor = max_rpm / max(abs(rpm1), abs(rpm2), abs(rpm3))
                rpm1 *= scale_factor
                rpm2 *= scale_factor
                rpm3 *= scale_factor

        return rpm1, rpm2, rpm3

    def forward_omni(self, rpm1, rpm2, rpm3): # R in meters
        # Convert RPM to radians per second
        omega1 = rpm1 * 2 * np.pi / 60
        omega2 = rpm2 * 2 * np.pi / 60
        omega3 = rpm3 * 2 * np.pi / 60

        # Convert angular velocities to linear velocities
        v1 = omega1 * self.Radius
        v2 = omega2 * self.Radius
        v3 = omega3 * self.Radius

        instance = np.array([[-np.sin(self.initial_theta), np.cos(self.initial_theta), self.Distance],
                            [-np.sin(np.pi/3 - self.initial_theta), -np.cos(np.pi/3 - self.initial_theta), self.Distance],
                            [np.sin(np.pi/3 + self.initial_theta), -np.cos(np.pi/3 + self.initial_theta), self.Distance]])
        vx, vy, w = np.linalg.inv(instance) @ np.array([v1, v2, v3])
        return vx, vy, w


class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel/rpm', 10)

        self.wheel_dimeter = 0.082
        self.wheel_distance = 0.10594374972
        self.max_rpm = 400
        self.factor_rpm = 0.85

        self.omni = Omni_kinematics(self.wheel_distance, self.wheel_dimeter/2)

    def joy_callback(self, msg):
        twist = Twist()
        # Mapping axes from joystick to cmd_vel
        x = -msg.axes[1]*2.0
        # x = 0
        y = -msg.axes[0]*2.0
        # y = 0
        z = -msg.axes[2]

        self.omni.inverse_omni(x, y, z, 3500, filter=True)

        twist.linear.x, twist.linear.y, twist.linear.z = self.omni.inverse_omni(x, y, z, 3500, filter=True)

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    joy_to_cmd_vel = JoyToCmdVel()

    rclpy.spin(joy_to_cmd_vel)

    joy_to_cmd_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
