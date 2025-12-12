import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class FigureEightMover(Node):
    def __init__(self):
        super().__init__('figure_eight_movement')
        self.declare_parameter('scale', 0.5)      # a: path scale (meters)
        self.declare_parameter('frequency', 0.5)  # k: phase speed (rad/s)
        self.declare_parameter('rate', 10.0)      # Hz

        self.scale = self.get_parameter('scale').value
        self.frequency = self.get_parameter('frequency').value
        self.rate = self.get_parameter('rate').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for checking clock
        self.clock_checker_timer = self.create_timer(0.1, self._check_clock)

        self.movement_timer = None
        self.start_time = None
        self.get_logger().info('Figure-eight mover initialized, waiting for /clock...')

    def _check_clock(self):
        if self.get_clock().now().nanoseconds > 0:
            self.start_time = self.get_clock().now()
            period = 1.0 / float(self.rate)
            self.movement_timer = self.create_timer(period, self.timer_callback)
            self.get_logger().info(f'Clock active — starting figure-eight: scale={self.scale}, frequency={self.frequency}')
            self.clock_checker_timer.cancel()

    def timer_callback(self):
        if self.start_time is None:
            return

        time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        phi = self.frequency * time

        # Derivatives
        dx_dphi = -self.scale * math.sin(phi)
        dy_dphi = self.scale * math.cos(2 * phi)
        d2x_dphi2 = -self.scale * math.cos(phi)
        d2y_dphi2 = -2 * self.scale * math.sin(2 * phi)

        x_dot = dx_dphi * self.frequency
        y_dot = dy_dphi * self.frequency
        x_ddot = d2x_dphi2 * self.frequency**2
        y_ddot = d2y_dphi2 * self.frequency**2

        v = math.sqrt(x_dot**2 + y_dot**2)

        if v > 1e-6:
            omega = (x_dot * y_ddot - y_dot * x_ddot) / (v**2)
        else:
            omega = 0.0

        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(omega)
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FigureEightMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
