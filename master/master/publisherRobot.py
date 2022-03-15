import rclpy
from rclpy.node import Node

from master_interfaces.msg import RobotStatus                            # CHANGE

class RobotPublisher(Node):

    def __init__(self):
        super().__init__('robot_publisher') #node name
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)  # CHANGE
        timer_period = 2  # seconds, can be changed to get a higher frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg2 = RobotStatus()
        msg2.joint1 = 1.0
        msg2.joint2 = 1.0
        msg2.joint3 = 1.0
        msg2.joint4 = 1.0
        msg2.joint5 = 1.0
        msg2.joint6 = 1.0
        msg2.joint7 = 1.0
        msg2.x = 1.0
        msg2.y = 1.0
        msg2.z = 1.0
        msg2.a = 1.0
        msg2.b = 1.0
        msg2.c = 1.0
        msg2.running = True
        self.publisher_.publish(msg2)
        self.get_logger().info('Publishing: "%s"' % msg2)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    robot_publisher = RobotPublisher()

    rclpy.spin(robot_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()