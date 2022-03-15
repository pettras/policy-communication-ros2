import rclpy
from rclpy.node import Node

from master_interfaces.msg import GripperStatus                            # CHANGE

class GripperPublisher(Node):

    def __init__(self):
        super().__init__('gripper_publisher') #node name
        self.publisher_ = self.create_publisher(GripperStatus, 'gripper_status', 10)  # CHANGE
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = GripperStatus()
        msg.status = 1
  
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.status)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    gripper_publisher = GripperPublisher()

    rclpy.spin(gripper_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gripper_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
