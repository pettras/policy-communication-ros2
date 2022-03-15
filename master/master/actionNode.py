import rclpy
from rclpy.node import Node

from master_interfaces.msg import GripperPos                            # CHANGE
from master_interfaces.msg import RobotPos   



class Publisher(Node):

    def __init__(self):
        super().__init__('action_node') #node name
        self.publisher_ = self.create_publisher(GripperPos, 'gripper_pos', 10)  # CHANGE
        self.publisher2 = self.create_publisher(RobotPos, 'robot_pos',10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.timer_callback) #sends gripper pos accorting to timer
        self.i = 0

    def timer_callback(self):
        msg = GripperPos()
        msg.pos = 1
        
        msg2 = RobotPos()
        msg2.joint1 = 1.0
        msg2.joint2 = 2.0
        msg2.joint3 = 3.0
        msg2.joint4 = 4.0
        msg2.joint5 = 5.0
        msg2.joint6 = 6.0
        msg2.joint7 = 7.0

        self.publisher_.publish(msg)
        self.publisher2.publish(msg2)
        
        print(msg, "\n", msg2)
        #self.get_logger().info('Publishing: "%s"' % msg.pos)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    action_publisher = Publisher()

    rclpy.spin(action_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    action_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
