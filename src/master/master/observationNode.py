import rclpy
from rclpy.node import Node

from master_interfaces.msg import GripperStatus                            # CHANGE
from master_interfaces.msg import RobotStatus                            # CHANGE
import numpy as np

class Subscriber(Node):

    def __init__(self):
        super().__init__('observation_node')
        self.subscription = self.create_subscription(
            GripperStatus,
            'gripper_status',
            self.listener_callback,
            10)
        
        self.subscription2 = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.listener_callback2,
            10)
        
        self.subscription  # prevent unused variable warning
        self.subscription2

        np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
        self.current_status = np.empty(15, np.float)

    def listener_callback(self, msg):
        self.current_status[14] = float(msg.status)
        #print(self.current_status)
        

        #self.get_logger().info('I heard: "%s"' % msg)


    def listener_callback2(self, msg):
        msg_vector = np.array([msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5,
        msg.joint6, msg.joint7, msg.x, msg.y, msg.z, msg.a, msg.b, msg.c, float(msg.running)])
        self.current_status[:14] = msg_vector
        print(self.current_status)
        print("\n")
        halla = 0

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()