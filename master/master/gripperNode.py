#Node that has to subscribe to /req_gripper_pos, sends command to gripper, gets gripper status and publish /cur_gripper_pos
from turtle import position
import rclpy
from rclpy.node import Node

from master_interfaces.msg import GripperStatus                            # CHANGE
from master_interfaces.msg import GripperPos                           # CHANGE

import time
from robotiq_modbus_controller.driver import RobotiqModbusRtuDriver

gripperConnected = True

class PublishingSubscriber(Node):

    def __init__(self):
        super().__init__('gripper_node') #node name
        
        # Set up subscriber to gripper position. Sent from actionNode.
        self.subscription = self.create_subscription(
            GripperPos,
            'gripper_pos',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

        # Set up publisher to gripper status. Sent to observationNode
        self.publisher_ = self.create_publisher(GripperStatus, 'gripper_status', 1)  # CHANGE
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.talker_callback)
        self.i = 0

        if gripperConnected:
            device = "/dev/ttyUSB1" # Name of USB port for gripper
            self.driver = RobotiqModbusRtuDriver(device)# Make a class that comes with the robotiq_modbus_controller python package.
            self.driver.connect() # Connect to gripper
            print("connected")
            self.driver.reset() # Rest the gripper
            print("reset")
            self.driver.activate() # Activate the gripper
            print("activate")
            time.sleep(2) # A pause to make sure that the gripper is fully reset before getting a command

    def listener_callback(self, msg): # Member function that handles the use of gripper_pos information
        if msg.pos==1:
            self.current_pos_request = 255
        else:
            self.current_pos_request = 0
        print(self.current_pos_request)
        #self.get_logger().info('I heard: "%s"' % msg)
        if gripperConnected:
            self.driver.move(pos = self.current_pos_request, speed = 100, force = 10)
            time.sleep(2)
            
    def talker_callback(self): # Member function that makes the gripper_status information
        msg = GripperStatus()
        if gripperConnected:
            current_pos = self.driver.status().position.po
            if current_pos < 50:
                msg.status = 0
            else:
                msg.status = 1
        else:
            msg.status = 2
  
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.status)
        self.i += 1



def main(args=None):
    rclpy.init(args=args)

    publisher_and_subscriber = PublishingSubscriber()

    rclpy.spin(publisher_and_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_and_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
