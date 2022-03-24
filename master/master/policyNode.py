#Node that has to subscribe to /gripper_status, /robot_status, /CV_image topics and
# publishes to /robot_pos and /gripper_pos after calling tha policy.

from argparse import Action
from turtle import position
import rclpy
from rclpy.node import Node

from master_interfaces.msg import GripperStatus   
from master_interfaces.msg import GripperPos     
from sunrisedds_interfaces.msg import JointPosition

import numpy as np
import time
from stable_baselines3 import PPO
import gym

import robosuite as suite
from robosuite import load_controller_config

from sensor_msgs.msg import Image, PointCloud2 
from zivid_interfaces.srv import Capture2D, Capture 
import matplotlib.pyplot as plt

import cv2

# Function that makes a client node, calls a capture signal til the Zivid server and closes the node.
# Copied from zivid-ros2/zivid_samples/zivid_samples/sample_capture.py
def capture2D():
    node = rclpy.create_node("zivid_camera_client")

    capture_client = node.create_client(Capture2D, "/zivid/capture_2d") 
    while not capture_client.wait_for_service(timeout_sec=1.0):
        print("service not available, waiting again...")

    request = Capture2D.Request() 
    future = capture_client.call_async(request)
    print(future)
    response = Capture2D.Response()
    print(response)
    node.destroy_node()

def capturePoints():
    node = rclpy.create_node("zivid_camera_client")

    capture_client = node.create_client(Capture, "/zivid/capture") 
    while not capture_client.wait_for_service(timeout_sec=1.0):
        print("service not available, waiting again...")

    request = Capture.Request() 
    future = capture_client.call_async(request)
    print(future)
    response = Capture.Response()
    print(response)
    node.destroy_node()

def reshape_image_2d(img):
    new_img = np.reshape(img, (1200,1944,4)) #(1200,1944,4)
    #new_img = cv2.resize(new_img, dsize=(256, 256), interpolation=cv2.INTER_CUBIC)
    new_img = new_img[0:1200, 372:1572]
    new_img = cv2.resize(new_img, dsize=(256, 256), interpolation=cv2.INTER_CUBIC) #USE WITH CAUTION, may interferre with the pixel size
    new_img = np.delete(new_img, 3, axis=2)
    return new_img


class PublishingSubscriber(Node):

    def __init__(self):
        super().__init__('policy_node')                 #node name
        self.subscription = self.create_subscription(
            GripperStatus,                              #interface name from master_interfaces.msg
            'gripper_status',                           #topic name
            self.gripper_listener_callback,             #callback function
            1)                                          #queue size
        
        self.subscription2 = self.create_subscription(
            JointPosition,
            'state',
            self.robot_listener_callback,
            1)

        #Subscription for the Zivid image topic
        self.image2d = True

        if self.image2d:
            self.subscription3 = self.create_subscription(
                Image,
                '/zivid/color/image_color',
                self.zivid_image_callback,
                1)
            self.current_image = np.zeros([1200,1944,4], dtype=np.uint8)

        else:
            self.subscription3 = self.create_subscription(
                PointCloud2,
                '/zivid/points/xyz',
                self.zivid_image_callback,
                1)
            self.current_image = np.zeros([1200,1944,7], dtype=np.uint8)#??
        self.subscription  # prevent unused variable warning
        self.subscription2
        self.subscription3

        # Set up publisher to gripper and robot pos
        self.publisher_ = self.create_publisher(GripperPos, 'gripper_pos', 1)
        self.publisher2 = self.create_publisher(JointPosition, 'command',1)
        
        #Check for movement
        self.timer = self.create_timer(
            1, 
            self.check_movement)

        
        np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
        self.current_status = np.empty(8, np.float)
        self.prev_status = np.empty(8, np.float)
        self.test = 0
        
        #Load trained model
        self.model=PPO.load("/home/kukauser/Downloads/dummy_policy_256_256")
        
        #Set up the Robosuite environment
        self.env = suite.make(
            env_name="Lift", # try with other tasks like "Stack" and "Door"
            robots="IIWA",  # try with other robots like "Sawyer" and "Jaco"
            gripper_types="Robotiq85Gripper",
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            use_object_obs=False,
            controller_configs=load_controller_config(default_controller="OSC_POSE"),
        )
        
    def check_movement(self):


        print("TEST: PREV", self.prev_status)
        print("TEST: CUR", self.current_status)
        if np.around(self.prev_status,3).all() == np.around(self.current_status,3).all(): #check that the robot is not moving
            self.capture_image()

            self.prev_status=self.current_status

            self.set_new_goal()



    def capture_image(self):
        if self.image2d:
            capture2D()
        else:  
            capturePoints()

    def gripper_listener_callback(self, msg): #gripper_callback
        self.current_status[7] = float(msg.status)
        #self.get_logger().info('I heard: "%s"' % msg)


    def robot_listener_callback(self, msg): #full_callback
        msg_vector = np.array([msg.position.a1, msg.position.a2, msg.position.a3, msg.position.a4, msg.position.a5,
        msg.position.a6, msg.position.a7])
        self.current_status[:7] = msg_vector
        #print("current_status",self.current_status)
    
    def zivid_image_callback(self, msg):
        self.current_image = msg.data


        
    def set_new_goal(self): 
        joint_states = self.current_status[:7]        
        image_state = reshape_image_2d(self.current_image)

        #print(image_state)
        k = cv2.imwrite(r'/home/kukauser/Downloads/k.png', cv2.cvtColor(image_state,cv2.COLOR_RGB2BGR))
      
        obs_states = {}
        obs_states['calibrated_camera_image']=image_state
        obs_states['robot0_joint_pos']=joint_states
        
        chosen_action_pose = self.model.predict(obs_states)
        print("chosen action pose",chosen_action_pose)
        obs = self.env.step(chosen_action_pose[0])
        chosen_action_joints = obs[0]['robot0_joint_pos']*0.8
        print("chosen action joints", chosen_action_joints)

        
        msg = GripperPos()
        if self.test == 10:
            self.test = 0
        if self.test > 5:
            msg.pos = 0
        elif self.test <= 5:
            msg.pos = 1
        self.test += 1

        msg2 = JointPosition()
        msg2.position.a1 = chosen_action_joints[0]
        msg2.position.a2 = chosen_action_joints[1]
        msg2.position.a3 = chosen_action_joints[2]
        msg2.position.a4 = chosen_action_joints[3]
        msg2.position.a5 = chosen_action_joints[4]
        msg2.position.a6 = chosen_action_joints[5]
        msg2.position.a7 = chosen_action_joints[6]
        #print("test1", chosen_action_joints[0].dtype)
        


        self.publisher_.publish(msg)
        self.publisher2.publish(msg2)
        print(msg, "\n", msg2)
        #self.get_logger().info('Publishing: "%s"' % msg.pos)

    



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



#funksjon for start position?
#sett på en sleep et sted for å sjekke om man har flere tråder