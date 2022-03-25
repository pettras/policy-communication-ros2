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

import sys
sys.path.insert(0, '/home/kukauser/Robot_Learning_master/code')
from src.models.robots.manipulators.iiwa_14_robot import IIWA_14
from src.models.grippers.robotiq_85_iiwa_14_gripper import Robotiq85Gripper_iiwa_14
from src.helper_functions.register_new_models import register_gripper, register_robot_class_mapping
from src.environments import Lift_4_objects, Lift_edit

from robosuite.environments.base import register_env
from robosuite.models.robots.robot_model import register_robot

import cv2
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})


image_observation = True
eef_observation = True
joint_observation = False
gripper_observation = True

#images_size = (486, 300)
save_image = True
#crop_image = False


max_robot_movement = 0.8
policy_model_path = "/home/kukauser/petter/zip_files/best_model" #/home/kukauser/Downloads/dummy_policy_256_256") #/home/kukauser/petter/zip_files/best_model


#gripper_connected = True
#robot_connected = True
#camera_connected = True



class PublishingSubscriber(Node):

    def __init__(self):
        super().__init__('policy_node')                 #node name
        
        #Subscribe to gripper
        self.subscription_gripper = self.create_subscription(
            GripperStatus,                              #interface name from master_interfaces.msg
            'gripper_status',                           #topic name
            self.gripper_current_status,             #callback function
            1)                                          #queue size
        
        #Subscribe to robot
        self.subscription_robot = self.create_subscription(
            JointPosition,
            'state',
            self.robot_current_status,
            1)

        #Subscribe to Zivid camera 2d image topic
        self.subscription_camera = self.create_subscription(
            Image,
            '/zivid/color/image_color',
            self.zivid_image_callback,
            1)
        self.current_image = np.zeros([1200,1944,4], dtype=np.uint8)

        #Prevent unused variable warning
        self.subscription_gripper  
        self.subscription_robot
        self.subscription_camera

        # Set up publisher to gripper and robot pos
        self.publisher_gripper = self.create_publisher(GripperPos, 'gripper_pos', 1)
        self.publisher_robot = self.create_publisher(JointPosition, 'command',1)
        
        #Check for movement
        self.timer = self.create_timer(
            1, 
            self.check_movement)

        #Create arrays for collection current and previous status of robot and gripper
        self.current_status = np.empty(8, np.float)
        self.prev_status = np.empty(8, np.float)
        
        #Load trained model
        self.policy_model=PPO.load(policy_model_path)

        #Set up the Robosuite environment
        register_robot(IIWA_14)
        register_gripper(Robotiq85Gripper_iiwa_14)
        register_robot_class_mapping("IIWA_14")
        register_env(Lift_edit)
        register_env(Lift_4_objects)

        self.robosuite_env = suite.make(
            env_name="Lift_edit", # try with other tasks like "Stack" and "Door"
            robots="IIWA_14",  # try with other robots like "Sawyer" and "Jaco"
            gripper_types="Robotiq85Gripper_iiwa_14",
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            use_object_obs=False,
            controller_configs=load_controller_config(default_controller="OSC_POSE"),
        )

        #TEST SECTION
        self.simulated_eef_state = None
        
    def check_movement(self):
        #print("TEST: PREV", self.prev_status)
        #print("TEST: CUR", self.current_status)
        if np.around(self.prev_status,3).all() == np.around(self.current_status,3).all(): #check that the robot is not moving
            self.capture_image()
            self.prev_status=self.current_status
            self.set_new_goal()


    def capture_image(self):
        capture2D()

    def gripper_current_status(self, msg): #gripper_callback
        self.current_status[7] = float(msg.status)
        #self.get_logger().info('I heard: "%s"' % msg)


    def robot_current_status(self, msg): #full_callback
        msg_vector = np.array([msg.position.a1, msg.position.a2, msg.position.a3, msg.position.a4, msg.position.a5,
        msg.position.a6, msg.position.a7])
        self.current_status[:7] = msg_vector
        #print("current_status",self.current_status)
    
    def zivid_image_callback(self, msg):
        self.current_image = msg.data


        
    def set_new_goal(self): 
        #Get the current state from robot and image
        joint_states = self.current_status[:7]
        #gripper_state = self.current_status[7]        
        image_state = reshape_image_2d(self.current_image)

        if save_image:
            k = cv2.imwrite(r'/home/kukauser/Downloads/k.png', cv2.cvtColor(image_state,cv2.COLOR_RGB2BGR))
        
        #OBSERVATION SPACE
        obs_states = {}
        if image_observation:
            obs_states['custom_image']=image_state #image name in dummy PPO: 'calibrated_camera_image'
        if eef_observation:
            obs_states['robot0_eef_pos']=self.simulated_eef_state 
        if joint_observation: #as in dummy_PPO
            obs_states['robot0_joint_pos']=joint_states
        if gripper_observation:
            # ... = current_status[7]
        
        #Test i joint states from robosuite and physical robot is the same (should be)
        physical_joint_state = joint_states #(a1,a2,a3,a4,a5,a6,a7)
        simulated_joint_state = obs[0]['robot0_joint_pos'] #(a1,a2,a3,a4,a5,a6,a7)
        print("physical_joint_state:", physical_joint_state, "\n")
        print("simulated_joint_state", simulated_joint_state, "\n")

        
        #The policy choose the next action (x,y,z,a,b,c) based on the observations
        chosen_action_pose = self.policy_model.predict(obs_states)
        print("chosen action pose",chosen_action_pose)
        #Robosuite translates the action (x,y,z,a,b,c) to joint angles (a1,a2,a3,a4,a5,a6,a7)
        obs = self.robosuite_env.step(chosen_action_pose[0])
        chosen_action_joints = obs[0]['robot0_joint_pos']*max_robot_movement #Will be innaccurate if max_robot_movement != 1
        print("chosen action joints", chosen_action_joints)

        #Test if the eef_pos can be taken from the robosuite environment
        self.simulated_eef_state = obs[0]['robot0_eef_pos'] #(x,y,z)

        
        gripper_msg = GripperPos()
        gripper_msg.pos=float(chosen_action_pose[0][-1]) #gripper action directly from policy

        robot_msg = JointPosition()
        robot_msg.position.a1 = chosen_action_joints[0] #joint action taken from robosuite
        robot_msg.position.a2 = chosen_action_joints[1]
        robot_msg.position.a3 = chosen_action_joints[2]
        robot_msg.position.a4 = chosen_action_joints[3]
        robot_msg.position.a5 = chosen_action_joints[4]
        robot_msg.position.a6 = chosen_action_joints[5]
        robot_msg.position.a7 = chosen_action_joints[6]
        #print("test1", chosen_action_joints[0].dtype)
        

        self.publisher_gripper.publish(gripper_msg)
        self.publisher_robot.publish(robot_msg)
        print(gripper_msg, "\n", robot_msg)
        #self.get_logger().info('Publishing: "%s"' % msg.pos)

    
# Function that makes a client node, calls a capture signal til the Zivid server and closes the node.
# Copied from zivid-ros2/zivid_samples/zivid_samples/sample_capture.py
def capture2D():
    node = rclpy.create_node("zivid_camera_client")

    capture_client = node.create_client(Capture2D, "/zivid/capture_2d") 
    while not capture_client.wait_for_service(timeout_sec=1.0):
        print("service not available, waiting again...")

    request = Capture2D.Request() 
    future = capture_client.call_async(request)
    #print(future)
    response = Capture2D.Response()
    #print(response)
    node.destroy_node()



def reshape_image_2d(img):
    new_img = np.reshape(img, (1200,1944,4)) #(1200,1944,4)
    new_img = np.delete(new_img, 3, axis=2) #delete 4th dimension
    #new_img = cv2.resize(new_img, dsize=(256, 256), interpolation=cv2.INTER_CUBIC)
    #new_img = new_img[0:1200, 372:1572]
    new_img = cv2.resize(new_img, dsize=(486, 300), interpolation=cv2.INTER_CUBIC) #USE WITH CAUTION, may interferre with the pixel size
    new_img = cv2.rotate(new_img, cv2.ROTATE_180)

    return new_img


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