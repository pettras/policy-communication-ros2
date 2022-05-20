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
from zivid_interfaces.srv import Capture, Capture2D
import matplotlib.pyplot as plt

import sys
sys.path.insert(0, '/home/kukauser/Robot_Learning_master/code')
from src.models.robots.manipulators.iiwa_14_robot import IIWA_14, IIWA_14_modified_flange, IIWA_14_modified_flange_joint_limit
from src.models.grippers.robotiq_85_iiwa_14_gripper import Robotiq85Gripper_iiwa_14, Robotiq85Gripper_iiwa_14_longer_finger
from src.helper_functions.register_new_models import register_gripper, register_robot_class_mapping
from src.environments import Lift_4_objects, Lift_edit

from robosuite.environments.base import register_env
from robosuite.models.robots.robot_model import register_robot
from robosuite.wrappers import DomainRandomizationWrapper

import cv2
from PIL import Image as PILImage
from math import isclose
np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

import yaml
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecTransposeImage
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from src.helper_functions.wrap_env import make_multiprocess_env
from src.wrapper import GymWrapper_multiinput
from stable_baselines3.common.monitor import Monitor

from src.helper_functions.camera_functions import adjust_width_of_image



image_observation = True
eef_observation = True
joint_observation = False
gripper_observation_sim = True
gripper_observation_phy = False
#images_size = (486, 300)
save_image = True
#crop_image = False


#policy_model_path = "/home/kukauser/petter/zip_files/36_cpu_best_model" #/home/kukauser/Downloads/dummy_policy_256_256") #/home/kukauser/petter/zip_files/best_model
#policy_model_path = "/home/kukauser/petter/zip_files/logs_modified_gripper_length_new/best_success_rate"
#load_vecnormalize_path = "/home/kukauser/petter/zip_files/logs_modified_gripper_length_new/vec_normalize_best_success_rate.pkl"

policy_model_path = "/home/kukauser/petter/zip_files/manulab_testing/custom_camera_domain_rand/best_model"
load_vecnormalize_path = "/home/kukauser/petter/zip_files/manulab_testing/custom_camera_domain_rand/vec_normalize_best_model.pkl"

#policy_model_path = "/home/kukauser/petter/zip_files/logs_modified_doamin_rand_3times_more/best_model"
#load_vecnormalize_path = "/home/kukauser/petter/zip_files/logs_modified_doamin_rand_3times_more/vec_normalize_best_model.pkl"

class PublishingSubscriber(Node):

    def __init__(self):
        super().__init__('policy_node')                 #node name
        
        #Subscribe to gripper
        self.subscription_gripper = self.create_subscription(
            GripperStatus,                              #interface name from master_interfaces.msg
            'gripper_status',                           #topic name
            self.gripper_current_status,             #callback function
            1,                                          #queue size
            )                                          
        
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
            0.5, 
            self.check_movement)

        #Create arrays for collection of physical and simulated robot and gripper data
        self.gripper_movement = 0
        self.simulated_eef_pos = np.zeros(3)
        self.simulated_joint_state = np.zeros(7)
        self.physical_joint_state = np.zeros(7)
        self.physical_gripper_state = 0
        self.simulated_gripper_state = 0
        self.first_runthrough=1
        self.counter=0

        #Load trained model
        self.policy_model=PPO.load(policy_model_path)

        #Set up the Robosuite environment
        register_robot(IIWA_14)
        register_robot(IIWA_14_modified_flange)
        register_robot(IIWA_14_modified_flange_joint_limit)
        register_gripper(Robotiq85Gripper_iiwa_14)
        register_gripper(Robotiq85Gripper_iiwa_14_longer_finger)
        register_robot_class_mapping("IIWA_14")
        register_robot_class_mapping("IIWA_14_modified_flange")
        register_robot_class_mapping("IIWA_14_modified_flange_joint_limit")
        register_env(Lift_edit)
        register_env(Lift_4_objects)

        yaml_file = "config_files/" +  "ppo_modified_gripper_length_domain_rand.yaml"
        print("you are using this yaml file: ", yaml_file)
        with open(yaml_file, 'r') as stream:
            config = yaml.safe_load(stream)

        domain_yaml_file = "config_files/domain_rand_args.yaml"
        with open(domain_yaml_file, 'r') as stream:
            domain_config = yaml.safe_load(stream)

        # Environment specifications
        env_options = config["robosuite"]
        env_options["custom_camera_trans_matrix"] = np.array(env_options["custom_camera_trans_matrix"])
        env_id = env_options.pop("env_id")

        # Observations
        obs_config = config["gymwrapper"]
        obs_list = obs_config["observations"] 
        obs_list.append("robot0_joint_pos")
        smaller_action_space = obs_config["smaller_action_space"]
        xyz_action_space = obs_config["xyz_action_space"]

        normalize_obs = config['normalize_obs']
        normalize_rew = config['normalize_rew']
        norm_obs_keys = config['norm_obs_keys']

        #use domain randomization
        use_domain_rand = config["use_domain_rand"]
        domain_rand_args = domain_config["domain_rand_args"]

        num_procs = 1
        seed = 0

        self.robosuite_env = GymWrapper_multiinput(suite.make(env_id, **env_options), obs_list, smaller_action_space, xyz_action_space) 
        #make_multiprocess_env(env_id, env_options, obs_list, smaller_action_space, xyz_action_space,  1, seed)
        if use_domain_rand:
            self.robosuite_env = DomainRandomizationWrapper(self.robosuite_env, **domain_rand_args)
        self.robosuite_env = Monitor(self.robosuite_env, info_keywords = ("is_success",))
        self.robosuite_env = DummyVecEnv([lambda : self.robosuite_env])
        self.robosuite_env = VecTransposeImage(self.robosuite_env)

        if normalize_obs or normalize_rew:
            self.robosuite_env = VecNormalize.load(load_vecnormalize_path, self.robosuite_env)
            #self.robosuite_env = VecNormalize(self.robosuite_env, norm_obs=normalize_obs,norm_reward=normalize_rew,norm_obs_keys=norm_obs_keys)

        self.robosuite_env.training = False
        self.robosuite_env.reset()
        print("Environment built")
        gripper_init_msg= GripperPos()
        gripper_init_msg.pos = -1.0
        for i in range(4):
            self.publisher_gripper.publish(gripper_init_msg)
            time.sleep(0.2)

    def zivid_image_callback(self, msg):
        self.current_image = msg.data
        print("Recieved image")

        self.set_new_goal() #When the image is revieved we set a new goal for the robot and gripper


    def check_movement(self):

        #print("TEST: phy", self.simulated_joint_state)
        #print("TEST: sim", self.physical_joint_state)


        if (np.allclose(self.simulated_joint_state, self.physical_joint_state, atol=0.0001) 
        and self.gripper_movement==0 or self.first_runthrough==1):#check that the robot is not moving
            self.capture_image()
            #time.sleep(0.5) #To recieve the image before running the set_new_goal function
            


    def capture_image(self):
        capture2D()

    def gripper_current_status(self, msg): #gripper_callback
        self.physical_gripper_state = float(msg.closed)
        self.gripper_movement = msg.movement
        #self.get_logger().info('I heard-----: "%s"' % msg)


    def robot_current_status(self, msg): #full_callback
        msg_vector = np.array([msg.position.a1, msg.position.a2, msg.position.a3, msg.position.a4, msg.position.a5,
        msg.position.a6, msg.position.a7])
        self.physical_joint_state = msg_vector
        
    


        
    def set_new_goal(self): 
        image_state = reshape_image_2d(self.current_image)

        
        #OBSERVATION SPACE
        obs_states = {}
        if image_observation:
            obs_states['custom_image']=image_state                      #image name in dummy PPO: 'calibrated_camera_image'
        if joint_observation:                                           #as in dummy_PPO. 
            obs_states['robot0_joint_pos']=self.physical_joint_state
        if eef_observation:
            obs_states['robot0_eef_pos']=self.simulated_eef_pos 
        if gripper_observation_sim:
            obs_states['gripper_status']=self.simulated_gripper_state 
        if gripper_observation_phy:
            obs_states['gripper_status']=self.physical_gripper_state 

        #Test i joint states from robosuite and physical robot is the same (should be)
        #print("physical_joint_state:", self.physical_joint_state)
        #print("simulated_joint_state", self.simulated_joint_state, "\n")
        #print("simulated_eef_pos", self.simulated_eef_pos, "\n")
       

        #The policy choose the next action (x,y,z,c,g) based on the observations
        chosen_action_pose = self.policy_model.predict(obs_states)
        #print("chosen action pose",chosen_action_pose[0]) 
        
        #Test for different actions:
        """chosen_action_pose = np.array([[0.0, 0.0, 0.0, 0.2, 0.0]])
        print("chosen action pose",chosen_action_pose[0])""" 

        if len(chosen_action_pose[0]) == 1: #check if chosen_action_pose[0] is on the right form
            chosen_action_pose = chosen_action_pose[0]
        
        #Robosuite translates the action (x,y,z,a,b,c) to joint angles (a1,a2,a3,a4,a5,a6,a7)
        obs = self.robosuite_env.step(chosen_action_pose)

        #print("OBSSSS", obs)
        self.simulated_joint_state = obs[0]['robot0_joint_pos'][0]
        #print("chosen action joints", self.simulated_joint_state[0])

        
        test_limits(self.simulated_joint_state)

        #Test if the eef_pos can be taken from the robosuite environment
        self.simulated_eef_pos = obs[0]['robot0_eef_pos'] #(x,y,z)
        self.simulated_gripper_state =  obs[0]['gripper_status']
        #print("Gripper", self.simulated_gripper_state)
        
        #print("TYPE", type(self.simulated_gripper_state[0]))
        #print("Message to gripper ------", float(chosen_action_pose[0][-1]))
        gripper_msg= GripperPos()
        gripper_msg.pos = float(chosen_action_pose[0][-1])


        robot_msg = JointPosition()
        robot_msg.position.a1 = float(self.simulated_joint_state[0]) #joint action taken from robosuite
        robot_msg.position.a2 = float(self.simulated_joint_state[1])
        robot_msg.position.a3 = float(self.simulated_joint_state[2])
        robot_msg.position.a4 = float(self.simulated_joint_state[3])
        robot_msg.position.a5 = float(self.simulated_joint_state[4])
        robot_msg.position.a6 = float(self.simulated_joint_state[5])
        robot_msg.position.a7 = float(self.simulated_joint_state[6])
        #print("test1", self.simulated_joint_state[0].dtype)



        
        self.publisher_gripper.publish(gripper_msg)
        self.publisher_robot.publish(robot_msg)
        #print(gripper_msg, "\n", robot_msg)
        #self.get_logger().info('Publishing: "%s"' % msg.pos)

        self.first_runthrough=0
        self.counter+=1
        print("COUNTER:", self.counter)
        assert self.counter < 200

    
# Function that makes a client node, calls a capture signal til the Zivid server and closes the node.
# Copied from zivid-ros2/zivid_samples/zivid_samples/sample_capture.py
def capture2D():
    node = rclpy.create_node("zivid_camera_client")

    capture_client = node.create_client(Capture2D, "/zivid/capture_2d")
    while not capture_client.wait_for_service(timeout_sec=1.0):
        print("service not available, waiting again...")

    request = Capture2D.Request()
    future = capture_client.call_async(request)
    response = Capture2D.Response()    
    node.destroy_node()    




def reshape_image_2d(img):
    new_img = np.reshape(img, (1200,1944,4)) #(1200,1944,4)
    new_img = np.delete(new_img, 3, axis=2) #delete 4th dimension

    #print(new_img.shape)
    new_img = new_img[0:1200, 372:1572] #h,w
    new_img = cv2.resize(new_img, dsize=(84, 84), interpolation=cv2.INTER_CUBIC)
    new_img = cv2.flip(new_img, 0)
    #new_img = cv2.normalize(imageread, resultimage, 0, 100, cv.NORM_MINMAX)


    #print("Before divide ------", new_img)
    #new_img = new_img / 255.0
    #img_float32 = np.float32(new_img)
    #lab_image = cv2.cvtColor(img_float32, cv2.COLOR_RGB2HSV)
    #print("After divide ---------", new_img)

    if save_image:
        print_img = cv2.flip(new_img, 0)
        print_img2 = np.reshape(img, (1200,1944,4)) #(1200,1944,4)
        print_img2 = np.delete(print_img2, 3, axis=2) #delete 4th dimension

        k = cv2.imwrite(r'/home/kukauser/Downloads/test_obs.png', cv2.cvtColor(print_img, cv2.COLOR_RGB2BGR))
        l = cv2.imwrite(r'/home/kukauser/Downloads/test_env.png', cv2.cvtColor(print_img2, cv2.COLOR_RGB2BGR))
    
    new_img = np.transpose(new_img, (2, 0, 1))
    #print("SHAPE", new_img.shape)

    #test
    img = np.transpose(new_img, (2, 0, 1))
    img = PILImage.fromarray(img, 'RGB')
    img.save('/home/kukauser/Downloads/k2.png')

    return new_img

def test_limits(joint_angles):
    assert -2.9 < joint_angles[0] < 2.9, "Joint 1 out of bounds"
    assert -1.91 < joint_angles[1] < 1.91, "Joint 2 out of bounds"
    assert -2.9 < joint_angles[2] < 2.9, "Joint 3 out of bounds"
    assert -1.9 < joint_angles[3] < 1.9, "Joint 4 out of bounds"
    assert -2.9 < joint_angles[4] < 2.9, "Joint 5 out of bounds"
    assert -1.9 < joint_angles[5] < 1.9, "Joint 6 out of bounds"
    assert -1.9 < joint_angles[6] < 1.9, "Joint 7 out of bounds"  #should be -2.9 < joint_angles[6] < 2.0, but the robot is out of bounds at 110 degrees.


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

#-0.017 0.871 0.020 -1.640 0.043 0.432 -0.054 
#-0.97 49.90 -1.15 -93.97 2.46 24.75 -3.09
#