#To activate: 
#cd dev_ws
#sourcerosinstall
#cd
#cd Robot_learning_master/code
#ros2 topic pub --once /calibration_trigger master_interfaces/msg/CalibrationTrigger "trigger_number: 1"


#Node that has to subscribe to /gripper_status, /robot_status, /CV_image topics and
# publishes to /robot_pos and /gripper_pos after calling tha policy.

from argparse import Action
from turtle import position
import rclpy
from rclpy.node import Node

from master_interfaces.msg import CalibrationTrigger   
from sunrisedds_interfaces.msg import JointPosition

import numpy as np
import time

import robosuite as suite
from robosuite import load_controller_config


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
from math import isclose
import math
from scipy.spatial.transform import Rotation as R
import zivid
import datetime

np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})


image_observation = True
eef_observation = False
joint_observation = False

#images_size = (486, 300)
save_image = True
#crop_image = False


test_robot_movement_mode = True


class PublishingSubscriber(Node):

    def __init__(self):
        super().__init__('policy_node')                 #node name
        
        #Subscribe to gripper
        self.subscription_gripper = self.create_subscription(
            CalibrationTrigger,                              #interface name from master_interfaces.msg
            'calibration_trigger',                           #topic name
            self.calibration_trigger_callback,             #callback function
            1)                                          #queue size

        #Subscribe to robot
        self.subscription_robot = self.create_subscription(
            JointPosition,
            'state',
            self.robot_current_status,
            1)

        #Prevent unused variable warning
        self.subscription_gripper  
        self.subscription_robot

        self.first_runtrhough = 1

        
        #Create arrays for collection of physical and simulated robot 
        self.simulated_eef_pos = None
        self.simulated_eef_quat = None
        self.simulated_joint_state = np.zeros(7)
        self.physical_joint_state = np.zeros(7)

        #Set up the Robosuite environment
        register_robot(IIWA_14)
        register_gripper(Robotiq85Gripper_iiwa_14)
        register_robot_class_mapping("IIWA_14")
        register_env(Lift_edit)
        register_env(Lift_4_objects)

        self.robosuite_env = suite.make(
            env_name="Lift_edit", # try with other tasks like "Stack" and "Door"
            robots="IIWA_14",  # try with other robots like "Sawyer" and "Jaco"
            gripper_types=None,
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            use_object_obs=False,
            controller_configs=load_controller_config(default_controller="JOINT_POSITION"),
            control_freq=5

        )

        #connect directly to camera (not with ROS2)
        self.app = zivid.Application()
        print("Connecting to camera")
        self.camera = self.app.connect_camera()
        print("Connected")


    def robot_current_status(self, msg): #full_callback
        msg_vector = np.array([msg.position.a1, msg.position.a2, msg.position.a3, msg.position.a4, msg.position.a5,
        msg.position.a6, msg.position.a7])
        self.physical_joint_state = msg_vector
        print(self.physical_joint_state)

    def calibration_trigger_callback(self,msg):
        if msg.trigger_number == 1:
            image = _acquire_checkerboard_frame(self.camera)
            time.sleep(1)
            #k = cv2.imwrite(r'/home/kukauser/dev_ws/zividTest.png', cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
        
            if self.first_runtrhough:
                self.move_simulated_robot()

            counter=0
            while not (np.allclose(self.simulated_joint_state, self.physical_joint_state, atol=0.004)):
                self.move_simulated_robot()
                counter +=1
                print("Step: ", counter)
                if counter == 150:
                    print("\n \n Problems moving the simulated robot to the right place \n \n")
                    rclpy.shutdown()
            trans_matrix_str = get_matrix_as_string(self.simulated_eef_pos, self.simulated_eef_quat)
            write_to_file(trans_matrix_str)

    def move_simulated_robot(self):
        print("physical_joint_state:", self.physical_joint_state, "\n")
        print("simulated_joint_state", self.simulated_joint_state, "\n --------------------- \n")
        #print("simulated_eef_state", self.simulated_eef_pos, "\n")


        goal_state = (self.physical_joint_state-self.simulated_joint_state)*10
        goal_state[2]*4
        goal_state[4]*4
        goal_state[5]*50
        goal_state[6]*50
        #Move simulated robot in Robosuite 
        obs = self.robosuite_env.step(goal_state)
        
        #Save observations
        self.simulated_joint_state = obs[0]['robot0_joint_pos']
        self.simulated_eef_pos = obs[0]['robot0_eef_pos']
        self.simulated_eef_quat = obs[0]['robot0_eef_quat']

        self.first_runtrhough = 0

def get_matrix_as_string(pos, quat): 
    #alt1
    #angle = euler_from_quaternion(quat)
    #rot = R.from_euler("zyx", angle, degrees=True)
    #alt2
    rot = R.from_quat(quat)

    rotmat = rot.as_matrix()

    T = np.zeros((4,4))
    T[:3, :3] = rotmat
    T[:3, 3] = pos
    T[3, :] = [0, 0, 0, 1]
    print(T)

    outstr = ""
    for r in T:
        for rc in r:
            outstr += " " + str(rc)


    return outstr + "\n"


def write_to_file(T_str):
    text_file = open("/home/kukauser/Documents/rot_mat.txt", "i")
    #write string to file
    text_file.write(T_str)
    #close file
    text_file.close()


def _acquire_checkerboard_frame(camera):
    """Acquire checkerboard frame.

    Args:
        camera: Zivid camera

    Returns:
        frame: Zivid frame

    """
    print("Configuring settings")
    settings = zivid.Settings()
    settings.acquisitions.append(zivid.Settings.Acquisition())
    settings.acquisitions[0].aperture = 8.0
    settings.acquisitions[0].exposure_time = datetime.timedelta(microseconds=20000)
    settings.processing.filters.smoothing.gaussian.enabled = True
    print("Capturing checkerboard image")
    return camera.capture(settings)

def euler_from_quaternion(x, y, z, w): #https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

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