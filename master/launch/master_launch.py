from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='master',
            executable='gripperNode',
        ),
        Node(
            package='master',
            executable='talkerRobot',
        ),
        #Node(
        #    package='master',
        #    executable='actionNode',
        #),

        #Node(
        #    package='master',
        #    executable='observationNode',
        #),

        Node(
            package='master',
            executable='policyNode',
        ),

        Node(
            package='master',
            #namespace='masterL',
            executable='listenerRobot',
            #name='robotListenerL'
        )
    ])