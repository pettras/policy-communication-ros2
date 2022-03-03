from setuptools import setup
import os #Added for launch file
from glob import glob #Added for launch file

package_name = 'master'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')) #Added for launch file (we can use ros2 launch master master_lauch.py from dev_ws folder instead of running ros2 launch master_launch.py in the launch folder)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kmriiwa',
    maintainer_email='pettras@ntnu.no',
    description='Robot communication',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'talkerRobot = master.publisherRobot:main',
            'listenerRobot = master.subscriberRobot:main',
        	'talkerGripper = master.publisherGripper:main',
        	'observationNode = master.observationNode:main',
            'actionNode = master.actionNode:main',
            'gripperNode = master.gripperNode:main',
            'policyNode = master.policyNode:main',

        ],
    },
)
