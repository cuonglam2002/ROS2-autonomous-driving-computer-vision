from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'self_driving'
config_module = 'self_driving/config'
det_module = 'self_driving/Detection'
detect_l_module = 'self_driving/Detection/Lanes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, config_module, det_module, detect_l_module],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='an',
    maintainer_email='an@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder_node = self_driving.video_recorder:main',
            'driving_node = self_driving.driving_node:main',
            'vel_wheel_node = self_driving.wheel_left_right:main',
            'computer_vision_node = self_driving.computer_vision_node:main',
            'cam_node = self_driving.ros2_camera:main',
            'start_node = self_driving.start_robot:main',
            'traffic_signal_node = self_driving.traffic_signal:main',  
            'yolo_node = self_driving.yolo_node:main',
            'hand_control_node = self_driving.hand_control_node:main',
             'pose_node = self_driving.pose_node:main',
        ],
    },
)
