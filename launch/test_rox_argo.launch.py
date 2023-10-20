#Neobotix2023
#Author: Pradheep Padmanabhan

import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    kin_params = LaunchConfiguration(
        'kin_params',
        default = os.path.join(
            get_package_share_directory('rox_argo_kinematics'),
            'launch',
            'rox_argo_kinematics.yaml'))

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rox_argo_kinematics', executable='rox_argo_kinematics_node', output='screen',
            name='argo_kinematics_node', parameters = [kin_params])
    ])
