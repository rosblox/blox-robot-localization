# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():


    gps_transform_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_transform',
        output='screen',
        arguments = "--x 1 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id gps".split(' '),
        )


    imu_transform_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_transform',
        output='screen',
        arguments = "--x 0 --y 1 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id imu".split(' '),
        )
    

    navsat_transform_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        respawn=True,
        parameters=[{
            "magnetic_declination_radians": 0.0,
            "yaw_offset": 0.0,
            "zero_altitude": True,
            "use_odometry_yaw": False,
            "wait_for_datum": False,
            "publish_filtered_gps": True,
            "broadcast_utm_transform": False,
        }])
    


    # <param name="magnetic_declination_radians" value="0"/>
    # <param name="yaw_offset" value="0"/>
    # <param name="zero_altitude" value="true"/>

    # <param name="use_odometry_yaw" value="false"/>
    # <param name="wait_for_datum" value="false"/>
   
    # <param name="publish_filtered_gps" value="true"/>
    # <param name="broadcast_utm_transform" value="false"/>
    
    # <remap from="/imu/data" to="/imu" />
    # <remap from="/gps/fix" to="/fix" />
    # <remap from="/odometry/filtered" to="/odometry/filtered" />



    ukf_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_node',
        output='screen',
        respawn=True,
        parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ukf.yaml')],
        )


    return LaunchDescription([
        gps_transform_node,
        imu_transform_node,
        navsat_transform_node,
        ukf_localization_node,
        ])
