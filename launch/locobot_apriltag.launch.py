# Copyright (c) 2018 Intel Corporation
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

import os
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from geometry_msgs.msg import Quaternion
import tf_transformations

def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


    sin_45 = math.sin(math.radians(45))
    cos_45 = math.cos(math.radians(45))
    sin_90 = math.sin(math.radians(90))
    cos_90 = math.cos(math.radians(90))

    rotation_q = [1.0, 0.0, 0.0, 0.0]
    
    apriltag01 = tf_transformations.quaternion_multiply([0.0, 0.0, 0.0, 1.0], [0.0,  0.0, 0.0, 1.0])
    apriltag02 = tf_transformations.quaternion_multiply([0.0, 0.0, 0.0, 1.0], [0.0,  0.0, sin_45, cos_45])
    apriltag03 = tf_transformations.quaternion_multiply([0.0, 0.0, 0.0, 1.0], [0.0,  0.0, sin_90, cos_90])
    apriltag04 = tf_transformations.quaternion_multiply([0.0, 0.0, 0.0, 1.0], [0.0,  0.0, -sin_45, cos_45])


    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='apriltag01_tf',
            output='screen',
#            parameters=[configured_params],
            remappings=remappings,
            # px py pz qx qy qz qw parent child
            arguments=['2.0', '0.0', '-10.5', str(apriltag01_q[0]), str(apriltag01_q[1]), str(apriltag01_q[2]), str(apriltag01_q[3]), 'pan_link', 'apriltag01_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='apriltag02_tf',
            output='screen',
#            parameters=[configured_params],
            remappings=remappings,
            # px py pz qx qy qz qw parent child
            arguments=['-7.5', '9.5',  '-10.5',  str(apriltag02_q[0]), str(apriltag02_q[1]), str(apriltag02_q[2]), str(apriltag02_q[3]), 'pan_link', 'apriltag02_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='apriltag03_tf',
            output='screen',
#            parameters=[configured_params],
            remappings=remappings,
            # px py pz qx qy qz qw parent child
            arguments=['-17', '0.0',  '-10.5',  str(apriltag03_q[0]), str(apriltag03_q[1]), str(apriltag03_q[2]), str(apriltag03_q[3]), 'pan_link', 'apriltag03_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='apriltag04_tf',
            output='screen',
#            parameters=[configured_params],
            remappings=remappings,
            # px py pz qx qy qz qw parent child
            arguments=['-7.5', '-9.5',  '-10.5', str(apriltag04_q[0]), str(apriltag04_q[1]), str(apriltag04_q[2]), str(apriltag04_q[3]), 'pan_link', 'apriltag04_link']),


    ])
