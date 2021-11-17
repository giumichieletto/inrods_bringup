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

    devlab04_q = tf_transformations.quaternion_multiply([0.0, 1.0, 0.0, 0.0], [0.0,  0.0, sin_45, cos_45])
    devlab05_q = tf_transformations.quaternion_multiply([1.0, 0.0, 0.0, 0.0], [0.0,  0.0, sin_90, cos_90])
    devlab07_q = tf_transformations.quaternion_multiply([1.0, 0.0, 0.0, 0.0], [0.0,  0.0, 0.0, 1.0])
    devlab08_q = tf_transformations.quaternion_multiply([0.0, 1.0, 0.0, 0.0], [0.0,  0.0, -sin_45, cos_45])
    devlab10_q = tf_transformations.quaternion_multiply([1.0, 0.0, 0.0, 0.0], [0.0,  0.0, sin_90, cos_90])

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='devlab04_tf',
            output='screen',
#            parameters=[configured_params],
            remappings=remappings,
            #             px      py      pz    qx  qy  qz  qw  parent            child
            arguments=[' 1.5', ' 0.0', ' 0.6', str(devlab04_q[0]), str(devlab04_q[1]), str(devlab04_q[2]), str(devlab04_q[3]), 'map', 'devlab04_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='devlab05_tf',
            output='screen',
#            parameters=[configured_params],
            remappings=remappings,
            #          px  py  pz  qx  qy  qz  qw  parent            child
            arguments=['3.75', '1.0',  '0.6',  str(devlab05_q[0]), str(devlab05_q[1]), str(devlab05_q[2]), str(devlab05_q[3]), 'map', 'devlab05_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='devlab07_tf',
            output='screen',
#            parameters=[configured_params],
            remappings=remappings,
            #          px  py  pz  qx  qy  qz  qw  parent            child
            arguments=['1.25', '2.0',  '0.6',  str(devlab07_q[0]), str(devlab07_q[1]), str(devlab07_q[2]), str(devlab07_q[3]), 'map', 'devlab07_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='devlab08_tf',
            output='screen',
#            parameters=[configured_params],
            remappings=remappings,
            #          px  py  pz  qx  qy  qz  qw  parent            child
            arguments=['2.25', '3.75',  '0.6', str(devlab08_q[0]), str(devlab08_q[1]), str(devlab08_q[2]), str(devlab08_q[3]), 'map', 'devlab08_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='devlab10_tf',
            output='screen',
#            parameters=[configured_params],
            remappings=remappings,
            #          px  py  pz  qx  qy  qz  qw  parent            child
            arguments=['3.75', '3.0',  '0.6', str(devlab10_q[0]), str(devlab10_q[1]), str(devlab10_q[2]), str(devlab10_q[3]), 'map', 'devlab10_link']),

    ])
