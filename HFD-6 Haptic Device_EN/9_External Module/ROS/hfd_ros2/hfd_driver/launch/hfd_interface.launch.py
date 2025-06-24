#!/usr/bin/env python3
# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import launch
from launch import LaunchDescription
import launch_ros
import launch_ros.actions


def generate_launch_description():
    """
    Generate launch description for hfd_interface node.
    """
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace='hfd',
            package='hfd_driver',
            executable='hfd_interface',
            name='hfd_interface',
            output='screen',
            # parameters=[{'param_name': 'param_value'}],  # Replace with actual parameters if needed
        ),
    ])
