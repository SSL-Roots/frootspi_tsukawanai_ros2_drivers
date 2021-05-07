# Copyright 2021 Roots
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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='led_switch_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',  # component_container_mtはmulti threads
            composable_node_descriptions=[
                ComposableNode(
                    package='frootspi_led',
                    plugin='frootspi_led::Driver',
                    name='led_driver'),
                ComposableNode(
                    package='frootspi_switch',
                    plugin='frootspi_switch::Driver',
                    name='switch_driver'),
            ],
            output='screen',
    )

    manager = Node(
        name='manager',
        package='frootspi_examples',
        executable='lifecycle_node_manager',
        output='screen',
        parameters=[{'components': ['led_driver', 'switch_driver']}]

    )

    return launch.LaunchDescription([container, manager])
