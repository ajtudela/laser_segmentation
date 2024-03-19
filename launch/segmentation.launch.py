# Copyright (c) 2017 Alberto J. Tudela Roldán
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launches a Segmentation node."""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler

import launch.events
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg


def generate_launch_description():
    # Getting directories and launch-files
    segmentation_dir = get_package_share_directory('laser_segmentation')
    default_params_file = os.path.join(segmentation_dir, 'params', 'default_params.yml')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file with segmentation configuration'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name='log_level',
        default_value='info',
        description='Logging level (info, debug, ...)'
    )

    # Prepare the laser segmentation node.
    segmentation_node = LifecycleNode(
        package='laser_segmentation',
        namespace='',
        executable='laser_segmentation',
        name='segmentation',
        parameters=[params_file],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['segmentation:=', LaunchConfiguration('log_level')]]
    )

    # When the node reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_node_reaches_inactive_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=segmentation_node,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(segmentation_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # Make the node take the 'configure' transition.
    emit_event_to_request_that_node_does_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(segmentation_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription([
        declare_params_file_arg,
        declare_log_level_arg,
        register_event_handler_for_node_reaches_inactive_state,
        emit_event_to_request_that_node_does_configure_transition,
        segmentation_node
    ])
