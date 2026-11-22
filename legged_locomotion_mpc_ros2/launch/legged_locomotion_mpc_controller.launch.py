# Copyright (c) 2026, Bartłomiej Krajewski
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
Legged Locomotion MPC controller (lifecycle node), configured and activated at start.

Activation needs odometry from the state estimator and joint states, so the estimator
has to be active before (legged_locomotion_control_sim.launch.py does it in the right order).

Inputs:  joint states, odometry (state estimator), contacts, /command_twist, /gait_parameters,
         /swing_parameters, /base_external_wrench, /elevation
Outputs: /mpc_policy for the whole body controller, or joint trajectory for the robot
         with use_mrt_control:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    config_directory = PathJoinSubstitution(
        [FindPackageShare('legged_locomotion_mpc_ros2'), 'config'])

    configure = LaunchConfiguration('configure')
    activate = LaunchConfiguration('activate')

    arguments = [
        DeclareLaunchArgument(
            'config_directory_path', default_value=config_directory,
            description='Directory with legged_model.info and task.info'),
        DeclareLaunchArgument(
            'urdf_path', default_value=PathJoinSubstitution([config_directory, 'meldog.urdf']),
            description='Robot URDF of the MPC model'),
        DeclareLaunchArgument(
            'parameters_file',
            default_value=PathJoinSubstitution(
                [config_directory, 'legged_locomotion_mpc_controller.yaml']),
            description='ROS parameters of the node'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument(
            'use_mrt_control', default_value='false',
            description='true: MRT sends joint trajectory to the robot, '
                        'false: policy is sent to the whole body controller'),
        DeclareLaunchArgument(
            'configure', default_value='true',
            description='Configure the node at start'),
        DeclareLaunchArgument(
            'activate', default_value='true',
            description='Activate the node after configuration'),

        # Topics
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument(
            'odometry_topic', default_value='/estimator_odom',
            description='Base odometry from the state estimator (/odom is the ground truth from Gazebo)'),
        DeclareLaunchArgument(
            'contacts_topic', default_value='/contact_sensors_broadcaster/contact_states'),
        DeclareLaunchArgument(
            'joint_trajectory_topic',
            default_value='/joint_forward_trajectory_controller/joint_forward_trajectory',
            description='Joint trajectory command for the robot (only with use_mrt_control:=true)'),
        DeclareLaunchArgument(
            'policy_topic', default_value='/mpc_policy',
            description='MPC policy for the whole body controller'),
        DeclareLaunchArgument('command_twist_topic', default_value='/command_twist'),
    ]

    node = LifecycleNode(
        package='legged_locomotion_mpc_ros2',
        executable='legged_locomotion_mpc_controller_node',
        name='legged_locomotion_mpc_controller',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('parameters_file'),
            {
                'config_directory_path': ParameterValue(
                    LaunchConfiguration('config_directory_path'), value_type=str),
                'urdf_path': ParameterValue(LaunchConfiguration('urdf_path'), value_type=str),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'use_mrt_control': ParameterValue(
                    LaunchConfiguration('use_mrt_control'), value_type=bool),
            },
        ],
        remappings=[
            ('/joint_states', LaunchConfiguration('joint_states_topic')),
            ('/odom', LaunchConfiguration('odometry_topic')),
            ('/contacts', LaunchConfiguration('contacts_topic')),
            ('/joint_trajectory', LaunchConfiguration('joint_trajectory_topic')),
            ('/mpc_policy', LaunchConfiguration('policy_topic')),
            ('/command_twist', LaunchConfiguration('command_twist_topic')),
        ],
    )

    configure_node = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(configure))

    # Only after configuration, so the node is not activated again after its own deactivation
    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ]),
        condition=IfCondition(AndSubstitution(configure, activate)))

    return LaunchDescription(arguments + [node, activate_after_configure, configure_node])
