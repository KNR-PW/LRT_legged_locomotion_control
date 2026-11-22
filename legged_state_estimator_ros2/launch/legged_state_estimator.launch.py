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
Legged State Estimator (lifecycle node), configured and activated at start.

First activation needs initial base odometry (/initial_odometry, transient local), joint states,
IMU and contacts. The base must not move during IMU bias calibration (before activation).
With publish_initial_odometry:=true the initial odometry (pose initial_x/y/z, no rotation,
zero velocity) is published by this launch file.

Inputs:  /initial_odometry, joint states, IMU, contacts
Outputs: odometry (/estimator_odom) for MPC and WBC, /joint_states_estimated (publish_joint_estimates).
         /odom is the ground truth from Gazebo, it is not used
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from lifecycle_msgs.msg import Transition

# Frames of legged_estimator.info (worldFrameName, baseFrameName)
WORLD_FRAME = 'world'
BASE_FRAME = 'trunk_link'


def generate_launch_description():
    config_directory = PathJoinSubstitution(
        [FindPackageShare('legged_state_estimator_ros2'), 'config'])

    configure = LaunchConfiguration('configure')
    activate = LaunchConfiguration('activate')

    arguments = [
        DeclareLaunchArgument(
            'config_directory_path', default_value=config_directory,
            description='Directory with legged_estimator.info'),
        DeclareLaunchArgument(
            'urdf_path', default_value=PathJoinSubstitution([config_directory, 'meldog.urdf']),
            description='Robot URDF of the estimator model'),
        DeclareLaunchArgument(
            'parameters_file',
            default_value=PathJoinSubstitution([config_directory, 'legged_state_estimator.yaml']),
            description='ROS parameters of the node'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument(
            'configure', default_value='true',
            description='Configure the node at start'),
        DeclareLaunchArgument(
            'activate', default_value='true',
            description='Activate the node after configuration'),

        # Initial odometry
        DeclareLaunchArgument(
            'publish_initial_odometry', default_value='false',
            description='Publish initial base odometry from initial_x/y/z (no rotation, zero velocity)'),
        DeclareLaunchArgument('initial_x', default_value='0.0', description='Initial base x [m]'),
        DeclareLaunchArgument('initial_y', default_value='0.0', description='Initial base y [m]'),
        DeclareLaunchArgument('initial_z', default_value='0.0', description='Initial base z [m]'),

        # Topics
        DeclareLaunchArgument('initial_odometry_topic', default_value='/initial_odometry'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('imu_topic', default_value='/imu_sensor_broadcaster/imu'),
        DeclareLaunchArgument(
            'contacts_topic', default_value='/contact_sensors_broadcaster/contact_states'),
        DeclareLaunchArgument(
            'odometry_topic', default_value='/estimator_odom',
            description='Base odometry estimate for MPC and WBC (/odom is the ground truth from Gazebo)'),
    ]

    node = LifecycleNode(
        package='legged_state_estimator_ros2',
        executable='legged_state_estimator_node',
        name='legged_state_estimator',
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
            },
        ],
        remappings=[
            ('/initial_odometry', LaunchConfiguration('initial_odometry_topic')),
            ('/joint_states', LaunchConfiguration('joint_states_topic')),
            ('/imu', LaunchConfiguration('imu_topic')),
            ('/contacts', LaunchConfiguration('contacts_topic')),
            ('/odom', LaunchConfiguration('odometry_topic')),
        ],
    )

    # Transient local, so estimator gets it also when started later. It is read only during
    # activation, so publishing it all the time does not change the estimate
    initial_odometry = [
        '{header: {frame_id: ', WORLD_FRAME, '}, child_frame_id: ', BASE_FRAME, ', '
        'pose: {pose: {position: {x: ', LaunchConfiguration('initial_x'),
        ', y: ', LaunchConfiguration('initial_y'),
        ', z: ', LaunchConfiguration('initial_z'), '}, '
        'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
    ]

    initial_odometry_publisher = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--rate', '1',
             '--qos-reliability', 'reliable', '--qos-durability', 'transient_local',
             '--qos-depth', '1',
             LaunchConfiguration('initial_odometry_topic'), 'nav_msgs/msg/Odometry',
             initial_odometry],
        output='log',
        condition=IfCondition(LaunchConfiguration('publish_initial_odometry')))

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

    return LaunchDescription(arguments + [
        initial_odometry_publisher, node, activate_after_configure, configure_node])
