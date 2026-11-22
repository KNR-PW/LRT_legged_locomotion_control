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
Simulation (Gazebo) only: all nodes use simulation clock (use_sim_time is always true).
Robot simulation has to be started before (ros2 launch meldog_description meldog_chained_gazebo.launch.py).

State estimator, whole body controller and MPC controller, activated in this order:

  1. state estimator is configured and activated,
  2. WBC is activated when the estimator is active (WBC needs odometry, it stands without policy),
  3. MPC is activated when WBC is active (or the estimator, with use_wbc:=false).

All nodes are configured at start. Every node is activated only once, after its own
deactivation it has to be activated by hand (ros2 lifecycle set <node> activate).
With use_wbc:=false MPC sends joint trajectory to the robot (MRT control) and WBC is not started.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, PathJoinSubstitution

from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState, StateTransition
from launch_ros.events.matchers import matches_node_name
from launch_ros.substitutions import FindPackageShare

from lifecycle_msgs.msg import Transition

ESTIMATOR = '/legged_state_estimator'
WBC = '/legged_whole_body_controller'
MPC = '/legged_locomotion_mpc_controller'


def package_launch(package, launch_file, arguments, condition=None):
    # Scoped, otherwise launch arguments of one package (config_directory_path, urdf_path,
    # parameters_file, ...) would be used by the packages included after it
    return GroupAction(
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
            launch_arguments=arguments.items())],
        scoped=True,
        condition=condition)


def activation_sequencer():
    # Last transition (start state, goal state) of every node and nodes already activated
    transitions = {}
    activated = set()

    def configured(node):
        return transitions.get(node) == ('configuring', 'inactive')

    def active(node):
        return transitions.get(node, (None, None))[1] == 'active'

    def activate(node, reason):
        activated.add(node)
        return [
            LogInfo(msg=f'Activating {node} ({reason})'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_node_name(node),
                transition_id=Transition.TRANSITION_ACTIVATE)),
        ]

    def on_transition(context):
        event = context.locals.event
        transitions[event.action.node_name] = (event.start_state, event.goal_state)

        use_wbc = IfCondition(LaunchConfiguration('use_wbc')).evaluate(context)
        actions = []

        if use_wbc and WBC not in activated and configured(WBC) and active(ESTIMATOR):
            actions += activate(WBC, 'state estimator is active')

        mpc_prerequisite = WBC if use_wbc else ESTIMATOR
        if MPC not in activated and configured(MPC) and active(mpc_prerequisite):
            actions += activate(MPC, f'{mpc_prerequisite} is active')

        return actions

    return RegisterEventHandler(OnStateTransition(
        matcher=lambda event: isinstance(event, StateTransition),
        entities=[OpaqueFunction(function=on_transition)]))


def generate_launch_description():
    # Simulation only, robot launch will have its own file
    use_sim_time = 'true'
    use_wbc = LaunchConfiguration('use_wbc')

    arguments = [
        DeclareLaunchArgument(
            'use_wbc', default_value='true',
            description='true: MPC -> WBC -> robot, false: MPC (MRT control) -> robot'),
        DeclareLaunchArgument(
            'publish_initial_odometry', default_value='false',
            description='Publish initial base odometry for the state estimator'),
        DeclareLaunchArgument('initial_x', default_value='0.0', description='Initial base x [m]'),
        DeclareLaunchArgument('initial_y', default_value='0.0', description='Initial base y [m]'),
        DeclareLaunchArgument('initial_z', default_value='0.0', description='Initial base z [m]'),
    ]

    estimator = package_launch(
        'legged_state_estimator_ros2', 'legged_state_estimator.launch.py', {
            'use_sim_time': use_sim_time,
            'publish_initial_odometry': LaunchConfiguration('publish_initial_odometry'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_z': LaunchConfiguration('initial_z'),
        })

    # Configured at start, activated by the sequencer
    wbc = package_launch(
        'legged_whole_body_control_ros2', 'legged_whole_body_controller.launch.py', {
            'use_sim_time': use_sim_time,
            'activate': 'false',
        },
        condition=IfCondition(use_wbc))

    mpc = package_launch(
        'legged_locomotion_mpc_ros2', 'legged_locomotion_mpc_controller.launch.py', {
            'use_sim_time': use_sim_time,
            'use_mrt_control': NotSubstitution(use_wbc),
            'activate': 'false',
        })

    # Registered before nodes are started, so no transition is missed
    return LaunchDescription(arguments + [activation_sequencer(), estimator, wbc, mpc])
