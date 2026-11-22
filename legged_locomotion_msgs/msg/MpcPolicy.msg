# This is a message that holds MPC policy for whole body controller.
#
# The policy is defined by:
#  * header, stamp is the start of MPC time (MPC times are relative to it), frame_id is the world frame
#  * policy, all times (time_trajectory, mode_schedule.event_times, init_observation.time)
#    are relative to header.stamp
#  * normals, terrain normals of end effectors in world frame at every point of policy.time_trajectory,
#    normals[k * end_effector_number + i] is normal of end effector i at time_trajectory[k]
#

std_msgs/Header header
ocs2_msgs/MpcFlattenedController policy
geometry_msgs/Vector3[] normals
