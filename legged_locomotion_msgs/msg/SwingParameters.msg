# This is a message that holds data to describe dynamic swing parameters for legged robot. 
#
# The Swing is defined by:
#  * inverted pendulum height, that defines base height perpendicular to terrain
#  * swing heights is a maximum height of foots at midpoint of swing phase
#  * phases, time progress for midpoints in the swing phase in [0, 1)
#  * tangential progresses, swing progress for midpoints in tangential direction in [0, 1]
#  * tangential velocity factors, Tangantial velocity as a factor of the average velocity. The tangential velocity will be: velocityFactor * swingdistance / dt
#

float64 inverted_pendulum_height
float64[] swing_heights
float64[] phases
float64[] tangential_progresses
float64[] tangential_velocity_factors