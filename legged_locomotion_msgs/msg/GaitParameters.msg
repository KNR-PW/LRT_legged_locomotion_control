# This is a message that holds data to describe dynamic gait parameters for legged robot. 
#
# The Gait is defined by:
#  * stepping frequency, how fast robot is taking steps (x > 0) (Hz)
#  * swing ratio, thats defines when leg is in contact or swing (1 > x > 0) 
#  * phase offsets between each leg (size(x) == leg number - 1) (1 > x[i] > 0)
#

float64 stepping_frequency
float64 swing_ratio
float64[] phase_offsets