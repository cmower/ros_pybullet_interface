std_msgs/Header header
string link_name
float64[] targetPosition
float64[] targetOrientation
float64[] lowerLimits
float64[] upperLimits
float64[] jointRanges
float64[] resetPoses
float64[] jointDamping
int64 solver  # use 0 for p.IK_DLS or 1 for p.IK_SDLS
float64[] currentPosition
int64 maxNumIterations
float64 residualThreshold

float64 dt  # used to calculate joint velocity (vel=(solution-currentPosition)/dt), this must be given if robot is in velocity control