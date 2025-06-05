# Reference frame_id. Landmark coordinates are relative to this frame.
#   Defaults to /map
string frame_id "/map"

# ID: Integer. Will be used as node ID in the graph.
#   ID must be a positive integer with a max value of 2147483647.
#   The default value of -1 is used to represent a landmark with unknown ID
#   Landmarks with unknown ID will be automatically assigned a positive ID when added to the graph
int32 id -1

# Type: Landmark type. Possible options:
#    * PointXY: Point in 2D space without orientation
#    * Pose2D: Pose in 2D space with orientation
uint8 TYPE_XY = 0  # With this type, theta value is ignored
uint8 TYPE_SE2 = 1
uint8 type

# Landmark coordinates
float64 x 0.0
float64 y 0.0
float64 theta 0.0
