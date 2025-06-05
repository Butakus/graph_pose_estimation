# Reference frame_id. Landmark coordinates are relative to this frame.
#   Defaults to /map
string frame_id "/map"

# Landmark information:
# ID: Integer. Will be used as node ID in the graph.
#     ID must be a positive integer with a max value of 2147483647.
int32 id

# Type: Landmark type. Possible options:
#    * PointXY: Point in 2D space without orientation
#    * Pose2D: Pose in 2D space with orientation
uint8 TYPE_XY = 0  # With this type, theta value is ignored
uint8 TYPE_SE2 = 1
uint8 type

# Landmark cooridnates
float64 x
float64 y
float64 theta
