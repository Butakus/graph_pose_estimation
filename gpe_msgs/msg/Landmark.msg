# Reference frame_id. Landmark coordinates are relative to this frame. Defaults to /map
string frame_id "/map"

# Landmark information:
# ID: Integer. Will be used as node ID in the graph.
#     ID must be a positive integer with a max value of 2147483647.
# Type: Landmark type. Possible options:
#    * PointXY: [x, y]
#    * PointXYZ: [x, y, z]
#    * Pose2D: [x, y, yaw]
#    * Pose3D: [x, y, z, roll, pitch, yaw]
#    * Pose3DQuat: [x, y, z, qx, qy, qz, qw]
# NOTE: At this moment only PointXY landmarks are supported.
# Coords: Array of coordinates. Length and meaning depend on type.
int32 id

# Landmark cooridnates
float64 x
float64 y
