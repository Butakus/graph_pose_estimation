# Reference frame_id. Landmark coordinates are relative to this frame.
#   Defaults to /map
string frame_id "/map"

# Landmark information:
# ID: Integer. Will be used as node ID in the graph.
#     ID must be a positive integer with a max value of 2147483647.
int32 id

# Type: Landmark type. Possible options:
#    * PointXYZ: Point in 3D space without orientation
#    * Pose3D: Pose in 3D space with orientation
# NOTE: At this moment only PointXY landmarks are supported.
uint8 TYPE_XYZ = 0  # With this type, pose.orientation is ignored
uint8 TYPE_SE3 = 1
uint8 type

# Landmark cooridnates
geometry_msgs/Pose pose
