# Reference frame_id. Landmark coordinates are relative to this frame.
#   Defaults to /map
string frame_id "/map"

# ID: Integer. Will be used as node ID in the graph.
#   ID must be a positive integer with a max value of 2147483647.
#   The default value of -1 is used to represent a landmark with unknown ID
#   Landmarks with unknown ID will be automatically assigned a positive ID when added to the graph
int32 id -1

# Type: Landmark type. Possible options:
#    * PointXYZ: Point in 3D space without orientation
#    * Pose3D: Pose in 3D space with orientation
# NOTE: At this moment only PointXY landmarks are supported.
uint8 TYPE_XYZ = 0  # With this type, pose.orientation is ignored
uint8 TYPE_SE3 = 1
uint8 type

# Landmark coordinates
geometry_msgs/Pose pose
