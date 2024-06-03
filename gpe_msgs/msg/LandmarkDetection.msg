# Reference frame_id. Landmark coordinates are relative to this frame.
#   It should be the sensor or the robot frame.
#   Defaults to /base_link
string frame_id "/base_link"

# Detection information:
# ID: Integer. Landmark ID.
#   It is set to -1 if the landmark has not been identified.
int32 id -1

# Type: Landmark type. Possible options:
#    * PointXY: [x, y]
#    * PointXYZ: [x, y, z]
#    * Pose2D: [x, y, yaw]
#    * Pose3D: [x, y, z, roll, pitch, yaw]
#    * Pose3DQuat: [x, y, z, qx, qy, qz, qw]
# NOTE: At this moment only PointXY landmarks are supported.

# Coords: Array of detection coordinates. Length and meaning depend on type.
#   Coordinates are relative to the frame_id, which should be the sensor or the robot frame
# float64[] coords
# Landmark cooridnates
float64 x
float64 y

# Covariance: A size-4 array representing a 2x2 (2D) covariance matrix
# float64[] covariance
float64[4] covariance
