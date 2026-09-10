# Detected landmark. Coordinates are relative to landmark.frame_id.
Landmark3D landmark

# Row-major 6x6 covariance matrix (x, y, z, roll, pitch, yaw).
#   For landmark.TYPE_XYZ, only the top-left 3x3 block is meaningful.
float64[36] covariance
