# Detected landmark. Coordinates are relative to landmark.frame_id.
Landmark2D landmark

# Row-major 3x3 covariance matrix (x, y, theta).
#   For landmark.TYPE_XY, only the top-left 2x2 block is meaningful.
float64[9] covariance
