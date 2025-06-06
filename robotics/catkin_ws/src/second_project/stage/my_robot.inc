define kinect ranger
(
  sensor
  (
    range_max 8.0
    fov 360.0
    samples 640
  )
  # generic model properties
  color "green"
  size [ 0.06 0.15 0.03 ]
)

define my_robot position
(
  size [0.54 0.40 0.10]
  color "blue"
  drive "diff"
  obstacle_return 1

  kinect()  
)

