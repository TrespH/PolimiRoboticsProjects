define kinect ranger
(
  sensor
  (
    range_max 8.0
    fov 360.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define my_robot position
(
  size [0.54 0.40 0.10]

  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.005 0.005 0 0 0 0.005]

  origin [ 0.0 0.0 0.0 0.0 ]

  gui_nose 1
  color "blue"
  drive "omni"
  obstacle_return 1

  kinect(pose [ 0 0.0 0 0.0 ])
)

