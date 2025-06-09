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

  odom_error [0.01 0.01 999999 999999 999999 0.01]

  origin [ 0.0 0.0 0.0 0.0 ]

  gui_nose 1
  color "blue"
  drive "diff"
  obstacle_return 1

  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
)

