define kinect ranger
(
  sensor
  (
    range [0 100]
    fov 360.0
    samples 1080
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.01 0.01 999999 999999 999999 0.01]

  size [ 0.54 0.40 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "green"
  
  kinect(pose [ 0.0 0.0 0.0 0.0 ])
)