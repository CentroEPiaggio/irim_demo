## FOR INSTALLING OPENNI2
`sudo apt install ros-melodic-rgbd-launch ros-melodic-openni2-*`
`sudo apt install ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-rqt-robot-plugins`

## RUNNING OPENNI2
`roslaunch openni2_launch openni2.launch`

### Viewing the image from RQT
rqt -> Plugins -> Visualization -> Image View -> select topic

## CAMERA CALIBRATION
`rosrun camera_calibration cameracalibrator.py --size 5x7 --square 0.048 image:=/camera/rgb/image_raw camera:=/camera/rgb/`

Tutorial: http://wiki.ros.org/camera_calibration/Tutorials
Other useful links:     http://wiki.ros.org/camera_calibration_parsers
                        http://wiki.ros.org/image_proc#image_proc.2BAC8-fuerte.image_proc.launch

## ARUCO ROS TESTING
For one marker with known id: `roslaunch irim_demo singleirim.launch`
For multiple markers: `roslaunch irim_demo marker_publisher_irim.launch` (Seems not to publish tf)