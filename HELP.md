## FOR INSTALLING OPENNI2
`sudo apt install ros-melodic-rgbd-launch ros-melodic-openni2-*`
`sudo apt install ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-rqt-robot-plugins`

## RUNNING OPENNI2
`roslaunch openni2_launch openni2.launch`

### Viewing the image from RQT
rqt -> Plugins -> Visualization -> Image View -> select topic

## CAMERA CALIBRATION
`rosrun camera_calibration cameracalibrator.py --size 5x7 --square 0.048 image:=/camera/rgb/image_raw camera:=/camera/rgb/`