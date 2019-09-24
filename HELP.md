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

## FOR INSTALLING REALSENSE-ROS (ROS Kinetic)
- Register the server's public key `sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE`
- Add the server to the list of repositories `sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u`
- Install the libraries and the developer/debug packages:
```
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```
- Clone the following to your catkin workspace:
```
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
```
- Finally, `catkin_make`