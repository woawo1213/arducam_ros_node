# ArduCAM USB2 & USB3 Shield ROS Node

(Tested on Ubuntu 18.04, ROS Melodic)

## Setup:
1. git clone https://bitbucket.org/wonikrobotics/arducam_camera.git
2. cd arducam_camera/src/arducam_config_parser
3. cp libarducam_config_parser.so /usr/lib
4. cd ../Arducam_SDK/
5. cp libArduCamLib.so libArduCamLib.so.2 libArduCamLib.so.2.0.0 /usr/lib
6. cd ..
7. cp arducam.rules /etc/udev/rules.d
8. sudo service udev reload
9. sudo service udev restart

## Usage:
1. roslaunch arducam_camera arducam_node.launch //single camera
2. roslaunch arducam_camera multiarducam_node.launch //multi camera

## Single camera example
- ### Topic list:
* /arducam_camera/camera_info
* /arducam_camera/image_raw
* /arducam_camera/image_raw/compressed
* /arducam_camera/image_raw/theora
* /arducam_camera/image_raw/theora/compressed

## More Information
[Notion]: https://half-rubidium-aa9.notion.site/Arducam-a09d2835176149f0be1e3560b79dd68a

