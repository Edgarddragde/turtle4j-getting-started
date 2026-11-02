
## EDGARD SETUP
need to connect via ethernet (couldn't connect or find wifi via wireless, eduroam stops us from doing this)
plug ethernet into raspberry pi behind robot and use ethernet to usb-c adapter to laptop to be able to ssh
go to network and see ethernet connection AX88179A and set 
ipv4 configured -> manually
ip address -> 192.168.185.2
subnet mask -> 255.255.0.0
leave everything else the same

maybe exit terminal but then ssh ubuntu@192.168.185.3 will work (.2 is your laptop and .3 is the robot)
sudo apt update
sudo apt install ros-humble-rosbridge-suite
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

then run -> ros2 topic echo /ip
if that didn't work then we need to edit release
Wifi: CP-IoT-Secure
Password: tRYWX466Qs
