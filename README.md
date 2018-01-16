People_Tracking_ParrotAR_Drone
-----
Code for autonomous people detection and tracking using a Parrot AR-Drone (version 2.0) as a part of a semester project jointly at Eurecom and Telecom ParisTech. People detection is done using Histogram of Gradient (HOG) feature and tracking with a particle filter. All the control commands are implemented in ROS Hydro using Open-CV library. For more details kindly contact - praveersingh1990atgmail.com , rana.aakankshaatgmail.com

CREDITS, LICENSE, CITATION
-----

Copyright © 2014 Praveer SINGH and Aakanksha RANA

All Rights Reserved. A license to use and copy this software and its documentation solely for your internal
research and evaluation  purposes, without fee and without a signed licensing agreement, is hereby granted
upon your download of the software, through which you agree to the following:
1)  the above copyright notice, this paragraph and the following three paragraphs will prominently appear
in all internal copies and modifications;
2)  no rights to sublicense or further distribute this software are granted;
3) no rights to modify this software are granted; and
4) no rights to assign this license are granted.

Please Contact the authors for commercial licensing opportunities, or for further distribution, modification or license rights.

Created by Praveer SINGH (https://scholar.google.com/citations?user=12RkAfQAAAAJ&hl=en) and Aakanksha RANA (https://scholar.google.fr/citations?user=_qF1ZsIAAAAJ&hl=en)

How to run the demos?
---------------------

#1. Install dependencies

[ ] Boost: "sudo apt-get install libboost-dev"
	because ROS (or ardrone_autonomy) using boost 1.46 in default, so we shouldn't use higher version of boost (1.54) 

[ ] SDL: "sudo apt-get install libsdl-mixer1.2-dev"

[ ] ROS: "www.ros.org/wiki/ROS/Installation"

[ ] ardrone_autonomy: "https://github.com/AutonomyLab/ardrone_autonomy#installation", but use "/opt/ros/groovy/stacks" instead of "~/ros/stacks"

after install ROS correctly, the enviroment variable ROS_PAKAGE_PATH (use command "env" to check):
ROS_PACKAGE_PATH=/opt/ros/groovy/share:/opt/ros/groovy/stacks

for the problem with compiling ardrone_autonomy "rosmake ardrone_autonomy" (mkdir: cannot create directory 'build': Permission denied)
https://github.com/AutonomyLab/ardrone_autonomy/issues/11
--> change permission of folder ardrone_autonomy
	sudo chmod 777 -R ardrone_autonomy/

[ ] install NTP server for synchronize time between 2 laptop
http://ubuntuforums.org/showthread.php?t=862620
https://www.digitalocean.com/community/articles/how-to-set-up-time-synchronization-on-ubuntu-12-04
http://rbgeek.wordpress.com/2012/04/30/time-synchronization-on-ubuntu-12-04lts-using-ntp/

FOR SERVER:
install ntpd:
sudo aptitude install ntpd

change server address to European  /etc/ntp.conf:
server 0.europe.pool.ntp.org
server 1.europe.pool.ntp.org
server 2.europe.pool.ntp.org
server 3.europe.pool.ntp.org
restart service:
sudo /etc/init.d/ntp restart

add this line into /etc/ntp.conf in order to other PCs can synchronize together
restrict 192.168.2.0 mask 255.255.255.0 nomodify notrap
because I manually setup 2 PC as 192.168.2.20 (server) and 192.168.2.30 (client)

open ports:
sudo iptables -A INPUT -p udp --dport 123 -j ACCEPT
sudo iptables -A OUTPUT -p udp --sport 123 -j ACCEPT

FOR CLIENT:
update time from client to server:
ntpdate 192.168.2.20 

we can check by change time in client, update and see what will happen.

#2. Ensure the right camera calibration for your drone

[ ] before using a new drone for the first time, calibrate both of its cameras: "www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration"
[ ] initially and when switching your drone, copy or symlink "data/calib<DRONE'S_SERIAL_NUMBER>/*" into "~/.ros/camera_info/"
my Dir: /opt/ros/stacks/ardrone_autonomy/data/camera_info

#3. Build

[ ] "cd code/libHawaii && make && cd ../demoARDrone && make", adjust paths in "{libHawaii,demoARDrone}/config.mk" in case of missing headers or libraries

Linking to boost library (filesystem, iostreams, serialization). Add these line into demoDrone/config.mk
LIBS_ALL += -lboost_iostreams
LIBS_ALL += -lboost_serialization
LIBS_ALL += -lboost_filesystem
This is the example in case we want to link to another libraries of boost

#4. Run

[ ] run ROS middleware: "roslaunch data/ardrone.launch"
[ ] run the actual demo: "./bin/demoARDrone"

if we have any problem with gamepad in run-time: Logitech F{3,5,7}10 or Microsoft Xbox gamepad in XInput mode required, in case don't have any gamepad to test, we can remove this asser error in file appBase.cpp
HAWAII_ERROR_CONDITIONAL( strcmp( SDL_JoystickName( 0 ), "Generic X-Box pad" ) != 0
		                       || SDL_JoystickNumAxes(    gamepadHandle ) != 6
		                       || SDL_JoystickNumBalls(   gamepadHandle ) != 0
		                       || SDL_JoystickNumButtons( gamepadHandle ) != 11 // developer note: I only found eight.
		                       || SDL_JoystickNumHats(    gamepadHandle ) != 1,
		                          "Logitech F{3,5,7}10 or Microsoft Xbox gamepad in XInput mode required" ) ;
---> but lead to a segmentation fault (don't know how to fixed it yet)
