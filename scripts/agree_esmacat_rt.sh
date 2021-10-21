#!/bin/bash          
#
# Script to launch (with delay) the AGREE esmacat rt.
# 
# Stefano Dalla Gasperina
# Tue 19 Oct 2021 
#
#

#_term() {
#  echo "Caught SIGTERM signal!"
#  kill -TERM "$child" 2>/dev/null
#}

#trap _term SIGTERM

echo "Running the real-time program"
cd /home/esmacat/esmacat_rt/esmacat_master_high_performance-release/esmacat_applications/agree_esmacat_rt
#echo esmacat | exec sudo -S ./run.sh &
exec sudo /home/esmacat/esmacat_rt/esmacat_master_high_performance-release/esmacat_applications/agree_esmacat_rt/agree_esmacat_rt &
exec sudo /home/esmacat/esmacat_rt/esmacat_master_high_performance-release/esmacat_core/application/ecat_main


#child=$!
#wait "$child"

##!/bin/bash
#echo "Doing some initial work...";
#/bin/start/main/server --nodaemon &



#function showHelp(){
#    echo
#    echo "This script can delay the launch of a roslaunch file"
#    echo "Place it in the 'scripts' folder of your catkin package"
#    echo "and make sure that the file is executable (chmod +x timed_roslaunch.sh)"
#    echo
#    echo "Run it from command line:"
#    echo
#    echo "Use: ./timed_roslaunch.sh [number of seconds to delay] [rospkg] [roslaunch file] [arguments (optional)]"
#    echo "Or: rosrun [yourpackage] time_roslaunch.sh [number of seconds to delay] [rospkg] [roslaunch file] [arguments (optional)]"
#    echo "Example: ./timed_roslaunch.sh 2 turtlebot_navigation amcl_demo.launch initial_pose_x:=17.0 initial_pose_y:=17.0"
#    echo
#    echo "Or run it from another roslaunch file:"
#    echo
#    echo '<launch>'
#    echo '  <arg name="initial_pose_y" default="17.0" />'
#    echo '  <node pkg="semantic_turtle_test" type="timed_roslaunch.sh"'
#    echo '    args="2 turtlebot_navigation amcl_demo.launch initial_pose_x:=17.0 initial_pose_y:=$(arg initial_pose_y)"'
#    echo '    name="timed_roslaunch" output="screen">'
#    echo '  </node>'
#    echo '</launch>'
#}
#
#if [ "$1" = "-h" ]; then
#    showHelp
#else
#    echo "start wait for $1 seconds"
#    sleep $1
#    echo "end wait for $1 seconds"
#    shift

#fi

