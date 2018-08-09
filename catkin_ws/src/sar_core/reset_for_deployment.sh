#!/bin/bash

echo "reset/clean system for deployment"

# clean ros log
rosclean purge

cd /home/sar/Downloads
rm -rf *

cd /home/sar/Documents
rm -rf *

cd /home/sar/face_analyzer_data
rm -rf *

cd /home/sar/face_analyzer_assessment
rm -rf *

cd /home/sar/src/sar-games
rm -rf *

# empty trash can
rm -rf ~/.local/share/Trash/*

# remove ros bags in sar_core
echo "removing rosbag logs"
rm -f /home/sar/catkin_ws/src/sar_core/logs/rosbags/*.bag

rm /home/sar/catkin_ws/src/sar_core/daily_execution_log.txt

rm /home/sar/.ros/ss_debug.log
rm /home/sar/.ros/ss_error.log

# echo "reseting IP address for games"
# cd /home/sar/catkin_ws/src/sar_core
# python src/update_ips.py
