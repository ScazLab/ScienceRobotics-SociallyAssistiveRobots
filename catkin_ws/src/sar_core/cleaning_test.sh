#!/bin/bash

# clean ros log
rosclean purge

# empty trash can
rm -rf ~/.local/share/Trash/*

# remove ros bags in sar_core
echo "removing rosbag logs"
rm -f /home/sar/catkin_ws/src/sar_core/logs/rosbags/*.bag

rm /home/sar/catkin_ws/src/sar_core/daily_execution_log.txt

rm /home/sar/.ros/ss_debug.log
rm /home/sar/.ros/ss_error.log

rm /home/sar/catkin_ws/src/sar_core/config/performance_log.json
rm /home/sar/catkin_ws/src/sar_core/logs/vja_log.json

rm /home/sar/catkin_ws/src/sar_core/logs/session_data.json
touch /home/sar/catkin_ws/src/sar_core/logs/session_data.json
echo '{"sessions": []}' > /home/sar/catkin_ws/src/sar_core/logs/session_data.json

rm /home/sar/catkin_ws/src/sar_core/logs/session_temp_log.json
touch /home/sar/catkin_ws/src/sar_core/logs/session_temp_log.json
echo '{"datetime": "", "game1": {"id":"", "status": ""}, "game2": {"id":"", "status": ""}, "game3": {"id":"", "status": ""}, "pj": ""}' > /home/sar/catkin_ws/src/sar_core/logs/session_temp_log.json

rm /home/sar/catkin_ws/src/sar_social_stories/src/socialstories.db
cd /home/sar/catkin_ws/src/sar_social_stories/src
python ss_init_db.py

echo "DONE"
