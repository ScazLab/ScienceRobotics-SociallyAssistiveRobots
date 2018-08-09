# put this file in where the target directory to save the data.
#!/bin/bash

mkdir -p sar_core/logs/
cp -a ~/catkin_ws/src/sar_core/logs/* sar_core/logs/
rm -f ~/catkin_ws/src/sar_core/logs/rosbags/*

mkdir -p sar_core/config
cp ~/catkin_ws/src/sar_core/config/performance_log.json sar_core/config

mkdir -p sar_social_stories/src/
cp ~/catkin_ws/src/sar_social_stories/src/socialstories.db sar_social_stories/src/

mkdir -p ros/logs/
mv ~/.ros/log/* ros/logs/
mv ~/.ros/ss_debug.log ros
mv ~/.ros/ss_error.log ros
