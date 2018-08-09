#!/bin/bash

echo "Running SAR application"

# read log and check date
# PLAYED_TODAY=0
# file="/home/sar/catkin_ws/src/sar_core/daily_execution_log.txt"
# while IFS= read -r line
# do
#         # display $line or do somthing with $line
#     if [ "$line" = $(date +%Y-%m-%d) ]
# 	then
# 	    PLAYED_TODAY=1
# 	fi
# done <"$file"

output=$(python src/check_daily_execution.py 2>&1)
echo $output

if [ $output = "bad" ]
then
	echo "You've already played today! Play again tomorrow!"
	sleep 5s
	/usr/bin/dbus-send --system --print-reply --dest="org.freedesktop.ConsoleKit" /org/freedesktop/ConsoleKit/Manager org.freedesktop.ConsoleKit.Manager.Stop
else
	echo "Welcome! Enjoy playing!"
	# echo $(date +%Y-%m-%d) >> "$file"
fi


source /home/sar/catkin_ws/devel/setup.bash
export ROS_IP=192.168.2.3
export ROS_MASTER_URI=http://192.168.2.3:11311

cd /home/sar/catkin_ws/src/expedition-games
http-server&

roslaunch sar_core sar_core.launch


# https://askubuntu.com/questions/187071/how-do-i-shut-down-or-reboot-from-a-terminal/187080
python /home/sar/catkin_ws/src/sar_core/nodes/online.py down
sleep 5s
/usr/bin/dbus-send --system --print-reply --dest="org.freedesktop.ConsoleKit" /org/freedesktop/ConsoleKit/Manager org.freedesktop.ConsoleKit.Manager.Stop


#exit
