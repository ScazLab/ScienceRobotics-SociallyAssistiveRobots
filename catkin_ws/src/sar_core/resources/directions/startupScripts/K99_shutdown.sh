#!/bin/sh
# startupScript.sh
# find python script, execute
# http://ccm.net/faq/3348-execute-a-script-at-startup-and-shutdown-on-ubuntu

exec 2> /tmp/rc.localoff.log
exec 1>&2
set -x


echo "sending off"
cd /home/sar/catkin_ws/src/sar_core/nodes
python online.py down
echo "sending done"
exit 0
