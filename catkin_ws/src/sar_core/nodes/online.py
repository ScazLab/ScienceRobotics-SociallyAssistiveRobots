#!/usr/bin/env python

from server_communication import Packet
from server_communication import Communication
import sys
import time
#/etc/network/if-up.d/startupSAR
import xml.etree.ElementTree


if __name__ == "__main__":
	e = xml.etree.ElementTree.parse('/home/sar/catkin_ws/src/sar_core/launch/sar_core.launch').getroot()
	for aparam in e.findall('param'):
		if(aparam.get('name') == '/sar/global/_robot_id'):
			_robot_id = aparam.get('value')
		if(aparam.get('name') == '/sar/global/_default_notification_level'):
			_level = aparam.get('value')
	if(sys.argv[1] == "up"):
		pack = Packet("System Online", _level, _robot_id)
		pack.add([{"dataset" : "message", "numerical" : int(time.time()) }]).send()
	elif(sys.argv[1] == "down"):
		pack = Packet("System Offline", _level, _robot_id)
		pack.add([{"dataset" : "message", "numerical" : int(time.time()) }]).send()
