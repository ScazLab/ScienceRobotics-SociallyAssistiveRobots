#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sar_jibo_command_msgs.msg import JiboHealth

from server_communication import Packet
from server_communication import Communication

class robot_health_monitor(object):

    def __init__(self):
        rospy.init_node('robot_health_monitor', anonymous=True)
        self._send_signals_to_server = rospy.get_param('/sar/global/_send_signals_to_server')
        rospy.Subscriber("/sar/jibo/health", JiboHealth, self.jibo_health_callback)


    def jibo_health_callback(self, data):
        _cpu_temp = data.cpu_temperature
        _battery_temp = data.battery_temperature
        _main_board_temp = data.main_board_temperature

        rospy.loginfo(_cpu_temp)

        _alert_cpu_temp = 75.0 #Change this

        if _cpu_temp >= _alert_cpu_temp: # auto shutdown when it is over 95
            if self._send_signals_to_server:
                pack = Packet("Robot Over Alert Temperature: " + str(_cpu_temp), 2)
                pack.add([{"dataset" : "message", "numerical" : 0}]).send()
            rospy.logwarn("Robot Over Alert Temperature: " + str(_cpu_temp))

        if self._send_signals_to_server:
            pack = Packet("Health Information", 0)
            pack.add([{"dataset":"cpu_temperature","numerical": _cpu_temp },
                {"dataset":"battery_temperature", "numerical":_battery_temp},
                {"dataset":"main_board_temperature", "numerical":_main_board_temp
                }]).send()
        rospy.loginfo("robot cpu_temperature: " + str(_cpu_temp))
        rospy.loginfo("robot battery_temperature: " + str(_battery_temp))
        rospy.loginfo("robot main_board_temperature: " + str(_main_board_temp))

    
    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    # if self._send_signals_to_server:
    #     pack = Packet("Good Morning", 2)
    #     pack.add([{"dataset" : "message", "numerical" : "1"}]).send()
    try:
        rm = robot_health_monitor()
        rm.run()
    except rospy.ROSInterruptException:
        pass
    # if self._send_signals_to_server:
    #     pack = Packet("Good Night", 2)
    #     pack.add([{"dataset" : "message", "numerical" : "1"}]).send()
