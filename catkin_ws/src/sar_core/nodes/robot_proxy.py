#!/usr/bin/env python

import threading
import rospy
from std_msgs.msg import String
from sar_jibo_command_msgs.msg import JiboHealth, JiboLookat, JiboAnimation
from sar_core.msg import SystemState

from server_communication import Packet
from server_communication import Communication

class robot_proxy(object):

    def __init__(self):
        rospy.init_node('robot_proxy', anonymous=True)
        self._send_signals_to_server = rospy.get_param('/sar/global/_send_signals_to_server')
        self._robot_id = rospy.get_param('/sar/global/_robot_id')
        self._default_notification_level = rospy.get_param('/sar/global/_default_notification_level')

        self.lock_robot_attention_target = threading.Lock()
        self.estimated_robot_attention_target = None
        self.previous_robot_attention_target = None
        self._f_system_down = False

        rospy.Subscriber("/sar/jibo/health", JiboHealth, self.jibo_health_callback)
        rospy.Subscriber('/sar/jibo/verbose_state', String, self.jibo_verbose_state_callback)

        rospy.Subscriber("/sar/system/state", SystemState, self.system_state_callback)

        # estimate robot attention
        self.robot_attention_target_pub = rospy.Publisher('/sar/jibo/robot_attention_target', String, queue_size=10)

        if self._send_signals_to_server:
            pack = Packet("Good Morning", self._default_notification_level, self._robot_id)
            pack.add([{"dataset" : "message", "numerical" : "1"}]).send()


    def system_state_callback(self, data):
        _sys_state = data.system_state
        rospy.loginfo("system_state = %d", _sys_state)
        if _sys_state == SystemState.SYSTEM_DOWN:
            self._f_system_down = True


    # TODO: test this method
    # the best way is to have the robot to provide the info about its attentional target
    def jibo_verbose_state_callback(self, data):
        with self.lock_robot_attention_target:
            if 'playing animation' in data.data:
                if 'started' in data.data:
                    anim = data.data[19:-10]
                    rospy.loginfo("anim = " + anim)
                    if ('screen' in anim) or ('game' in anim):
                        self.estimated_robot_attention_target = 'screen'
                    else:
                        self.estimated_robot_attention_target = 'child'
            elif 'looking at' in data.data:
                if 'started' in data.data:
                    xyz = data.data[12:-10].split(',')
                    _x = float(xyz[0])
                    _y = float(xyz[1])
                    _z = float(xyz[2])
                    # TODO: modify the screen and parent locations if necessary
                    if (abs(_x - 0.15) < 0.1) and (abs(_y + 0.65) < 0.1) and (abs(_x - 0.25) < 0.1):
                        self.estimated_robot_attention_target = 'screen'
                        rospy.loginfo('robots attention is on screen')
                    # TODO: dynamically update these values
                    elif (abs(_x - 1) < 0.1) and (abs(_y - 0.5) < 0.1) and(abs(_z - 1) < 0.1):
                        self.estimated_robot_attention_target = 'parent'
                    else:
                        self.estimated_robot_attention_target = 'child' # TODO: may want to do some test
            elif 'face following' in data.data:
                self.estimated_robot_attention_target = 'child'

            self.previous_robot_attention_target = self.estimated_robot_attention_target


    def jibo_health_callback(self, data):
        _cpu_temp = data.cpu_temperature
        _battery_temp = data.battery_temperature
        _main_board_temp = data.main_board_temperature

        # rospy.loginfo(_cpu_temp)

        _alert_cpu_temp = 75.0 # alert the admin when the cpu temp is higher than 75

        if _cpu_temp >= _alert_cpu_temp: # TODO: auto shutdown when it is over 95
            if self._send_signals_to_server:
                pack = Packet("Alert! Robot Over Temperature: " + str(_cpu_temp), 2, self._robot_id)
                pack.add([{"dataset" : "message", "numerical" : str(_cpu_temp)}]).send()
            rospy.logwarn("Alert! Robot Over Temperature: " + str(_cpu_temp))

        if self._send_signals_to_server:
            pack = Packet("Robot Health Information", self._default_notification_level, self._robot_id)
            pack.add([{"dataset":"cpu_temperature","numerical": _cpu_temp },
                {"dataset":"battery_temperature", "numerical":_battery_temp},
                {"dataset":"main_board_temperature", "numerical":_main_board_temp
                }]).send()

        rospy.loginfo("robot cpu_temperature: " + str(_cpu_temp))
        rospy.loginfo("robot battery_temperature: " + str(_battery_temp))
        rospy.loginfo("robot main_board_temperature: " + str(_main_board_temp))


    def run(self):
        rate = rospy.Rate(10) # 30hz, is this too often?
        while not rospy.is_shutdown():
            if self._f_system_down:
                break

            with self.lock_robot_attention_target:
                if self.estimated_robot_attention_target != None:
                    self.robot_attention_target_pub.publish(self.estimated_robot_attention_target)
            rate.sleep()
        if self._send_signals_to_server:
            pack = Packet("Good Night", self._default_notification_level, self._robot_id)
            pack.add([{"dataset" : "message", "numerical" : "1"}]).send()


if __name__ == '__main__':
    try:
        rp = robot_proxy()
        rp.run()
    except rospy.ROSInterruptException:
        pass
