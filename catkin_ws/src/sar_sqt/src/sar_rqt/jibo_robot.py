import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from sar_jibo_command_msgs.msg import JiboSpeech, JiboAnimation, JiboLookat, JiboHealth
from sar_robot_command_msgs.msg import RobotCommand
from std_msgs.msg import String, Float32
from std_msgs.msg import Header # standard ROS msg header

class jibo_gaze_control(Plugin):

    def __init__(self, context):
        super(jibo_gaze_control, self).__init__(context)

        rospy.Subscriber("/sar/jibo/health", JiboHealth , self.jibo_health_callback)

        self.jibo_lookat_pub = rospy.Publisher('/sar/jibo/lookat', JiboLookat, queue_size=10)
        self.jibo_speech_pub = rospy.Publisher('/sar/jibo/speech', JiboSpeech, queue_size=10)
        self.jibo_animation_pub = rospy.Publisher('/sar/jibo/animation', JiboAnimation, queue_size=10)
        self.jibo_command_pub = rospy.Publisher('/sar/robot_command', RobotCommand, queue_size=10)

        # Give QObjects reasonable names
        self.setObjectName('jibo_gaze_control')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('sar_rqt'), 'resource', 'JiboGazeControlPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('jibo_gaze_control_ui')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface


        self._widget.submit_button.clicked[bool].connect(self._handle_submit_clicked)
        self._widget.expressions_submit.clicked[bool].connect(self._handle_expressions_submit)
        self._widget.speech_submit.clicked[bool].connect(self._handle_speech_submit)


        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def jibo_health_callback(self, data):
        pass
        #cpu_temp_pub = rospy.Publisher('/sar/jibo/health/cpu_temperature', String, queue_size=1)
        # TODO: other jibo health info
        #_t = str(data.cpu_temperature)
        #rospy.loginfo(data.cpu_temperature)
        #_t.data = data.cpu_temperature
        #cpu_temp_pub.publish(float(_t))

    def _handle_submit_clicked(self):
        if(self._widget.x_text.text() == "") or (self._widget.y_text.text() == "") or (self._widget.z_text.text() == ""):
            os.system("notify-send -t 5 'One of the textboxes are blank!'")
        else:
            self.send_jibo_lookat(float(self._widget.x_text.text()), float(self._widget.y_text.text()), float(self._widget.z_text.text()))
            rospy.loginfo('lookat: ' + str(self._widget.x_text.text()) + ', ' + str(self._widget.y_text.text()) + ', ' + str(self._widget.z_text.text()))

    def _handle_speech_submit(self):
        # self.send_jibo_speech(str(self._widget.speech_text.toPlainText()), {})
        self.send_robot_command(RobotCommand.DO, str(self._widget.speech_text.toPlainText()))

    def _handle_expressions_submit(self):
        self.send_jibo_animation(str(self._widget.expressions_text.text()) + ".keys")

    def send_jibo_lookat(self, _x, _y, _z, _duration=-1):
        msg = JiboLookat()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.x = _x
        msg.y = _y
        msg.z = _z
        msg.duration = _duration
        self.jibo_lookat_pub.publish(msg)

    def send_jibo_speech(self, _content, _parameters):
        msg = JiboSpeech()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.speech_content = _content
        # default paramters for jibo speech
        _pitch = 7.6
        _bandwidth = 2.0
        _stretch = 1.07
        if _parameters: # dictionary is not empty
            _pitch = _parameters['pitch']
            _bandwidth = _parameters['pitchBandwidth']
            _stretch = _parameters['duration_stretch']
        msg.pitch = _pitch
        msg.pitch_bandwidth = _bandwidth
        msg.duration_stretch = _stretch
        self.jibo_speech_pub.publish(msg)

    def send_jibo_animation(self, _name, _n=1.0):
        msg = JiboAnimation()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.animation_name = _name
        msg.repeat_n = _n
        self.jibo_animation_pub.publish(msg)

    def send_robot_command(self, _type, _cmd):
        msg = RobotCommand()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = _type
        msg.properties = _cmd
        self.jibo_command_pub.publish(msg)
