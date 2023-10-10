import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import String
from wf_msgs.srv import Button


class WFPlugin(Plugin):

    def __init__(self, context):
        super(WFPlugin, self).__init__(context)
        self.setObjectName('WFPlugin')

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            pass
            
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('wf_gui'), 'resource', 'WFPlugin.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('WFPluginUi')

        ## button setting
        ## follow button setting
        self._widget.follow_right_Button.clicked.connect(self.followRightButtonCB)

        ## save button setting
        self._widget.follow_left_Button.clicked.connect(self.FollowLeftButtonCB)

        self._widget.stopButton.clicked.connect(self.stopButtonCB)

        ## subscriber setting
        rospy.Subscriber("/wf/status", String, self.labelCB)

        rospy.wait_for_service('/wf/button')
        self.button_click = rospy.ServiceProxy('/wf/button', Button)

        ## service setting
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        context.add_widget(self._widget)

    ###### Label callback function #####
    def labelCB(self, data):
        self._widget.status_Label.setText(data.data)

    ############ Button Callback for follow path ##############

    def followRightButtonCB(self):
        push_button = "followR"
        self.button_click(push_button)

    ############ Button Callback for save path ##############

    def followLeftButtonCB(self):
        push_button = "followL"
        self.button_click(push_button)

    def stopButtonCB(self):
        push_button = "stop"
        self.button_click(push_button)

    def aioButtonCB(self):
        push_button = "aio"
        self.button_click(push_button)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass