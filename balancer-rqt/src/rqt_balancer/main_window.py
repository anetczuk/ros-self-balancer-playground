import os
import rospy
import rospkg
from std_msgs.msg import Float64

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from .pid_widget import PidWidget


class MainWindow(Plugin):

    def __init__(self, context):
        super(MainWindow, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MainWindow')
        
        self._parse_cmd_args(context)
        self._init_widget(context)
        
#         self.angle_pub = rospy.Publisher('/self_balancer/angle_pid/setpoint', Float64, queue_size=10, latch=True)
#         self.kp_pub = rospy.Publisher('/self_balancer/angle_pid/kp', Float64, queue_size=10, latch=True)
#         self.ki_pub = rospy.Publisher('/self_balancer/angle_pid/ki', Float64, queue_size=10, latch=True)
#         self.kd_pub = rospy.Publisher('/self_balancer/angle_pid/kd', Float64, queue_size=10, latch=True)
#         
#         self._widget.angleSB.valueChanged.connect( self._angleSB_changed )
#         self._widget.kpSB.valueChanged.connect( self._kpSB_changed )
#         self._widget.kiSB.valueChanged.connect( self._kiSB_changed )
#         self._widget.kdSB.valueChanged.connect( self._kdSB_changed )

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
        
    def _parse_cmd_args(self, context):
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print( 'arguments: ', args)
            print( 'unknowns: ', unknowns)
    
    def _init_widget(self, context):
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_balancer'), 'resource', 'MainWindow.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        self._widget.pitchPid = PidWidget( self._widget, "pitch_pid" )
        #self._widget.speedPid = PidWidget( self._widget, "speed_pid" )
        
        # Add widget to the user interface
        context.add_widget(self._widget)
