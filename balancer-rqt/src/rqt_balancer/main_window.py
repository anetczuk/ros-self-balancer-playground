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

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        self.pitchWidget.save_settings(plugin_settings)
        self.speedWidget.save_settings(plugin_settings) 

    def restore_settings(self, plugin_settings, instance_settings):
        self.pitchWidget.restore_settings(plugin_settings)
        self.speedWidget.restore_settings(plugin_settings)

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
        
        self.pitchWidget = PidWidget( self._widget.pitchPid, "pitch_pid" )
        self.speedWidget = PidWidget( self._widget.speedPid, "speed_pid" )
#         sumWidget = PidWidget( self._widget.pitchPid, "sum_pid" )
        
        # Add widget to the user interface
        context.add_widget(self._widget)
