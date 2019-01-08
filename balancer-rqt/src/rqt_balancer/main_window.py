import os
import rospy
import rospkg
from std_msgs.msg import String

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from .pid_single_widget import PidSingleWidget
from .pid_cascade_widget import PidCascadeWidget
from .fuzzy_widget import FuzzyWidget


class MainWindow(Plugin):

    def __init__(self, context):
        super(MainWindow, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MainWindow')
        
        self.driver_type_pub = rospy.Publisher('/self_balancer/driver_type', String, queue_size=10, latch=True)
        
        self._parse_cmd_args(context)
        self._init_widget(context)       

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        widgetsCount = self._mainWindowUi.driverWidget.count()
        for i in range(0, widgetsCount):
            driver = self._mainWindowUi.driverWidget.widget(i)
            driver.save_settings(plugin_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        widgetsCount = self._mainWindowUi.driverWidget.count()
        for i in range(0, widgetsCount):
            driver = self._mainWindowUi.driverWidget.widget(i)
            driver.restore_settings(plugin_settings)

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
        self._mainWindowUi = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_balancer'), 'resource',  self.__class__.__name__ + '.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._mainWindowUi)
        # Show _mainWindowUi.windowTitle on left-top of each plugin (when 
        # it's set in _mainWindowUi). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._mainWindowUi.setWindowTitle(self._mainWindowUi.windowTitle() + (' (%d)' % context.serial_number()))
        
        self._mainWindowUi.driverCB.currentIndexChanged.connect( self._driverChanged )
        
        self._init_drivers()
       
        # Add widget to the user interface
        context.add_widget(self._mainWindowUi)

    def _driverChanged(self):
        driver_type = self._mainWindowUi.driverCB.currentText()
        rospy.loginfo("selecting driver: %r", driver_type )
        index = self._mainWindowUi.driverCB.currentIndex()
        self._mainWindowUi.driverWidget.setCurrentIndex( index )
        self.driver_type_pub.publish( str(driver_type) )
    
    def _init_drivers(self):
        singlePidDriver = PidSingleWidget( self._mainWindowUi.driverWidget )
        self._create_driver_widget("PID_SINGLE", singlePidDriver)
        
        cascadePidDriver = PidCascadeWidget( self._mainWindowUi.driverWidget )
        self._create_driver_widget("PID_CASCADE", cascadePidDriver)
        
        fuzzyDriver = FuzzyWidget( self._mainWindowUi.driverWidget )
        self._create_driver_widget("FUZZY", fuzzyDriver)
    
    def _create_driver_widget(self, driver_type, driver):
        rospy.loginfo("registering driver: %r", driver_type )
        self._mainWindowUi.driverWidget.addWidget( driver )
        self._mainWindowUi.driverCB.addItem( driver_type )

