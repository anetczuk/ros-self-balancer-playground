import os
import rospy
import rospkg
from std_msgs.msg import String

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from .pid_single_widget import PidSingleWidget
from .pid_cascade_widget import PidCascadeWidget
from .fuzzy_double_widget import FuzzyDoubleWidget


class MainWindow(Plugin):

    def __init__(self, context):
        super(MainWindow, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MainWindow')

        self.controller_type_pub = rospy.Publisher('/self_balancer/controller_type', String, queue_size=10, latch=True)

        self._parse_cmd_args(context)
        self._init_widget(context)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        widgetsCount = self._mainWindowUi.controllerWidget.count()
        for i in range(0, widgetsCount):
            controller = self._mainWindowUi.controllerWidget.widget(i)
            controller.save_settings(plugin_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        widgetsCount = self._mainWindowUi.controllerWidget.count()
        for i in range(0, widgetsCount):
            controller = self._mainWindowUi.controllerWidget.widget(i)
            controller.restore_settings(plugin_settings)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def _parse_cmd_args(self, context):
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument( "-q", "--quiet", action="store_true",
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

        self._mainWindowUi.controllerCB.currentIndexChanged.connect( self._controllerChanged )

        self._init_controllers()

        # Add widget to the user interface
        context.add_widget(self._mainWindowUi)

    def _controllerChanged(self):
        controller_type = self._mainWindowUi.controllerCB.currentText()
        rospy.loginfo("selecting controller: %r", controller_type )
        index = self._mainWindowUi.controllerCB.currentIndex()
        self._mainWindowUi.controllerWidget.setCurrentIndex( index )
        self.controller_type_pub.publish( str(controller_type) )

    def _init_controllers(self):
        singlePidController = PidSingleWidget( self._mainWindowUi.controllerWidget )
        self._create_controller_widget("PID_SINGLE", singlePidController)

        cascadePidController = PidCascadeWidget( self._mainWindowUi.controllerWidget )
        self._create_controller_widget("PID_CASCADE", cascadePidController)

        doubleFuzzyController = FuzzyDoubleWidget( self._mainWindowUi.controllerWidget )
        self._create_controller_widget("FUZZY_DOUBLE", doubleFuzzyController)

    def _create_controller_widget(self, controller_type, controller):
        rospy.loginfo("registering controller: %r", controller_type )
        self._mainWindowUi.controllerWidget.addWidget( controller )
        self._mainWindowUi.controllerCB.addItem( controller_type )

