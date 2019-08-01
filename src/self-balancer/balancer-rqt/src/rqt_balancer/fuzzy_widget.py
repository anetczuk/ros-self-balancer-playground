import os
import rospy
import rospkg
from std_msgs.msg import Float64

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class FuzzyWidget(QWidget):

    def __init__(self, parent, fuzzyName):
        super(FuzzyWidget, self).__init__(parent)
        # Create QWidget
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_balancer'), 'resource',  self.__class__.__name__ + '.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        # Give QObjects reasonable names

        self.controller_name = fuzzyName
        self.widgetName.setText( self.controller_name )

        self.pitch_max_pub = rospy.Publisher('/self_balancer/' + self.controller_name + '/err_max', Float64, queue_size=10, latch=True)
        self.speed_max_pub = rospy.Publisher('/self_balancer/' + self.controller_name + '/derr_max', Float64, queue_size=10, latch=True)
        self.voltage_max_pub = rospy.Publisher('/self_balancer/' + self.controller_name + '/output_max', Float64, queue_size=10, latch=True)

        self.prevErrVal = None
        self.prevDerrVal = None

        self.errSB.valueChanged.connect( self._errSB_changed )
        self.derrSB.valueChanged.connect( self._derrSB_changed )
        self.errEnabledCB.stateChanged.connect( self._err_enable_changed )
        self.derrEnabledCB.stateChanged.connect( self._derr_enable_changed )
        self.outputSB.valueChanged.connect( self._outputSB_changed )

    def _errSB_changed(self, value):
        print( self.controller_name, ' err changed: ', value)
        self.pitch_max_pub.publish(value)

    def _derrSB_changed(self, value):
        print( self.controller_name, ' derr changed: ', value)
        self.speed_max_pub.publish(value)

    def _outputSB_changed(self, value):
        print( self.controller_name, ' output changed: ', value)
        self.voltage_max_pub.publish(value)

    def _err_enable_changed(self, value):
        if value == 0:
            self.prevErrVal = self.errSB.value()
            self.errSB.setValue( 0 )
            self.errSB.setEnabled( False )
        else:
            if self.prevErrVal is not None:
                self.errSB.setValue( self.prevErrVal )
            self.errSB.setEnabled( True )

    def _derr_enable_changed(self, value):
        if value == 0:
            self.prevDerrVal = self.derrSB.value()
            self.derrSB.setValue( 0 )
            self.derrSB.setEnabled( False )
        else:
            if self.prevDerrVal is not None:
                self.derrSB.setValue( self.prevDerrVal )
            self.derrSB.setEnabled( True )

    def save_settings(self, plugin_settings):
        settings = plugin_settings.get_settings(self.controller_name)
        settings.set_value("err", self.errSB.value() )
        settings.set_value("derr", self.derrSB.value() )
        settings.set_value("output", self.outputSB.value() )

    def restore_settings(self, plugin_settings):
        settings = plugin_settings.get_settings(self.controller_name)
        err = settings.value("err", 0)
        self.errSB.setValue( float(err) )
        derr = settings.value("derr", 0)
        self.derrSB.setValue( float(derr) )
        output = settings.value("output", 0)
        self.outputSB.setValue( float(output) )
