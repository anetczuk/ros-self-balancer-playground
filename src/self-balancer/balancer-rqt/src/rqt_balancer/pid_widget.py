import os
import rospy
import rospkg
from std_msgs.msg import Float64

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class PidWidget(QWidget):

    def __init__(self, parent, pidName):
        super(PidWidget, self).__init__(parent)
        # Create QWidget
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_balancer'), 'resource',  self.__class__.__name__ + '.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        # Give QObjects reasonable names

        self.pid_name = pidName
        self.widgetName.setText( pidName )

        self.setpoint_pub = rospy.Publisher('/self_balancer/' + pidName + '/setpoint', Float64, queue_size=10, latch=True)
        self.kp_pub = rospy.Publisher('/self_balancer/' + pidName + '/kp', Float64, queue_size=10, latch=True)
        self.ki_pub = rospy.Publisher('/self_balancer/' + pidName + '/ki', Float64, queue_size=10, latch=True)
        self.kd_pub = rospy.Publisher('/self_balancer/' + pidName + '/kd', Float64, queue_size=10, latch=True)

        self.setpointSB.valueChanged.connect( self._setpointSB_changed )
        self.kpSB.valueChanged.connect( self._kpSB_changed )
        self.kiSB.valueChanged.connect( self._kiSB_changed )
        self.kdSB.valueChanged.connect( self._kdSB_changed )

    def _setpointSB_changed(self, value):
        print( self.pid_name, ' setpoint changed: ', value)
        self.setpoint_pub.publish(value)

    def _kpSB_changed(self, value):
        print( self.pid_name, ' kp changed: ', value)
        self.kp_pub.publish(value)

    def _kiSB_changed(self, value):
        print( self.pid_name, ' ki changed: ', value)
        self.ki_pub.publish(value)

    def _kdSB_changed(self, value):
        print( self.pid_name, ' kd changed: ', value)
        self.kd_pub.publish(value)

    def save_settings(self, plugin_settings):
        settings = plugin_settings.get_settings(self.pid_name)
        settings.set_value("setpoint", self.setpointSB.value() )
        settings.set_value("kp", self.kpSB.value() )
        settings.set_value("ki", self.kiSB.value() )
        settings.set_value("kd", self.kdSB.value() )

    def restore_settings(self, plugin_settings):
        settings = plugin_settings.get_settings(self.pid_name)
        setpoint = settings.value("setpoint", 0)
        self.setpointSB.setValue( float(setpoint) )
        kp = settings.value("kp", 0)
        self.kpSB.setValue( float(kp) )
        ki = settings.value("ki", 0)
        self.kiSB.setValue( float(ki) )
        kd = settings.value("kd", 0)
        self.kdSB.setValue( float(kd) )
