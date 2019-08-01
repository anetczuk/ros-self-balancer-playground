import os
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from .pid_widget import PidWidget


class PidCascadeWidget(QWidget):

    def __init__(self, parent):
        super(PidCascadeWidget, self).__init__(parent)
        # Create QWidget
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_balancer'), 'resource',  self.__class__.__name__ + '.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.pitchPidWidget = PidWidget( self.pitchWidget, "cascade_pid/pitch" )
        self.speedPidWidget = PidWidget( self.speedWidget, "cascade_pid/speed" )

    def save_settings(self, plugin_settings):
        self.pitchPidWidget.save_settings(plugin_settings)
        self.speedPidWidget.save_settings(plugin_settings)

    def restore_settings(self, plugin_settings):
        self.pitchPidWidget.restore_settings(plugin_settings)
        self.speedPidWidget.restore_settings(plugin_settings)
