# MIT License
#
# Copyright (c) 2017 Arkadiusz Netczuk <dev.arnet@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import time
import rospy
from std_msgs.msg import Float64

import numpy as np
from skfuzzy import control as ctrl
from skfuzzy.control.visualization import FuzzyVariableVisualizer, ControlSystemVisualizer
from enum import Enum, unique
# from enum import EnumMeta
import matplotlib.pyplot

from ..synchronized import synchronized
# from apt_offline_core.AptOfflineMagicLib import NONE


class Error():

    def __init__(self):
        self.last_input = None
        self.value = None

    def calculate(self, inputValue):
        err = -inputValue
        dt_err = 0.0
        if self.last_input is not None:
            dt_err = self.last_input - inputValue
        self.last_input = inputValue
        self.value = (err, dt_err)
        return self.value


class PlotsDict():

    def __init__(self):
        self._variable_plots = {}
        self.request_list = []

    def request_plot(self, fuzzyObj, window_title):
        self.request_list.append( (fuzzyObj, window_title) )

    def process_requests(self):
        if len(self.request_list) < 1:
            return
        for fuzz, title in self.request_list:
            self.plot( fuzz, title )
        self.request_list.clear()

    def plot(self, fuzzyObj, window_title):
        if fuzzyObj is None:
            return
        if window_title is None:
            return

        vizualizer = self._create_visualizer( window_title, fuzzyObj )
        fig = vizualizer.fig

        #self.close( window_title )
        #fig = self._plot_fuzzy(fuzzyObj)

        if fig is None:
            return
        fig.canvas.set_window_title( window_title )
        self._variable_plots[ window_title ] = vizualizer

    def close(self, figKey):
        if figKey is None:
            return
        vizualizer = self._variable_plots.get( figKey, None )
        if vizualizer is None:
            return
        fig = vizualizer.fig
        matplotlib.pyplot.close(fig)

    def close_all(self):
        for items in self._variable_plots.items():
            self.close(items[0])

    def _plot_fuzzy(self, fuzzyObj):
        if isinstance(fuzzyObj, ctrl.fuzzyvariable.FuzzyVariable):
            fig, ax = FuzzyVariableVisualizer(fuzzyObj).view()
            fig.show()
            return fig
        if isinstance(fuzzyObj, ctrl.ControlSystem):
            fig, ax = ControlSystemVisualizer(fuzzyObj).view()
            fig.show()
            return fig
        return None

    def _create_visualizer(self, window_title, fuzzyObj):
        visualiser = self._variable_plots.get( window_title, None )
#             visualiser.fuzzy_var = fuzzyObj
#             fig, ax = visualiser.view()
#             fig.show()
#             return visualiser
        if isinstance(fuzzyObj, ctrl.fuzzyvariable.FuzzyVariable):
            if visualiser is None:
                visualiser = FuzzyVariableVisualizer(fuzzyObj)
                visualiser.view()
                visualiser.fig.show()
            else:
                visualiser.ax.clear()
                visualiser.fuzzy_var = fuzzyObj
                visualiser.view()
                visualiser.fig.canvas.draw_idle()
            return visualiser
        if isinstance(fuzzyObj, ctrl.ControlSystem):
            if visualiser is None:
                visualiser = ControlSystemVisualizer(fuzzyObj)
                visualiser.view()
                visualiser.fig.show()
            else:
                visualiser.ax.clear()
                visualiser.ctrl = fuzzyObj
                visualiser.view()
                visualiser.fig.canvas.draw_idle()
            return visualiser
        return None


@unique
class FuzzyType(Enum):
    INVALID = "INVALID"
    BOTH    = "BOTH"
    ERR     = "ERR"
    DERR    = "DERR"


class Fuzzy():

    def __init__(self, err_param=90, derr_param=30, output_param=20):
        self._plots = PlotsDict()
        self.error = Error()
        self.set_err_range( err_param )
        self.set_derr_range( derr_param )
        self.set_output_range( output_param )
        self.build()

    @synchronized
    def is_enabled(self):
        if self.type() == FuzzyType.INVALID:
            return False
        return True

    @synchronized
    def type(self):
        if self.con_output is None:
            return FuzzyType.INVALID

        if self.ant_err is not None:
            if self.ant_derr is not None:
                return FuzzyType.BOTH
            else:
                return FuzzyType.ERR
        else:
            if self.ant_derr is not None:
                return FuzzyType.DERR
            else:
                return FuzzyType.INVALID

    @synchronized
    def build(self):
        fuzzy_type = self.type()

        if fuzzy_type == FuzzyType.INVALID:
            rospy.loginfo("fuzzy disabled")
            self.sim = None
            return

        if fuzzy_type == FuzzyType.BOTH:
            rules = []
            rules.append( self._create_rule("N", "N", "N") )
            rules.append( self._create_rule("N", "Z", "N") )
            rules.append( self._create_rule("N", "P", "Z") )

            rules.append( self._create_rule("Z", "N", "N") )
            rules.append( self._create_rule("Z", "Z", "Z") )
            rules.append( self._create_rule("Z", "P", "P") )

            rules.append( self._create_rule("P", "N", "Z") )
            rules.append( self._create_rule("P", "Z", "P") )
            rules.append( self._create_rule("P", "P", "P") )

#             rules.append( self._create_rule("NB", "NB", "NB") )
#             rules.append( self._create_rule("NB", "NS", "NB") )
#             rules.append( self._create_rule("NB",  "Z", "NS") )
#             rules.append( self._create_rule("NB", "PS", "NS") )
#             rules.append( self._create_rule("NB", "PB",  "Z") )
#
#             rules.append( self._create_rule("NS", "NB", "NB") )
#             rules.append( self._create_rule("NS", "NS", "NS") )
#             rules.append( self._create_rule("NS",  "Z", "NS") )
#             rules.append( self._create_rule("NS", "PS",  "Z") )
#             rules.append( self._create_rule("NS", "PB", "PS") )
#
#             rules.append( self._create_rule("Z", "NB", "NS") )
#             rules.append( self._create_rule("Z", "NS", "NS") )
#             rules.append( self._create_rule("Z",  "Z",  "Z") )
#             rules.append( self._create_rule("Z", "PS", "PS") )
#             rules.append( self._create_rule("Z", "PB", "PS") )
#
#             rules.append( self._create_rule("PS", "NB", "NS") )
#             rules.append( self._create_rule("PS", "NS",  "Z") )
#             rules.append( self._create_rule("PS",  "Z", "PS") )
#             rules.append( self._create_rule("PS", "PS", "PS") )
#             rules.append( self._create_rule("PS", "PB", "PB") )
#
#             rules.append( self._create_rule("PB", "NB",  "Z") )
#             rules.append( self._create_rule("PB", "NS", "PS") )
#             rules.append( self._create_rule("PB",  "Z", "PS") )
#             rules.append( self._create_rule("PB", "PS", "PB") )
#             rules.append( self._create_rule("PB", "PB", "PB") )

            self.output_ctrl = ctrl.ControlSystem( rules )
            self.sim = ctrl.ControlSystemSimulation(self.output_ctrl)
#             self._plots.request_plot( self.output_ctrl, "rules" )
            return

        if fuzzy_type == FuzzyType.ERR:
            rules = []
            rules.append( ctrl.Rule(self.ant_err["N"], self.con_output["N"]) )
            rules.append( ctrl.Rule(self.ant_err["Z"], self.con_output["Z"]) )
            rules.append( ctrl.Rule(self.ant_err["P"], self.con_output["P"]) )
#             rules.append( ctrl.Rule(self.ant_err["NB"], self.con_output["NB"]) )
#             rules.append( ctrl.Rule(self.ant_err["NS"], self.con_output["NS"]) )
#             rules.append( ctrl.Rule(self.ant_err[ "Z"], self.con_output[ "Z"]) )
#             rules.append( ctrl.Rule(self.ant_err["PS"], self.con_output["PS"]) )
#             rules.append( ctrl.Rule(self.ant_err["PB"], self.con_output["PB"]) )

            self.output_ctrl = ctrl.ControlSystem( rules )
            self.sim = ctrl.ControlSystemSimulation(self.output_ctrl)
#             self._plots.request_plot( self.output_ctrl, "rules" )
            return

        if fuzzy_type == FuzzyType.DERR:
            rules = []
            rules.append( ctrl.Rule(self.ant_derr["N"], self.con_output["N"]) )
            rules.append( ctrl.Rule(self.ant_derr["Z"], self.con_output["Z"]) )
            rules.append( ctrl.Rule(self.ant_derr["P"], self.con_output["P"]) )
#             rules.append( ctrl.Rule(self.ant_derr["NB"], self.con_output["NB"]) )
#             rules.append( ctrl.Rule(self.ant_derr["NS"], self.con_output["NS"]) )
#             rules.append( ctrl.Rule(self.ant_derr[ "Z"], self.con_output[ "Z"]) )
#             rules.append( ctrl.Rule(self.ant_derr["PS"], self.con_output["PS"]) )
#             rules.append( ctrl.Rule(self.ant_derr["PB"], self.con_output["PB"]) )

            self.output_ctrl = ctrl.ControlSystem( rules )
            self.sim = ctrl.ControlSystemSimulation(self.output_ctrl)
#             self._plots.request_plot( self.output_ctrl, "rules" )
            return

        self.sim = None
        raise ValueError('Invalid state: ' + fuzzy_type)

    def reset_state(self):
        self.error = Error()

    @synchronized
    def set_err_range(self, value):
        absval = abs(value)
        if absval < 0.1:
            self.ant_err = None
            return
        invert = False
        if value < 0.0:
            invert = True
        self.ant_err = ctrl.Antecedent(np.arange(-absval, absval, 1), 'err')
        self.ant_err.automf(names=["N", "Z", "P"], invert=invert)
        ##self.ant_err.automf(names=["NB", "NS", "Z", "PS", "PB"], invert=invert)
        self._plots.request_plot( self.ant_err, "err" )

    @synchronized
    def set_derr_range(self, value):
        absval = abs(value)
        if absval < 0.1:
            self.ant_derr = None
            return
        invert = False
        if value < 0.0:
            invert = True
        self.ant_derr = ctrl.Antecedent(np.arange(-absval, absval, 1), 'derr')
        self.ant_derr.automf(names=["N", "Z", "P"], invert=invert)
        ##self.ant_derr.automf(names=["NB", "NS", "Z", "PS", "PB"], invert=invert)
        self._plots.request_plot( self.ant_derr, "derr" )

    @synchronized
    def set_output_range(self, value):
        absval = abs(value)
        if absval < 0.1:
            self.con_output = None
            return
        invert = False
        if value < 0.0:
            invert = True
        self.con_output = ctrl.Consequent(np.arange(-absval, absval, 1), 'output')
        self.con_output.automf(names=["N", "Z", "P"], invert=invert)
        ##self.con_output.automf(names=["NB", "NS", "Z", "PS", "PB"], invert=invert)
        self._plots.request_plot( self.con_output, "output" )

    @synchronized
    def compute(self, valueInput):
        fuzzy_type = self.type()

        if fuzzy_type == FuzzyType.INVALID:
            self.error.calculate(valueInput)
            return 0

        err = self.error.calculate(valueInput)

        if fuzzy_type == FuzzyType.BOTH or fuzzy_type == FuzzyType.ERR:
            self.sim.input['err'] = err[0]

        if fuzzy_type == FuzzyType.BOTH or fuzzy_type == FuzzyType.DERR:
            self.sim.input['derr'] = err[1]

        self.sim.compute()

        self._plots.process_requests()

        output = self.sim.output['output']
        return output

    def close(self):
        self._plots.close_all()

    def _create_rule(self, err, derr, out):
        rule = ctrl.Rule(self.ant_err[err] | self.ant_derr[derr], self.con_output[out])
        return rule


class FuzzyObject():

    def __init__(self, controller_name, err_param=90, derr_param=30, output_param=20):
        self.controller_name = controller_name
        self.fuzzy = Fuzzy(err_param, derr_param, output_param)

        rospy.Subscriber("/self_balancer/" + self.controller_name + "/err_max", Float64, self._err_callback)
        rospy.Subscriber("/self_balancer/" + self.controller_name + "/derr_max", Float64, self._derr_callback)
        rospy.Subscriber("/self_balancer/" + self.controller_name + "/output_max", Float64, self._output_callback)
        self.output_pub = rospy.Publisher("/self_balancer/" + self.controller_name + "/output", Float64, queue_size=10)
        self.err_pub = rospy.Publisher("/self_balancer/" + self.controller_name + "/err", Float64, queue_size=10)
        self.derr_pub = rospy.Publisher("/self_balancer/" + self.controller_name + "/derr", Float64, queue_size=10)

    def reset_state(self):
        rospy.loginfo("resetting Fuzzy")
        self.fuzzy.reset_state()

    def calc(self, inputValue):
        output = self.fuzzy.compute(inputValue)
        self.output_pub.publish( output )
        err = self.fuzzy.error.value
        if err is not None:
            self.err_pub.publish( err[0] )
            self.derr_pub.publish( err[1] )
        return output

    def close(self):
        self.fuzzy.close()

    def _err_callback(self, value):
        rospy.loginfo("%s setting err max: %r", self.controller_name, value )
        self.fuzzy.set_err_range(value.data)
        self.fuzzy.build()

    def _derr_callback(self, value):
        rospy.loginfo("%s setting derr max: %r", self.controller_name, value )
        self.fuzzy.set_derr_range(value.data)
        self.fuzzy.build()

    def _output_callback(self, value):
        rospy.loginfo("%s setting output max: %r", self.controller_name, value )
        self.fuzzy.set_output_range(value.data)
        self.fuzzy.build()

