#!/usr/bin/env python3
#
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


from simple_pid.PID import PID
import rospy
from std_msgs.msg import Float64

from .cart_driver import CartDriver


class PIDDriver(CartDriver):

    def __init__(self):
        CartDriver.__init__(self)
        self.pid = None
        
        self.reset_state()

        rospy.Subscriber("/self_balancer/pid/kp", Float64, self._kp_callback)
        rospy.Subscriber("/self_balancer/pid/ki", Float64, self._ki_callback)
        rospy.Subscriber("/self_balancer/pid/kd", Float64, self._kd_callback)

    def reset_state(self):
        newPid = PID(0.0, 0.0, 0.0, sample_time=None)
        if self.pid is not None:
            params = self.pid.tunings
            newPid.tunings = params
        self.pid = newPid
        
    def steer(self, pitch):
        if pitch is None:
            return (0.0, 0.0)
        if abs(pitch) > 50.0:
            ## no chances to keep standing -- stop wheels
            return None
        val = self.pid( pitch )
        return (val, val)

    def _kp_callback(self, value):
        rospy.loginfo("setting Kp: %r", value )
        self.pid.Kp = value.data

    def _ki_callback(self, value):
        rospy.loginfo("setting Ki: %r", value )
        self.pid.Ki = value.data

    def _kd_callback(self, value):
        rospy.loginfo("setting Kd: %r", value )
        self.pid.Kd = value.data
