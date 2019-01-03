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


class PIDObject():
    
    def __init__(self, pidname):
        self.pid = None
        self.reset_state()
        
        rospy.Subscriber("/self_balancer/" + pidname + "/setpoint", Float64, self._setpoint_callback)
        rospy.Subscriber("/self_balancer/" + pidname + "/kp", Float64, self._kp_callback)
        rospy.Subscriber("/self_balancer/" + pidname + "/ki", Float64, self._ki_callback)
        rospy.Subscriber("/self_balancer/" + pidname + "/kd", Float64, self._kd_callback)
        self.error_pub = rospy.Publisher("/self_balancer/" + pidname + "/error", Float64, queue_size=10)
            
    def steer(self, inputValue):
        if abs(inputValue) > 50.0:
            ## no chances to keep standing -- stop wheels
            return None
#         dinput = pitch - self.pid._last_input
        val = self.pid( inputValue )
#         rospy.loginfo("pid: %r -> %r di:%r es:%r p:%r", pitch, val, dinput, self.pid._error_sum, self.pid._proportional )
        self.error_pub.publish( self.pid._error_sum + self.pid._proportional )      ## when proportional_on_measurement=False
        return val
    
    def reset_state(self):
        ## nice params p:1.5 d:08 i:0.2 on voltage control
        newPid = PID(0.0, 0.0, 0.0, sample_time=None, proportional_on_measurement=False)
        newPid.output_limits=(-10.0, 10.0)
        newPid._last_input = 0.0
        if self.pid is not None:
            newPid.tunings = self.pid.tunings
            newPid.setpoint = self.pid.setpoint
        self.pid = newPid
    
    def _setpoint_callback(self, value):
        rospy.loginfo("setting angle: %r", value )
        self.pid.setpoint = value.data
        
    def _kp_callback(self, value):
        rospy.loginfo("setting Kp: %r", value )
        self.pid.Kp = value.data

    def _ki_callback(self, value):
        rospy.loginfo("setting Ki: %r", value )
        self.pid.Ki = value.data

    def _kd_callback(self, value):
        rospy.loginfo("setting Kd: %r", value )
        self.pid.Kd = value.data
    

class PIDDriver(CartDriver):

    def __init__(self):
        CartDriver.__init__(self)
        self.pitchpid = PIDObject("pitch_pid")
        #self.speedpid = PIDObject("speed_pid")

    def reset_state(self):
        rospy.loginfo("resetting PID" )
        ## nice params p:1.5 d:08 i:0.2 on voltage control
        self.pitchpid.reset_state()
        #self.speedpid.reset_state()
        
    def steer(self, pitch):
        if pitch is None:
            return (0.0, 0.0)
        if abs(pitch) > 50.0:
            ## no chances to keep standing -- stop wheels
            return None
        pitchValue = self.pitchpid.steer( pitch )
        return (pitchValue, pitchValue)
