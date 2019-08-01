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


import rospy

from ..cart_controller import CartController
from .pid_object import PIDObject


class PIDCascadeController(CartController):

    def __init__(self):
        CartController.__init__(self)
        self.speedpid = PIDObject("cascade_pid/speed", 30.0)
        self.speedpid.set_params( -0.5, -0.3, 1.6 )
        self.pitchpid = PIDObject("cascade_pid/pitch", 10.0)
        self.pitchpid.set_params( 0.5, 0.6, 1.6 )

    def reset_state(self):
        rospy.loginfo("resetting PID" )
        self.pitchpid.reset_state()
        self.speedpid.reset_state()

    def steer(self, cart):
        pitchInput = cart.pitch
        speedInput = cart.wheel_speed
        if speedInput is None:
            return (0.0, 0.0)

        speedValue = self.speedpid.calc( speedInput )
        pitchValue = pitchInput - speedValue
        outputValue = self.pitchpid.calc( pitchValue )

        err_speed = self.speedpid.error_sum()
        err_pitch = self.pitchpid.error_sum()
        rospy.loginfo("pid: %+.8f %+.8f -> %+.8f -> %+.8f e: %+.8f %+.8f", pitchInput, speedInput, speedValue, outputValue, err_speed, err_pitch)
        return (outputValue, outputValue)

    def terminate(self):
        ## do nothing
        pass
