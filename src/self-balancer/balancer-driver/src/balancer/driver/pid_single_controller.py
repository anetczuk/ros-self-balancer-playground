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


class PIDSingleController(CartController):

    def __init__(self):
        CartController.__init__(self)
        self.pitchpid = PIDObject("single_pid/pitch", 10.0)
        self.pitchpid.set_params( 0.6, 0.1, 1.6 )

    def reset_state(self):
        rospy.loginfo("resetting PID" )
        self.pitchpid.reset_state()

    def steer(self, cart):
        pitch = cart.pitch
        pitchValue = self.pitchpid.calc( pitch )

        rospy.loginfo("pid: %+.8f -> %+.8f", pitch, pitchValue)
        return (pitchValue, pitchValue)

    def terminate(self):
        ## do nothing
        pass
