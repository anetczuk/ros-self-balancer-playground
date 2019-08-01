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
from .fuzzy_object import FuzzyObject


class FuzzyController(CartController):

    def __init__(self):
        CartController.__init__(self)
        self.pitchfuzzy = FuzzyObject("double_fuzzy/pitch", 10.0, 10.0, 80.0 )
        self.speedfuzzy = FuzzyObject("double_fuzzy/speed", 50.0, 100.0, 50.0 )

    def reset_state(self):
        rospy.loginfo("resetting Fuzzy" )
        self.pitchfuzzy.reset_state()
        self.speedfuzzy.reset_state()

    def steer(self, cart):
        pitchInput = cart.pitch
        speedInput = cart.wheel_speed
        if speedInput is None:
            return (0.0, 0.0)

        pitchOutput = self.pitchfuzzy.calc(pitchInput)
        speedOutput = self.speedfuzzy.calc(speedInput)
        output = pitchOutput + speedOutput

        rospy.loginfo("fuzzy: %+.8f %+.8f -> %+.8f", pitchInput, speedInput, output)
        return (output, output)

    def terminate(self):
        ## do nothing
        self.pitchfuzzy.close()
        self.speedfuzzy.close()
