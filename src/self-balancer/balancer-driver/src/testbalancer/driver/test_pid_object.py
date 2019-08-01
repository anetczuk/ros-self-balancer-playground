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


import unittest

from balancer.driver.pid_object import PIDClassic as PID
from simple_pid.PID import _current_time


class PIDObjectTest(unittest.TestCase):
    def setUp(self):
        self.pid = PID()
        self.pid.reset_state()
        self.pid.calc( 0.0 )             ## first call -- initialize

    def tearDown(self):
        self.pid = None

    def test_setpoint_poportinal(self):
        self.pid.set_params(1.0, 0.0, 0.0)
        self.pid.set_target(1.0)
        output = self.pid.calc( 2.0 )
        self.assertEqual( -1.0, output )

    def test_setpoint_integral(self):
        self.pid.set_params(0.0, 1.0, 0.0)
        self.pid.set_target(1.0)
        self.pid.set_last_time( _current_time() - 1 )      ## mimic time pass
        output = self.pid.calc( 2.0 )
        self.assertAlmostEqual( -1.0, output, delta=0.0001 )

    def test_setpoint_derivative(self):
        self.pid.set_params(0.0, 0.0, 1.0)
        self.pid.set_target(1.0)
        output = self.pid.calc( 2.0 )
        self.assertEqual( -1.0, output )


