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

from balancer.driver.fuzzy_object import Fuzzy


class FuzzyTest(unittest.TestCase):
    def setUp(self):
        self.fuzzy = Fuzzy()

    def tearDown(self):
        self.fuzzy = None

    def test_PP(self):
        output = self.fuzzy.compute( 10.0 )
        self.assertLess( output, 0.0 )
    
    def test_NN(self):
        output = self.fuzzy.compute( -10.0 )
        self.assertLess( output, 0.0 )

    def test_set_err_range_zero(self):
        self.fuzzy.set_err_range(0.0)
        self.fuzzy.build()
        output = self.fuzzy.compute( 10.0 )
        self.assertLess( output, 0.0 )

    def test_set_err_range_negative(self):
        self.fuzzy.set_err_range(-10.0)
        self.fuzzy.build()
        output = self.fuzzy.compute( 10.0 )
        self.assertGreater( output, 0.0 )

    def test_set_derr_range_zero(self):
        self.fuzzy.set_derr_range(0.0)
        self.fuzzy.build()
        output = self.fuzzy.compute( 10.0 )
        self.assertLess( output, 0.0 )

    def test_set_derr_range_negative(self):
        self.fuzzy.set_derr_range(-10.0)
        self.fuzzy.build()
        output = self.fuzzy.compute( 10.0 )
        self.assertLess( output, 0.0 )

    def test_set_output_range_zero(self):
        self.fuzzy.set_output_range(0.0)
        self.fuzzy.build()
        output = self.fuzzy.compute( 10.0 )
        self.assertAlmostEqual( output, 0.0, delta=0.0000001 )

    def test_set_output_range_negative(self):
        self.fuzzy.set_output_range( -10.0)
        self.fuzzy.build()
        output = self.fuzzy.compute( 10.0 )
        self.assertLess( output, 0.0 )

