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


import sys
import os

#### append local library
script_dir = os.path.dirname(__file__)
sys.path.append(os.path.abspath( os.path.join(script_dir, "../..") ))

import numpy as np
# Plot the result in pretty 3D with alpha blending
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Required for 3D plotting

from balancer.driver.fuzzy_controller import FuzzyObject


if __name__ == '__main__':
    fuzzy = FuzzyObject()

    fuzzy.ant_pitch.view()
    fuzzy.ant_speed.view()
    fuzzy.con_voltage.view()

#     rule1.view()
    fuzzy.con_voltage.view(sim=fuzzy.sim)

    # We can simulate at higher resolution with full accuracy
    upsampledx = np.linspace(-90, 90, 21)
    upsampledy = np.linspace(-30, 30, 21)
    x, y = np.meshgrid(upsampledx, upsampledy)
    z = np.zeros_like(x)

    # Loop through the system 21*21 times to collect the control surface
    for i in range(21):
        for j in range(21):
            out = fuzzy.compute(x[i, j], y[i, j])
            z[i, j] = out

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis',
                           linewidth=0.4, antialiased=True)

    cset = ax.contourf(x, y, z, zdir='z', offset=-2.5, cmap='viridis', alpha=0.5)
    cset = ax.contourf(x, y, z, zdir='x', offset=3, cmap='viridis', alpha=0.5)
    cset = ax.contourf(x, y, z, zdir='y', offset=3, cmap='viridis', alpha=0.5)

    ax.set_xlabel("pitch")
    ax.set_ylabel("speed")
    ax.set_zlabel("voltage")

    ax.view_init(30, 200)

    plt.show()

#     input("Press Enter to continue...")
