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


import math
import numpy as np
import quaternion

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty

try:
    import matplotlib

    # Make sure that we are using QT5
    matplotlib.use('Qt5Agg')
    from PyQt5.QtWidgets import qApp
    
    import matplotlib.pyplot as plt
    ##print("all backends:", plt.rcsetup.all_backends )
    ##print("current backend:", plt.get_backend() )
    
except ImportError:
    ### No module named <name>
    #logging.exception("Exception while importing")
    print("Exception while importing")
    exit(1)

from .driver.pid_single_controller import PIDSingleController
from .driver.pid_cascade_controller import PIDCascadeController
from .driver.fuzzy_controller import FuzzyController


def rotationMatrixToEulerAngles(R):
    """
    Calculate rotation matrix to euler angles.

    The result is the same as MATLAB except the order
    of the euler angles ( x and z are swapped ).
    """
    sy = math.sqrt( R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0] )
    singular = sy < 1e-6
    if not singular:
        x = math.atan2( R[2, 1], R[2, 2] )
        y = math.atan2( -R[2, 0], sy )
        z = math.atan2( R[1, 0], R[0, 0] )
    else:
        x = math.atan2( -R[1, 2], R[1, 1] )
        y = math.atan2( -R[2, 0], sy )
        z = 0
    return np.array([x, y, z])


class Cart:

    def __init__(self):
        self.qorientation = None    ## quaternion
        self.euler_angles = None    ## in radians
        self.pitch = None           ## in degrees
        self.wheel_speed = None
        self.controller = None
        
        self._create_controller("PID_SINGLE")
        
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)

    def run(self):
        # [INFO] [1546208845.310780, 1800.357000]: /pic_controller_32685_1546208844652Imu received:
        # ['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__getstate__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__setstate__', '__sizeof__', '__slots__', '__str__', '__subclasshook__',
        # '_check_types', '_connection_header', '_full_text', '_get_types', '_has_header', '_md5sum', '_slot_types', '_type',
        # 'angular_velocity', 'angular_velocity_covariance', 'deserialize', 'deserialize_numpy', 'header', 'linear_acceleration', 'linear_acceleration_covariance', 'orientation', 'orientation_covariance', 'serialize', 'serialize_numpy']
        rospy.Subscriber("/teeterbot/fallen_over", Bool, self._cart_fallen)
        rospy.Subscriber("/teeterbot/imu", Imu, self._imu_callback)
        rospy.Subscriber("/teeterbot/right_wheel_speed", Float64, self._wheel_callback)

        rospy.Subscriber("/self_balancer/controller_type", String, self._controller_type_callback)
        pitch_pub = rospy.Publisher('/self_balancer/pitch', Float64, queue_size=10)
        output_pub = rospy.Publisher('/self_balancer/output', Float64, queue_size=10)
        
#         left_pub = rospy.Publisher('/teeterbot/left_torque_cmd', Float64, queue_size=10)
#         right_pub = rospy.Publisher('/teeterbot/right_torque_cmd', Float64, queue_size=10)
        left_pub = rospy.Publisher('/teeterbot/left_motor_voltage', Float64, queue_size=10)
        right_pub = rospy.Publisher('/teeterbot/right_motor_voltage', Float64, queue_size=10)

        r = rospy.Rate(30)                  # 10hz

        while not rospy.is_shutdown():
            ##str = "hello world %s"%rospy.get_time()
            #val = random.uniform(-1.0, 1.0)
            
            pitch_pub.publish( self.pitch )

            output = self.drive()
            if output is not None:
                left_out = output[0]
                right_out = output[1]
                rospy.loginfo("output: %+.8f -> %+.8f %+.8f", self.pitch, left_out, right_out)
                left_pub.publish(left_out)
                right_pub.publish(right_out)
                output_pub.publish(right_out)
            else:
                left_pub.publish(0.0)
                right_pub.publish(0.0)
                output_pub.publish(0.0)
            
            ## redraw plot windows
            qApp.processEvents()
            
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as e:
                ## happens when world is resetted
                rospy.loginfo("exception: %r", e )
                self._reset_controller()

    def drive(self):
        if self.pitch is None:
#             rospy.loginfo("pid state: %r (%s)", pitchInput, self.pitchpid.state() )
            return None
        if abs(self.pitch) > 70.0:
            ## no chances to keep standing -- stop wheels
            return (0.0, 0.0)
        output = self.controller.steer( self )
        if output is None:
            return (0.0, 0.0)
        return output

    def _controller_type_callback(self, controller_type):
        controller_data = controller_type.data
        self._create_controller( controller_data )
        
    def _create_controller(self, controller_type):
        rospy.loginfo("got controller: %s", controller_type)
        if controller_type == "PID_SINGLE":
            if self.controller is not None:
                self.controller.terminate()
            rospy.loginfo("setting single PID controller" )
            self.controller = PIDSingleController()
            return
        if controller_type == "PID_CASCADE":
            if self.controller is not None:
                self.controller.terminate()
            rospy.loginfo("setting cascade PID controller" )
            self.controller = PIDCascadeController()
            return
        if controller_type == "FUZZY_DOUBLE":
            if self.controller is not None:
                self.controller.terminate()
            rospy.loginfo("setting double Fuzzy controller" )
            self.controller = FuzzyController()
            return
        rospy.loginfo("unknown controller type: %s", controller_type)
        
    def _cart_fallen(self, value):
        if value.data is False:
            ## cart stand up
            rospy.loginfo("cart stand up - resetting simulation" )
            self.reset_simulation()
            self._reset_controller()
        
    def _reset_controller(self):
        rospy.loginfo("resetting controller's state" )
        self.controller.reset_state()
        
    def _imu_callback(self, imu_data):
        self.qorientation = np.quaternion( imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w )
        quat_vecs = quaternion.as_rotation_matrix( self.qorientation )
        self.euler_angles = rotationMatrixToEulerAngles( quat_vecs )           ## yaw, pitch, roll
        pitchrad = self.euler_angles[1]
        
        ## change pitch sign, to the same as of wheel speed
        self.pitch = math.degrees(pitchrad)
        ## rospy.loginfo(rospy.get_caller_id()+"Imu received: %r %r", imuangle, angledeg )
        
#         self.euler_angles = quaternion.as_euler_angles(self.qorientation)           ## yaw pitch roll
#         rospy.loginfo(rospy.get_caller_id()+"Imu received: %r", self.euler_angles )

    def _wheel_callback(self, wheel_data):
        self.wheel_speed = wheel_data.data
        