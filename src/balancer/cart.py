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


import math
import numpy as np
import quaternion
 
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

from .cart_driver import CartDriver


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])


class Cart:
    
    def __init__(self):
        self.qorientation = None    ## quaternion
        self.euler_angles = None    ## in radians
        self.pitch = None           ## in degrees
    
    def run(self, driver: CartDriver):
        # [INFO] [1546208845.310780, 1800.357000]: /pic_controller_32685_1546208844652Imu received: 
        # ['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__getstate__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__setstate__', '__sizeof__', '__slots__', '__str__', '__subclasshook__', 
        # '_check_types', '_connection_header', '_full_text', '_get_types', '_has_header', '_md5sum', '_slot_types', '_type', 
        # 'angular_velocity', 'angular_velocity_covariance', 'deserialize', 'deserialize_numpy', 'header', 'linear_acceleration', 'linear_acceleration_covariance', 'orientation', 'orientation_covariance', 'serialize', 'serialize_numpy']
        rospy.Subscriber("/teeterbot/imu", Imu, self._imu_callback)
        
        left_pub = rospy.Publisher('/teeterbot/left_torque_cmd', Float64, queue_size=10)
        right_pub = rospy.Publisher('/teeterbot/right_torque_cmd', Float64, queue_size=10)
        
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            ##str = "hello world %s"%rospy.get_time()
            #val = random.uniform(-1.0, 1.0)
            
            output = driver.steer( self.pitch )
            left_out = output[0]
            right_out = output[1]
            rospy.loginfo("torque: %r %r", left_out, right_out)
            
            left_pub.publish(left_out)
            right_pub.publish(right_out)
            r.sleep()

    def _imu_callback(self, imu_data):
        self.qorientation = np.quaternion( imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w )
        quat_vecs = quaternion.as_rotation_matrix( self.qorientation )
        self.euler_angles = rotationMatrixToEulerAngles( quat_vecs )           ## yaw, pitch, roll
        pitchrad = self.euler_angles[1]
        self.pitch = math.degrees(pitchrad)
        ## rospy.loginfo(rospy.get_caller_id()+"Imu received: %r %r", imuangle, angledeg )
        
#         self.euler_angles = quaternion.as_euler_angles(self.qorientation)           ## yaw pitch roll
#         rospy.loginfo(rospy.get_caller_id()+"Imu received: %r", self.euler_angles )
