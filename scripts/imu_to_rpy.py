#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3

class ImuToRpy():
    """ Class to get the orientation messages of Imu and spit them out
    again in roll pitch yaw in another topic for ease of debugging.
    """
    def __init__(self):
        # self.myo_name = rospy.get_param(self.myo + '/name')
        self.imu_sub = rospy.Subscriber('imu',
                                        Imu, self.imu_cb, queue_size=10)
        self.rpy_pub = rospy.Publisher('imu_rpy',
                                       Vector3, queue_size=10)

    def imu_cb(self, data):
        """:type data: Imu"""
        roll, pitch, yaw = euler_from_quaternion([data.orientation.x,
                                                 data.orientation.y,
                                                 data.orientation.z,
                                                 data.orientation.w])
        rpy_msg = Vector3()
        rpy_msg.x = roll
        rpy_msg.y = pitch
        rpy_msg.z = yaw
        self.rpy_pub.publish(rpy_msg)


if __name__ == '__main__':
    rospy.init_node('myo_imu_rpy', log_level=rospy.INFO)
    itrpy = ImuToRpy()
    rospy.spin()
