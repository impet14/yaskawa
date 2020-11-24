#!/usr/bin/env python

import threading
import unittest
import rospy
import rostest

import collections
import sensor_msgs.msg

class TestOpenZen(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_openzen')
    
    def setUp(self):
        self.openzen_imu_access = threading.Lock()
        self.openzen_imu = collections.deque(maxlen=1000)
        rospy.Subscriber('/imu/data', sensor_msgs.msg.Imu, self.incoming_imu, queue_size=1000)

    def incoming_imu(self, imu_data):
        with self.openzen_imu_access:
            self.openzen_imu.append(imu_data)

    def test_imu_data_incoming(self):
        # wait for some data to arrive
        rospy.sleep(0.5)

        with self.openzen_imu_access:
            self.assertTrue(len(self.openzen_imu) > 0)

            # some sanity check on content
            last_imu = self.openzen_imu.pop()

            self.assertAlmostEqual(0.0, last_imu.linear_acceleration.x, 2)
            self.assertAlmostEqual(0.0, last_imu.linear_acceleration.y, 2)
            self.assertAlmostEqual(9.81, last_imu.linear_acceleration.z, 2)

            # the test sensor will output some arbitrary gyro values
            self.assertLess(0.0, last_imu.angular_velocity.x)
            self.assertLess(0.0, last_imu.angular_velocity.y, 2)
            self.assertLess(0.0, last_imu.angular_velocity.z, 2)

if __name__ == '__main__':
    rostest.rosrun('openzen_driver', 'test_openzen_readout', TestOpenZen)
