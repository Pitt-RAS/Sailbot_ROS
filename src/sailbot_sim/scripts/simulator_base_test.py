#!/usr/bin/env python2
import rospy
import rostest
import unittest
import numpy as np
from math import atan2, degrees

from sailbot_sim.msg import TrueWind

class SailbotSimulatorTest(unittest.TestCase):

    def test_true_wind(self):
        groundTruthWind = np.array([rospy.get_param("/sim_wind_vector/x", 0), rospy.get_param("/sim_wind_vector/y", 0)])
        groundTruthWindDirection = degrees(atan2(groundTruthWind[1], groundTruthWind[0]))
        groundTruthWindSpeed = np.linalg.norm(groundTruthWind)
        
        trueWindMsg = rospy.wait_for_message("/true_wind", TrueWind)

        self.assertAlmostEquals(trueWindMsg.speed, groundTruthWindSpeed)
        self.assertAlmostEquals(trueWindMsg.direction, groundTruthWindDirection)

rospy.init_node("simulator_base_test")
rostest.rosrun('sailbot_test', 'simulator_base_test', SailbotSimulatorTest)

