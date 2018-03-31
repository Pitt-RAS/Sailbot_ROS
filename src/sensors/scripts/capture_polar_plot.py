import rospy
import numpy as np
from tf import TransformListener
from sailbot_sim.msg import TrueWind
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from math import floor


class PolarCaptureNode:
    def __init__(self):
        self._odom_sub = rospy.Subscriber("odometry/filtered", Odometry, self._update_odom)
        self._truewind_sub = rospy.Subscriber("true_wind", TrueWind, self._update_truewind)

        self._odom = None
        self._truewind = None

        self._transformer = TransformListener()

        self._plot = [None for n in range(360)]

    def _update_odom(self, odom):
        self._odom = odom

    def _update_truewind(self, truewind):
        self._truewind = truewind

    def update(self):
        if self._odom is None or self._truewind is None:
            return

        try:
            velocity = Vector3Stamped(header=self._odom.header, vector=self._odom.twist.twist.linear)
            velocity = self._transformer.transformVector3("boat", velocity)
            self._plot[self._truewind.direction] = velocity.vector.x
            print("{} at {}".format(self._truewind.direction, velocity.vector.x))
        except e:
            pass

rospy.init_node("polar_plot_capture")
node = PolarCaptureNode()
rate = rospy.Rate(30)
while not rospy.is_shutdown():
    node.update()
    rate.sleep()

x = []
y = []

for angle, velocity in enumerate(node._plot):
    if velocity is not None:
        x.append(angle)
        y.append(velocity)

plot = np.interp(range(360), x, y)

print("angle,velocity")
for angle, velocity in enumerate(plot):
    print("{},{}".format(angle, velocity))

