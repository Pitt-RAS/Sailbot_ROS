# sailbot_sim

Simulates a boat that acts according to the simplified polar diagram presented in "Autonomous sailboat navigation for short course racing".

## Nodes

### `odom_sim.py` -  Odometry simulator

**Subscribes to:**
* `cmd_heading` (Int32) - The current commanded boat heading in degrees
* `true_wind` (Vector3) - The true wind vector

**Publishes:**
* `odom` (Odometry) - The pose of the boat
* `wind_speed` (Int32) - The current relative wind speed
* `wind_direction` (Int32) - The current relative wind direction in degrees
* `boat -> odom` transform

**Services:**
* `sim_reset_pose` - Resets the position and heading of the simulated boat to 0

**Params:**
* `~rate` - The update rate for the simulator
* `~maxomega` - The max rotational rate of the simulated boat. The boat will rotate constantly at this rate until it reaches the commanded angle
* `/sim_wind_vector/x` - The X component of the ground truth true wind vector
* `/sim_wind_vector/y` - The Y component of the ground truth true wind vector

## Launch
### `sim.launch`

Start the simulator and publish a static transform for the sail. Also sets `robot_description` to `boat.urdf` for the RobotModel visualization in RViz.

The default true wind vector is (-2, 0).

### Args
* `sim_wind_x`
* `sim_wind_y`

The wind vector can be set without editing the roslaunch file. For example: `roslaunch sailbot_sim sim_with_rviz.launch sim_wind_x:=-2 sim_wind_y:=0`

### `sim_with_rviz.launch`

Same as `sim.launch`, but also starts RViz with the robot model and wind vectors.

## Using the simulator

To command the boat heading, publish a Int32 containing the new absolute heading of the boat on `/cmd_heading` in degrees. In the simulator, the boat will turn to this heading at a constant rotational rate. The velocity of the boat will always be determined through the polar plot.

Here is a simple example of publishing a fixed heading:

1. Remember to start the sim with: `roslaunch sailbot_sim sim_with_rviz.launch`

2. Create a python file (anglePublisher.py) with this code:

```python
#!/usr/bin/env python2
import rospy
from std_msgs.msg import Int32

rospy.init_node("heading_sender")

headingPub = rospy.Publisher("/cmd_heading", Int32, queue_size=10)

# Set the angle to 45 degrees
angleMessage = Int32(45)

# Publish at 10hz
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    headingPub.publish(angleMessage)
    rate.sleep()
```

3. Run it with `python anglePublisher.py`

