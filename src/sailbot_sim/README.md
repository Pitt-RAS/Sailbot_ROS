# sailbot_sim

Simulates a boat that acts according to the simplified polar diagram presented in "Autonomous sailboat navigation for short course racing".

## Nodes

### `odom_sim.py` -  Odometry simulator

**Subscribes to:**
* `cmd_heading` (Float32) - The current commanded boat heading in radians 
* `true_wind` (Vector3) - The true wind vector

**Publishes:**
* `odom` (Odometry) - The pose of the boat
* `base_link -> odom` transform

**Services:**
* `sim_reset_pose` - Resets the position and heading of the simulated boat to 0

**Params:**
* `~rate` - The update rate for the simulator
* `~maxomega` - The max rotational rate of the simulated boat. The boat will rotate constantly at this rate until it reaches the commanded angle

### `sim_windsensor.py` - Simulate the wind sensor

**Subscribes to:**
* `odom` (Odometry) - The pose of the boat

**Publishes:**
* `true_wind` (Vector3) - The true wind vector
* `relative_wind` (Vector3) - The relative wind vector
* `true_wind_marker` (visualization_msgs/Marker) - RViz marker for showing the true wind vector, centered in the odom frame
* `relative_wind_marker` (visualization_msgs/Marker) - RViz marker for showing the relative wind vector, centered in the base_link frame

**Params:**
* `/sim_wind_vector/x` - The X component of the true wind vector
* `/sim_wind_vector/y` - The Y component of the true wind vector

Defaults to (0,0) if no param is set. 

### `keyboard_steer.py` - Drive the boat using your keyboard

**Publishes:**
* `/cmd_heading` (Float32) - The target heading


## Launch
### `sim.launch`

Start the odom sim, wind sensor sim, and publish a static transform for the sail. Also sets `robot_description` to `boat.urdf` for the RobotModel visualization in RViz.

The default true wind vector is (-2, 0).

### Args
* `sim_wind_x`
* `sim_wind_y`

The wind vector can be set without editing the roslaunch file. For example: `roslaunch sailbot_sim sim_with_rviz.launch sim_wind_x:=-2 sim_wind_y:=0`

### `sim_with_rviz.launch`

Same as `sim.launch`, but also starts RViz with the robot model and wind vectors.

## Using the simulator

To command the boat heading, publish a Float32 containing the new absolute heading of the boat on `/cmd_heading` in radians. In the simulator, the boat will turn to this heading at a constant rotational rate. The velocity of the boat will always be determined through the polar plot.

Here is a simple example of publishing a fixed heading:

1. Remember to start the sim with: `roslaunch sailbot_sim sim_with_rviz.launch`

2. Create a python file (anglePublisher.py) with this code:

```python
#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float32
from math import radians

rospy.init_node("heading_sender")

headingPub = rospy.Publisher("/cmd_heading", Float32, queue_size=10)

# Set the angle to 45 degrees
angleMessage = Float32(radinas(45))

while not rospy.is_shutdown():
  headingPub.publish(angleMessage)

```

3. Run it with `python anglePublisher.py`

