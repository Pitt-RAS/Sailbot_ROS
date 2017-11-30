# sailbot_sim

Simulates a boat that acts according to the simplified polar diagram presented in "Autonomous sailboat navigation for short course racing".

## Nodes

### `odom_sim.py` -  Odometry simulator

**Subscribes to:**
* `cmd_heading` (Float32) - The current commanded boat heading in degrees
* `true_wind` (Vector3) - The true wind vector

**Publishes:**
* `odom` (Odometry) - The pose of the boat
* `base_link -> odom` transform

**Services:**
* `sim_reset_pose` - Resets the position and heading of the simulated boat to 0

**Params:**
* `~rate` - The update rate for the simulator

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

