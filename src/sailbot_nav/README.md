# Sailbot Nav

ROS-ified version of Ryan's implementation of the paper: "Autonomous sailboat navigation for short course racing"

## Nodes

### `goal_point_pub.py`

Publishes a static goal point of (4,0) to /goal

### Nav planner - `node.py` 

Subscribes to:
* `/odom` (Odometry) - Current robot pose 
* `/true_wind` (Vector3) - The true wind vector
* `/goal` (PointStamped) - The goal point to navigate to

Publishes:
* `/cmd_heading` (Float32) - The target heading

Params
* `~beating_parameter`

## Launch

### `sail_to_point.launch`
Start the planner and start publishing a static goal point to `/goal`

### `nav.launch`
Just start the planner

