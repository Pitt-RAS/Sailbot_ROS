# Current goal position or direction for the boat, to accomplish current objective
# goalType: 0: goal point, 1: goal direction, 2: station keeping
# goalPoint: In UTM frame
Header header
int32 goalType
geometry_msgs/Point goalPoint 
float32 goalDirection
bool useBox
geometry_msgs/Point box1
geomtry_msgs/Point box2
geometry_msgs/Point box3
geometry_msgs/Point box4
