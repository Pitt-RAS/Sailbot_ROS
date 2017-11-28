#
# Algorithm from:
#
#  "Autonomous sailboat navigation for short course racing"
#
#    Roland Stelzer, Tobias Proll
#
#    ScienceDirect
#

from math import sqrt, pi, sin, cos

# Returns an instantaneous heading for moving from position_boat to
# position_target, given the current heading of the boat and the wind.
#
# The beating_parameter (A.K.A. p_c) controls the width of the rectangular
# beating band, i.e., how far we beat back and forth against the wind.
#
# Parameters position_boat and position_target are (x, y) tuples of meters.
#
# Parameter speed_wind is m/s. Parameter heading_wind is radians.
#
# Parameter beating_parameter is a real in (1.0, infinity).
#
# All parameters are of the ground inertial reference frame. Headings are
# absolute, not relative to the boat.
#
# The coorindate plane is right-handed. Zero degrees is along the positive
# z-axis, and counter-clockwise is positive.
#
# The wind heading is the direction the wind is coming _from_, not the
# direction of its motion.

def heading(position_boat, heading_boat,
            position_target,
            speed_wind, heading_wind,
            beating_parameter):
    assert(speed_wind >= 0.0)
    assert(0.0 <= heading_wind and heading_wind <= 2*pi)
    assert(beating_parameter > 1.0)

    displacement_target = (position_target[0] - position_boat[0],
                           position_target[1] - position_boat[1])

    distance_target = sqrt(  displacement_target[0]**2 \
                           + displacement_target[1]**2)

    beating_parameter = 1 + beating_parameter / distance_target

    (heading1, speed1)  = _optimum_heading(displacement_target,
                                           speed_wind, heading_wind,
                                           True)

    (heading2, speed2)  = _optimum_heading(displacement_target,
                                           speed_wind, heading_wind,
                                           False)

    if   _smallest_angle(heading1, heading_boat) \
       < _smallest_angle(heading2, heading_boat):
        if speed1 * beating_parameter < speed2:
            return heading2
        else:
            return heading1
    else:
        if speed2 * beating_parameter < speed1:
            return heading1
        else:
            return heading2

# Maximizes the boat velocity component in the direction of the target
# position. Returns the optimal heading and speed.
#
# If direction is True, scans from heading_wind to heading_begin + pi. Else
# scans from heading_wind to heading_begin - pi.
#
# Parameter displacement_target is a (x, y) tuple of meters.
#
# Parameter speed_wind is m/s.
#
# Parameters heading_wind is radians.
#
# Parameter direction is a bool.
#
# Return value is (radians, m/s).

def _optimum_heading(displacement_target, speed_wind, heading_wind, direction):
    # m/s in target direction
    useful_speed_max = 0.0

    # radians
    heading_max = heading_wind

    offset = 0.0
    step_size = pi / 1000.0

    while offset <= pi:
        if direction:
            heading_boat = heading_wind + offset
        else:
            heading_boat = heading_wind - offset
        heading_boat = _wrap_angle(heading_boat)

        angle_wind   = _wrap_angle(heading_wind - heading_boat)

        speed_boat = _boat_speed(speed_wind, angle_wind)
        velocity_boat = (speed_boat * cos(heading_boat),
                         speed_boat * sin(heading_boat))

        useful_speed =   velocity_boat[0] * displacement_target[0] \
                       + velocity_boat[1] * displacement_target[1]

        if useful_speed > useful_speed_max:
            heading_max = heading_boat
            useful_speed_max = useful_speed

        offset += step_size

    return (heading_max, useful_speed_max)

# Returns the boat speed for relative wind speed and angle.
#
# A.K.A f_polar
#
# Parameters speed and angle are radians.
#
# Parameter angle is the angle the wind _comes_ _from_.
#
# Return value is m/s.

def _boat_speed(speed, angle):
    assert(0.0 <= angle and angle <= 2*pi)

    front = angle <   pi/4 or  angle > 7*pi/4
    back  = angle > 3*pi/4 and angle < 5*pi/4

    if front or back:
        return 0.0

    return speed

# Returns the smallest angle between two headings.

def _smallest_angle(heading1, heading2):
    assert(0.0 <= heading1 <= 2*pi)
    assert(0.0 <= heading2 <= 2*pi)

    angle = abs(heading1 - heading2)

    if angle >= pi:
        angle = 2*pi - angle

    return angle

# Returns an angle in radians wrapped to [0, 2*pi).

def _wrap_angle(angle):
    return angle % (2*pi)
