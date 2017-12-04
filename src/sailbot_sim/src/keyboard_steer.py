#!/usr/bin/env python2
import rospy
import curses
from std_msgs.msg import Float32

def main(stdscr):
    rospy.init_node("sim_keyboard_steer")
    thetaMsg = Float32()
    thetaMsg.data = 0;
    thetaPub = rospy.Publisher("cmd_heading", Float32, queue_size=10)

    while not rospy.is_shutdown():
        c = stdscr.getch()
        if c == ord('a'):
            thetaMsg.data += 1 
        elif c == ord('d'):
            thetaMsg.data -= 1
        thetaPub.publish(thetaMsg);

curses.wrapper(main)



