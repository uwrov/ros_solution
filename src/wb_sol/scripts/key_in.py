#!/usr/bin/env python3
import rospy
import curses
from geometry_msgs.msg import Twist

# Send messages to /wheely_boi/wheely_boi/cmd based on
# keyboard input from user
def move():
    # Create a new node
    rospy.init_node('wheely_boi', anonymous=True)
    
    # Create a publisher and point it to a topic
    velocity_publisher = rospy.Publisher('/wheely_boi/wheely_boi/cmd', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    # Initailize object we'll be publishing
    t = Twist()
    t.linear.x = 0
    t.angular.z = 0

    # Collecting User Input
    while not rospy.is_shutdown():
        key = curses.wrapper(getch_c)
        if (key == ord('w')):
            t.linear.x = min(t.linear.x + 0.1, 1.0)
        elif (key == ord('a')):
            t.angular.z = min(t.angular.z + 0.1, 1.0)
        elif (key == ord('s')):
            t.linear.x = max(t.linear.x - 0.1, -1.0)
        elif (key == ord('d')):
            t.angular.z = max(t.angular.z - 0.1, -1.0)
        elif (key == ord('q')):
            t.linear.x = 0
            t.angular.z = 0
        
        t.linear.x = round(t.linear.x, 1)
        t.angular.z = round(t.angular.z, 1)

        rospy.loginfo("Sending Command v:" + str(t.linear.x) 
                      + ", y:" + str(t.angular.z))
        velocity_publisher.publish(t)
        rate.sleep()


# Grab input from user (nonblocking)
def getch_c(stdscr):
    # do not wait for input when calling getch
    stdscr.nodelay(1)

    # get keyboard input, returns -1 if none available
    c = stdscr.getch()
    if c != -1:
        return c


if __name__ == '__main__':
    move()

