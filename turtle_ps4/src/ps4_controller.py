#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from std_srvs.srv import Empty
from m2_ps4.srv import Ps4Data
# hint: some imports are missing

old_data = Ps4Data()
old_t = Twist()
speed = 1


def getDirection(data):
    global speed
    if data.dpad_y > old_data.dpad_y:
        speed += 1
    elif data.dpad_y < old_data.dpad_y:
        speed -= 1
    if speed > 5 or speed < 1:
        speed = 1
    return data.hat_rx * speed, data.hat_ly * speed;


def getColor(data):
    if data.triangle and not old_data.triangle:
        return 0, 255, 0
    if data.circle and not old_data.circle:
        return 255, 0, 0
    if data.cross and not old_data.cross:
        return 0, 0, 255
    if data.square and not old_data.square:
        return 100, 0, 100
    return 0, 0, 0


def callback(data):
    global old_data
    global old_t
    
    # you should publish the velocity here!
    
    # hint: to detect a button being pressed, you can use the following pseudocode:
    # 
    # if ((data.button is pressed) and (old_data.button not pressed)),
    # then do something...
    
    t = Twist()
    t.angular.z, t.linear.x = getDirection(data)
    if not (t.angular.z == old_t.angular.z and t.linear.x == old_t.angular.z):
        pub.publish(t)

    # setting color
    r, g, b = getColor(data)
    if not (r == 0 and g == 0 and b == 0):
        srv_col(r, g, b)

    # clearing screen (i.e. the pen trail)
    if data.ps and not old_data.ps:
        srv_clr()

    old_t = t
    old_data = data


if __name__ == '__main__':
    rospy.init_node('ps4_controller')
    
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    # publisher object goes here... hint: the topic type is Twist
    sub = rospy.Subscriber('input/ps4_data', Ps4Data, callback)
    # subscriber object goes here
    
    # one service object is needed for each service called!
    srv_col = rospy.ServiceProxy('turtle1/set_pen', SetPen)
    srv_clr = rospy.ServiceProxy('clear', Empty)
    # service client object goes here... hint: the srv type is SetPen
    # fill in the other service client object...
    
    rospy.spin()
