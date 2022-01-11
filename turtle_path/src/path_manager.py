#!/usr/bin/env python
import rospy
from math import pi, fmod, sin, cos, sqrt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_path.srv import SetOrientation, WalkDistance
# hint: some imports are missing

cur_pos = Pose()


def parse_angle(deg, dist): # get the x, y distance req to move
    if deg > 3 * pi / 2:
        calDeg = deg - 3 * pi / 2
        return dist * sin(calDeg), dist * cos(calDeg) * -1
    if deg > pi:
        calDeg = deg - pi
        return dist * cos(calDeg) * -1, dist * sin(calDeg) * -1
    if deg > pi / 2:
        calDeg = deg - pi / 2
        return dist * sin(calDeg) * -1, dist * cos(calDeg)
    return dist * cos(deg), dist * sin(deg)


def cb_pose(data): # get the current position from subscribing the turtle position
    global cur_pos
    cur_pos = data


def cb_walk(req):
    if req.distance < 0:
        return False

    # hint: calculate the projected (x, y) after walking the distance,
    # and return false if it is outside the boundary

    x, y = parse_angle(cur_pos.theta, req.distance)
    x += cur_pos.x
    y += cur_pos.y
    # now x, y stores the req specific pos
    if (x > 11 or x < 0) or (y > 11 or y < 0):
        return False

    rate = rospy.Rate(100) # 100Hz control loop

    while not (x - 0.05 < cur_pos.x < x + 0.05) or not (y - 0.05 < cur_pos.y < y + 0.05): # control loop
        
        # in each iteration of the control loop, publish a velocity

        # hint: you need to use the formula for distance between two points
        t = Twist()
        t.linear.x = sqrt((x - cur_pos.x) ** 2 + (y - cur_pos.y) ** 2)
        pub.publish(t)

        rate.sleep()
    
    vel = Twist() # publish a velocity 0 at the end, to ensure the turtle really stops
    pub.publish(vel)

    return True


def cb_orientation(req):

    rate = rospy.Rate(100) # 100Hz control loop
    

    # if req.orientation > 2pi (i.e. one cycle), trim out excessive cycle
    deg = req.orientation % (2 * pi)
    # since idk why the while loop does not stop when deg > pi so I choose to use dist as the condition for the while loop
    dist = fmod(deg - cur_pos.theta + pi + 2 * pi, 2 * pi) - pi

    while not (-0.05 < dist < 0.05): # control loop
        
        # in each iteration of the control loop, publish a velocity

        # hint: signed smallest distance between two angles: 
        # see https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        dist = fmod(deg - cur_pos.theta + pi + 2 * pi, 2 * pi) - pi

        t = Twist()
        t.angular.z = dist
        pub.publish(t)

        rate.sleep()
    
    vel = Twist() # publish a velocity 0 at the end, to ensure the turtle really stops
    pub.publish(vel)

    return True


if __name__ == '__main__':
    rospy.init_node('path_manager')
    
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    # publisher of the turtle velocity
    sub = rospy.Subscriber('turtle1/pose', Pose, cb_pose)
    # subscriber of the turtle position, callback to cb_pose
    
    ## init each service server here:
    # rospy.Service( ... )		# callback to cb_orientation
    # rospy.Service( ... )		# callback to cb_walk
    rospy.Service("set_orientation", SetOrientation, cb_orientation)
    rospy.Service("walk_distance", WalkDistance, cb_walk)
    
    rospy.spin()
