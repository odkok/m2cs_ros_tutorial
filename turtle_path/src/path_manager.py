#!/usr/bin/env python
import rospy
from math import pi, fmod, sin, cos, sqrt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_path.srv import SetOrientation, WalkDistance
# hint: some imports are missing

cur_pos = Pose()

def cb_pose(data): # get the current position from subscribing the turtle position
    global cur_pos
    cur_pos = data

def cb_walk(req):
    if (req.distance < 0):
        return False

    # hint: calculate the projected (x, y) after walking the distance,
    # and return false if it is outside the boundary
    old_x, old_y = cur_pos.x, cur_pos.y
    theta = cur_pos.theta
    new_x = old_x + req.distance * cos(theta)
    new_y = old_y + req.distance * sin(theta)
    if new_x < 0 or new_x > 11 or new_y < 0 or new_y > 11:
        return False

    rate = rospy.Rate(100) # 100Hz control loop
    dist = req.distance

    while not rospy.is_shutdown(): # control loop
        
        # in each iteration of the control loop, publish a velocity

        # hint: you need to use the formula for distance between two points
        dist = sqrt((cur_pos.x - new_x) ** 2 + (cur_pos.y - new_y) ** 2)
        if dist < 0.1:
            break
        vel = Twist()
        vel.linear.x = dist
        pub.publish(vel)
        
        rate.sleep()
    
    vel = Twist() # publish a velocity 0 at the end, to ensure the turtle really stops
    pub.publish(vel)

    return True

def cb_orientation(req):

    rate = rospy.Rate(100) # 100Hz control loop
    
    while (not rospy.is_shutdown()): # control loop
        
        # in each iteration of the control loop, publish a velocity

        # hint: signed smallest distance between two angles: 
        # see https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        dist = fmod(req.orientation - cur_pos.theta + pi + 2 * pi, 2 * pi) - pi
        if abs(dist) < pi / 100:
            break
        vel = Twist()
        vel.angular.z = dist
        pub.publish(vel)
        
        rate.sleep()
    
    vel = Twist() # publish a velocity 0 at the end, to ensure the turtle really stops
    pub.publish(vel)

    return True

if __name__ == '__main__':
    rospy.init_node('path_manager')
    
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=cb_pose)
    
    ## init each service server here:
    rospy.Service("/set_orientation", SetOrientation, cb_orientation)		# callback to cb_orientation
    rospy.Service("/walk_distance", WalkDistance, cb_walk)		# callback to cb_walk

    rospy.spin()
