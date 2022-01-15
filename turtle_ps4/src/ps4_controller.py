#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim import srv
from turtlesim.srv import SetPen, SetPenRequest
from m2_ps4.msg import Ps4Data
from std_srvs.srv import Empty, EmptyRequest
# hint: some imports are missing

old_data = Ps4Data()
k = 1

def clear_path():
    rospy.wait_for_service("clear")
    clear_client = rospy.ServiceProxy("clear", Empty)
    req = EmptyRequest()
    resp = clear_client(req)
    return resp

# color: (b, g, r)
def set_pen_color(color):
    rospy.wait_for_service("/turtle1/set_pen")
    srv_col = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
    req = SetPenRequest()
    req.b = color[0]
    req.g = color[1]
    req.r = color[2]
    resp = srv_col(req)
    return resp


def callback(data):
    global old_data, pub, k,srv_col
    if data.dpad_y == 1.0 and k < 5:
        k += 1
    if data.dpad_y == -1.0 and k > 1:
        k -= 1

    vel = Twist()
    if data.hat_ly:
        vel.linear.x = data.hat_ly * k
    if data.hat_rx:
        vel.angular.z = data.hat_rx * k
    
    # you should publish the velocity here!
    pub.publish(vel)
    
    # hint: to detect a button being pressed, you can use the following pseudocode:
    # 
    # if ((data.button is pressed) and (old_data.button not pressed)),
    # then do something...

    if data.ps:
        clear_path()

    if data.triangle:
        set_pen_color((0, 255, 0))
    if data.circle:
        set_pen_color((0, 0, 255))
    if data.cross:
        set_pen_color((255, 0, 0))
    if data.square:
        set_pen_color((255, 0, 255))
    
    old_data = data

if __name__ == '__main__':
    rospy.init_node('ps4_controller')
    
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
    sub = rospy.Subscriber("input/ps4_data", Ps4Data, callback=callback)
    
    # one service object is needed for each service called!
    
    # fill in the other service client object...
    
    rospy.spin()
