#!/usr/bin/env python3
from robot_actions.srv import MovementData, MovementDataResponse
import rospy

v = [
    [(0,-1),(0,-3),(-2,-1),(2,-1)],
    [(1,0),(3,0),(1,-2),(1,2)],
    [(0,1),(0,3),(2,1),(-2,1)],
    [(-1,0),(-3,0),(-1,2),(-1,-2)],
    [0,0,-1,1]
]

def handle_move(req):
    x = req.x+v[req.dir][req.cmd][0]
    y = req.y+v[req.dir][req.cmd][1]
    d = (req.dir + v[4][req.cmd])%4
    d = d if d>=0 else 3
    return MovementDataResponse(x,y,d)

if __name__ == "__main__":
    rospy.init_node('move_server')
    s = rospy.Service('movement', MovementData, handle_move)
    rospy.spin()