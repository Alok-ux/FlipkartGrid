#!/usr/bin/env python3

from phase1_goal_detector.srv import GoalArray, GoalArrayResponse
import rospy

def handle_goal_response(req):
    return GoalArrayResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('get_goal_array')
    rospy.Service('get_goal_array', GoalArray, handle_goal_response)
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
