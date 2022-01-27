#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import actionlib
from time import sleep

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

if __name__ == '__main__':
    rospy.init_node('topological_navigation_client')
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    client.wait_for_server()
    
# The waypoints are created using the week 8 workshops

    goal = GotoNodeGoal()
    goal.target = "WayPoint5"
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    sleep(5)

    



    # send 1st goal
    goal = GotoNodeGoal()
    goal.target = "WayPoint0"
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    sleep(5)

    # In this the robot moves from waypoint 0 to waypoint 2

    # send 2nd goal
    goal.target = "WayPoint2"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    sleep(5)

    # In this the robot moves from waypoint 2 to waypoint 3

    # send 3rd goal
    goal.target = "WayPoint3"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    sleep(5)

    # In this the robot moves from waypoint 3 to waypoint 4

    # send 4th goal
    goal.target = "WayPoint4"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    sleep(5)
