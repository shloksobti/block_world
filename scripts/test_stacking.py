#!/usr/bin/env python
from block_world.srv import BlankSrv
from block_world.msg import StackBlocksAction, StackBlocksGoal

import rospy
import actionlib

def spawn_blocks_client():
    rospy.wait_for_service('/spawn_blocks')
    try:
        spawn = rospy.ServiceProxy('/spawn_blocks', BlankSrv)
        spawn()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def reset_client():
    rospy.wait_for_service('/reset')
    try:
        reset = rospy.ServiceProxy('/reset', BlankSrv)
        reset()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def stack_blocks_client():
    client = actionlib.SimpleActionClient('/stack_blocks', StackBlocksAction)
    client.wait_for_server()
    goal = StackBlocksGoal(top="Block1", bottom="Block2")
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


if __name__ == "__main__":
    rospy.init_node('test_stacking_node')
    while True:
        spawn_blocks_client()
        input("Press Enter to Stack...")
        result = stack_blocks_client()
        if (not result.success):
            print("Stacking failed...resetting environment")
            reset_client()

        yes_no = input("Do you want to continue (y/n)...? ")
        if yes_no == "n":
            break
