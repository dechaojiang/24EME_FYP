#!/usr/bin/python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs

def simple_pick_place():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place',
                    anonymous=True)
    
    ur_group = moveit_commander.MoveGroupCommander("UR5_movegroup")
    rbt_group= moveit_commander.MoveGroupCommander("Robotiq_movegroup")

    ur_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    ur_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for UR')

    rbt_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    rbt_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for Robotiq')

    ur_group.set_named_target("UR_INIT")
    plan_success, plan, planning_time, error_code = ur_group.plan()
    ur_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    ur_goal.trajectory = plan
    ur_client.send_goal(ur_goal)
    ur_client.wait_for_result()
    init_Pose = ur_group.get_current_pose("ur5/tool0").pose

    ur_group.set_named_target("UR_PREGRASP")
    plan_success, plan, planning_time, error_code = ur_group.plan()
    ur_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    ur_goal.trajectory = plan
    ur_client.send_goal(ur_goal)
    ur_client.wait_for_result()

    Pose = ur_group.get_current_pose("ur5/tool0")
    down_Pose = Pose
    up_Pose = Pose.pose
    print(Pose)
    down_Pose.pose.position.z = Pose.pose.position.z - 0.07
    print(down_Pose)

    ur_group.set_pose_target(down_Pose)
    ur_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    ur_goal.trajectory = plan
    ur_client.send_goal(ur_goal)
    ur_client.wait_for_result()

    rbt_group.set_named_target("RBT_GRASP")
    plan_success, plan, planning_time, error_code = rbt_group.plan()
    rbt_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    rbt_goal.trajectory = plan
    rbt_client.send_goal(rbt_goal)
    rbt_client.wait_for_result()

    # waypoints2 = []
    # up_Pose.position.z = Pose.pose.position.z + 0.2
    # waypoints2.append(init_Pose)
    # fraction2 = 0.0
    # for count_cartesian_path in range(0,3):
    #   if fraction2 < 1.0:
    #     (plan_cartesian, fraction2) = ur_group.compute_cartesian_path(waypoints2, 0.01, 0.0)
    #   else:
    #     break
    # ur_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    # ur_goal.trajectory = plan_cartesian
    # ur_client.send_goal(ur_goal)
    # ur_client.wait_for_result()




    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__=='__main__':
  try:
    simple_pick_place()
  except rospy.ROSInterruptException:
    pass