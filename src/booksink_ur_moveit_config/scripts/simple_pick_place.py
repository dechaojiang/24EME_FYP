#!/usr/bin/python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

g_pose = None

def callback(msg):
  global g_pose
  g_pose = msg

def simple_pick_place():
    
    # initialize node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place',
                    anonymous=True)
    # creating ur group and rbt group for moveit control
    ur_group = moveit_commander.MoveGroupCommander("UR5_movegroup")
    rbt_group= moveit_commander.MoveGroupCommander("Robotiq_movegroup")

    ur_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    ur_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for UR')

    rbt_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    rbt_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for Robotiq')

    ur_goal = moveit_msgs.msg.ExecuteTrajectoryGoal() # create goal

    ur_group.set_named_target("UR_INIT") # set target
    plan_success, plan, planning_time, error_code = ur_group.plan() # plan
    ur_goal.trajectory = plan # give data to the goal
    ur_client.send_goal(ur_goal)
    ur_client.wait_for_result()
    init_Pose = ur_group.get_current_pose("ur5/tool0").pose

    ur_group.set_named_target("UR_PREGRASP")
    plan_success, plan, planning_time, error_code = ur_group.plan()
    ur_goal.trajectory = plan
    ur_client.send_goal(ur_goal)
    ur_client.wait_for_result()

    Pose = ur_group.get_current_pose("ur5/tool0").pose
    Pose.position.z = Pose.position.z - 0.07
    print(Pose)

    ur_group.set_pose_target(Pose)
    plan_success, plan, planning_time, error_code = ur_group.plan()
    ur_goal.trajectory = plan
    ur_client.send_goal(ur_goal)
    ur_client.wait_for_result()

    rbt_group.set_named_target("RBT_GRASP")
    plan_success, plan, planning_time, error_code = rbt_group.plan()
    rbt_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    rbt_goal.trajectory = plan
    rbt_client.send_goal(rbt_goal)
    rbt_client.wait_for_result()

    Pose.position.z = Pose.position.z + 0.15
    ur_group.set_pose_target(Pose)
    plan_success, plan, planning_time, error_code = ur_group.plan()
    ur_goal.trajectory = plan
    ur_client.send_goal(ur_goal)
    ur_client.wait_for_result()

    
    waypoints = []
    waypoints.append(init_Pose)
    fraction = 0.0
    for count_cartesian_path in range(0,2):
      if fraction < 1.0:
        (plan_cartesian, fraction2) = ur_group.compute_cartesian_path(waypoints, 0.01, 0.0)
      else:
        break
    ur_goal.trajectory = plan_cartesian
    ur_client.send_goal(ur_goal)
    ur_client.wait_for_result()




    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

def gpd_grasp():
    
   
    # # initialize node
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place',
                    anonymous=True)
    rate = rospy.Rate(3)

    global g_pose 
    rospy.Subscriber("/get_grasps/gpd_grasps", PoseStamped, callback)
    # creating ur group and rbt group for moveit control
    ur_group = moveit_commander.MoveGroupCommander("UR5_movegroup")
    rbt_group= moveit_commander.MoveGroupCommander("Robotiq_movegroup")

    ur_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    ur_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for UR')

    rbt_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    rbt_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for Robotiq')

    ur_goal = moveit_msgs.msg.ExecuteTrajectoryGoal() # create goal

    while not rospy.is_shutdown():
      if g_pose != None:
        break
      rate.sleep()
    
    # while not rospy.is_shutdown():
    #   print(g_pose)
    #   rate.sleep()

    ur_group.set_pose_target(g_pose)
    plan_success, plan, planning_time, error_code = ur_group.plan()
    ur_goal.trajectory = plan
    ur_client.send_goal(ur_goal)
    ur_client.wait_for_result()

  



if __name__=='__main__':
  try:
    #simple_pick_place()
    gpd_grasp()
  except rospy.ROSInterruptException:
    pass
