#!/usr/bin/python
import rospy
import tf
from gpd.msg import GraspConfigList
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

grasps = []

def callback(msg):
    global grasps
    grasps = msg.grasps

# Create a ROS node.
rospy.init_node('get_grasps')

# Subscribe to the ROS topic that contains the grasps.
sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

# Wait for grasps to arrive.
rate = rospy.Rate(1)

while not rospy.is_shutdown():    
    if len(grasps) > 0:
        rospy.loginfo('Received %d grasps.', len(grasps))
        break
    rate.sleep()
    
grasp = grasps[0] # grasps are sorted in descending order by score
print(grasp)


############################ quaternion transform #############################
q = quaternion_from_euler(grasp.approach.x, grasp.approach.y, grasp.approach.z)
print(f"The quaternion representation is {q[0]} {q[1]} {q[2]} {q[3]}.")

q_msg = Quaternion(q[0], q[1], q[2], q[3])
grasp_pose_source = PoseStamped()
grasp_pose_source.header.frame_id = "camera_depth_optical_frame"
grasp_pose_source.pose.orientation = q_msg
grasp_pose_source.pose.position = grasp.bottom
print("The source grasp pose: ", grasp_pose_source)

############################# TF ###################################
cam_2_ur = tf.TransformListener()
cam_2_ur.waitForTransform('booksink_link', 'camera_depth_optical_frame', rospy.Time(), rospy.Duration(1.0))
grasp_pose_ur = cam_2_ur.transformPose("booksink_link", grasp_pose_source)
print("The transformed grasp pose: ", grasp_pose_ur)



pub = rospy.Publisher('/get_grasps/gpd_grasps', PoseStamped, queue_size=10)

while not rospy.is_shutdown():
    pub.publish(grasp_pose_ur)
    # print(grasp_pose_source)
    rate.sleep()

