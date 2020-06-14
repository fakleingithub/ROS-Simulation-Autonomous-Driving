#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf
import numpy as np
from tf.transformations import euler_from_quaternion

rospy.init_node('odom_pub')
odom_pub=rospy.Publisher('odom', Odometry, queue_size=1)
datapreprocessing_pose_pub=rospy.Publisher('datapreprocessing_pose', Pose2D, queue_size=1)
datapreprocessing_velocity_pub=rospy.Publisher('datapreprocessing_velocity', Float32, queue_size=1)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

br = tf.TransformBroadcaster()

odom=Odometry()
header = Header()
header.frame_id='/odom'

datapreprocessing_pose=Pose2D()

datapreprocessing_velocity=Float32()

model = GetModelStateRequest()
model.model_name='cc_2019_car'

r = rospy.Rate(2)

while not rospy.is_shutdown():
    result = get_model_srv(model)

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header
    odom_pub.publish (odom)

    quaternion = (
       result.pose.orientation.x,
       result.pose.orientation.y,
       result.pose.orientation.z,
       result.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    datapreprocessing_pose.x = result.pose.position.x
    datapreprocessing_pose.y = result.pose.position.y
    datapreprocessing_pose.theta = yaw

    datapreprocessing_pose_pub.publish (datapreprocessing_pose)

    datapreprocessing_velocity = np.sqrt((result.twist.linear.x)**2 + (result.twist.linear.y)**2) 
    datapreprocessing_velocity_pub.publish (datapreprocessing_velocity)

    br.sendTransform((result.pose.position.x, result.pose.position.y, result.pose.position.z),
                 (result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w),
                 rospy.Time.now(),
                 "rear_axis_middle_ground",
                 "base_link")

    r.sleep()
