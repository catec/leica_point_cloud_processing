#!/usr/bin/env python

import rospy 
import rospkg 
import tf
from math import pi
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def get_next_pose(roll, pitch, yaw):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose = Pose()
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose.orientation

def main():
    rospy.init_node('move_c5_tilt')
    r = rospy.Rate(10)

    state_msg = ModelState()
    state_msg.model_name = 'c5'
    state_msg.pose.position.x = 0
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 0.125
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_model_state')

    yaw = 0

    while not rospy.is_shutdown():

        # update rotation value
        yaw = 0 if yaw>(2*pi) else yaw+0.01
        print("setting yaw: {}".format(yaw))
        state_msg.pose.orientation = get_next_pose(0,0,yaw)

        # move model
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass