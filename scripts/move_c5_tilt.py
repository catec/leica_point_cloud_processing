#!/usr/bin/env python

import rospy 
import rospkg 
import tf
from math import pi
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState, GetModelState

def orientation_to_quaternion(orientation):
    quaternion = (  orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w   )
    return quaternion

def quaternion_to_orientation(quaternion):
    pose = Pose()
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]  
    return pose.orientation

def get_next_orientation(current_orientation,roll, pitch, yaw):
    current_rotation = orientation_to_quaternion(current_orientation)
    euler_angle = tf.transformations.euler_from_quaternion(current_rotation)

    r = euler_angle[0] + roll
    p = euler_angle[1] + pitch
    y = euler_angle[2] + yaw
    quaternion = tf.transformations.quaternion_from_euler(r, p, y)
    orientation = quaternion_to_orientation(quaternion)

    return orientation

def get_init_pose():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_state("c5", "world")
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def main():
    rospy.init_node('move_c5_tilt')
    r = rospy.Rate(5)

    init_model_state = get_init_pose()

    state_msg = ModelState()
    state_msg.model_name = 'c5'
    state_msg.pose = init_model_state.pose

    print("Posicion inicial: ")
    print(state_msg.pose)

    rospy.wait_for_service('/gazebo/set_model_state')

    yaw = 0

    while not rospy.is_shutdown():

        # update rotation value
        # yaw = 0 if yaw>(2*pi) else yaw+0.01
        yaw_inc = 0.01
        state_msg.pose.orientation = get_next_orientation(state_msg.pose.orientation,0,0,yaw_inc)

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