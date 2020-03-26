#! /usr/bin/env python

from tf import TransformBroadcaster
import rospy
from rospy import Time
from gazebo_msgs.msg import ModelStates


MODEL_NAME = "c5"


def get_model_tf_cb(msg):
    # get index of model name
    i = msg.name.index(MODEL_NAME)

    # construct tf data
    translation = (msg.pose[i].position.x,
                   msg.pose[i].position.y,
                   msg.pose[i].position.z)

    rotation = (msg.pose[i].orientation.x,
                msg.pose[i].orientation.y,
                msg.pose[i].orientation.z,
                msg.pose[i].orientation.w)

    # publish tf
    b = TransformBroadcaster()
    b.sendTransform(translation, rotation, Time.now(), '/c5_link', '/world')


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    rospy.Subscriber("/gazebo/model_states", ModelStates, get_model_tf_cb)

    rospy.loginfo("tf_broadcaster-> Publishing tf for: " + MODEL_NAME)

    rospy.spin()
