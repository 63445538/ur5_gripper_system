#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from rg6_gripper_bringup.msg import Gripper
from copy import deepcopy
import message_filters
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

class FullState:
    def __init__(self):
        rospy.init_node('joint_state_republisher', anonymous=True)
        gripper_ns = rospy.get_param("~gripper_ns", "/gripper")
        joint_states_sub = message_filters.Subscriber('/joint_states', JointState)
        gripper_state_sub = message_filters.Subscriber(gripper_ns, Gripper)
        self._ts = message_filters.ApproximateTimeSynchronizer([joint_states_sub, gripper_state_sub], 10, 0.1, allow_headerless=True)
        self._ts.registerCallback(self._joint_states_callback)
        self._joint_states_pub = rospy.Publisher('/joint_states_full', JointState, queue_size=1)
        rospy.spin()

    def _joint_states_callback(self, ur5_joints, gripper_joint):
        msg = JointState()
        msg.header = ur5_joints.header
        msg.name = list(ur5_joints.name)
        msg.name.append('finger_joint')
        msg.position = list(ur5_joints.position)
        msg.position.append((1.0-(gripper_joint.width/140.0))*0.7)
        msg.velocity = list(ur5_joints.velocity)
        msg.velocity.append(0.0)
        msg.effort = list(ur5_joints.effort)
        msg.effort.append(0.0)
        # print msg
        self._joint_states_pub.publish(msg)

        
if __name__ == '__main__':
    FullState()
