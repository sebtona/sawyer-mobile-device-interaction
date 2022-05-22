#!/usr/bin/env python
import rospy
import intera_interface

rospy.init_node('get_joint_angles')

limb = intera_interface.Limb('right')
angles = limb.joint_angles()
print angles