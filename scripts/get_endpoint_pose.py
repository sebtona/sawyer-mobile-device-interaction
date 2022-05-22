#!/usr/bin/env python
import rospy
import intera_interface

rospy.init_node('get_endpoint_pose')

limb = intera_interface.Limb('right')
pose = limb.endpoint_pose()
print pose