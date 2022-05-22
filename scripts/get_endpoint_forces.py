#!/usr/bin/env python
import rospy
import intera_interface

rospy.init_node('get_endpoint_force')

limb = intera_interface.Limb('right')
force = limb.endpoint_effort()['force'].z
print force