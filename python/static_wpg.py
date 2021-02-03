#/usr/bin/env python2.7

import cartesian_interface as ci
import rospy
from cartesian_interface.pyci_all import *
import time

rospy.init_node('static_wpg', anonymous=True)

ciros = pyci.CartesianInterfaceRos()

com = ciros.getTask("com")
com_position, _, _ = com.getPoseReference()

l_sole = ciros.getTask("l_sole")
l_sole_pose, _, _ = l_sole.getPoseReference()

r_sole = ciros.getTask("r_sole")
r_sole_pose, _, _ = r_sole.getPoseReference()

print "com: ", com_position.translation
print "l_sole: ", l_sole_pose.translation
print "r_sole: ", r_sole_pose.translation

waypoints = []
waypoints.append(pyci.WayPoint(Affine3(pos=[com_position.translation[0], l_sole_pose.translation[1], com_position.translation[2]]), 10.))

com.setWayPoints(waypoints)

raw_input("Press for rise leg")

waypoints = []
waypoints.append(pyci.WayPoint(Affine3(pos=[r_sole_pose.translation[0], r_sole_pose.translation[1], r_sole_pose.translation[2]+0.1]), 10.))

r_sole.setWayPoints(waypoints)

raw_input("Press for lower leg")
waypoints = []
waypoints.append(pyci.WayPoint(Affine3(pos=[r_sole_pose.translation[0]+0.05, r_sole_pose.translation[1]-0.05, r_sole_pose.translation[2]]), 10.))
r_sole.setWayPoints(waypoints)
waypoints=[]
waypoints.append(pyci.WayPoint(Affine3(pos=[com_position.translation[0]+0.05, com_position.translation[1], com_position.translation[2]]), 20.))
com.setWayPoints(waypoints)
