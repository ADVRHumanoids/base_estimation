#/usr/bin/env python2.7

import cartesian_interface as ci
import rospy
from cartesian_interface.pyci_all import *
import time
from base_estimation.msg import ContactsStatus
import threading

def thread_func():
    rospy.spin()

def contacts_status_cb(data):
    global phase

    contacts = data.contacts_status
    for contact in contacts:
        print contact.header.frame_id, ": ", contact.status
        if phase == 0:
            #CHANGE CONTACTS STATUS TO ID ONLY IN PHASE0

rospy.init_node('static_wpg', anonymous=True)

ciros = pyci.CartesianInterfaceRos()

com = ciros.getTask("com")
com_position, _, _ = com.getPoseReference()

l_sole = ciros.getTask("l_foot")
l_sole_pose, _, _ = l_sole.getPoseReference()

r_sole = ciros.getTask("r_foot")
r_sole_pose, _, _ = r_sole.getPoseReference()

print "com: ", com_position.translation
print "l_foot: ", l_sole_pose.translation
print "r_foot: ", r_sole_pose.translation

phase = 0
rospy.Subscriber("/odometry/contacts/status", ContactsStatus, contacts_status_cb)
x = threading.Thread(target=thread_func)
x.start()


waypoints = []
waypoints.append(pyci.WayPoint(Affine3(pos=[com_position.translation[0], l_sole_pose.translation[1], com_position.translation[2]]), 10.))

com.setWayPoints(waypoints)

raw_input("Press for rise leg")

phase = 1
waypoints = []
waypoints.append(pyci.WayPoint(Affine3(pos=[r_sole_pose.translation[0], r_sole_pose.translation[1], r_sole_pose.translation[2]+0.1]), 10.))

r_sole.setWayPoints(waypoints)

raw_input("Press for lower leg")
phase = 2
waypoints = []
waypoints.append(pyci.WayPoint(Affine3(pos=[r_sole_pose.translation[0]+0.05, r_sole_pose.translation[1]-0.05, r_sole_pose.translation[2]]), 10.))
r_sole.setWayPoints(waypoints)
waypoints=[]
waypoints.append(pyci.WayPoint(Affine3(pos=[com_position.translation[0]+0.05, com_position.translation[1], com_position.translation[2]]), 20.))
com.setWayPoints(waypoints)
