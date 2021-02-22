#/usr/bin/env python2.7

import cartesian_interface as ci
import rospy
from cartesian_interface.pyci_all import *
import time
from base_estimation.msg import ContactsStatus
from base_estimation.msg import ContactStatus
import threading

def thread_func():
    rospy.spin()

def contacts_status_cb(data, args):
    global phase

    pub = args

    contacts = data.contacts_status
    msg = ContactsStatus()
    for contact in contacts:
        c = ContactStatus()
        c.header.frame_id = contact.header.frame_id
        c.status = contact.status
        msg.contacts_status.append(c)
    if phase == 0:
        pub.publish(msg)

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
pub = rospy.Publisher('/inverse_dynamics/contacts/status', ContactsStatus, queue_size=1)
rospy.Subscriber("/odometry/contacts/status", ContactsStatus, contacts_status_cb, (pub))
x = threading.Thread(target=thread_func)
x.start()


waypoints = []
waypoints.append(pyci.WayPoint(Affine3(pos=[com_position.translation[0], l_sole_pose.translation[1], com_position.translation[2]]), 10.))

com.setWayPoints(waypoints)

raw_input("Press for rise leg")

phase = 1
waypoints = []
waypoints.append(pyci.WayPoint(Affine3(pos=[r_sole_pose.translation[0], r_sole_pose.translation[1], r_sole_pose.translation[2]+0.05]), 5.))

r_sole.setWayPoints(waypoints)

raw_input("Press for lower leg")
phase = 2
waypoints = []
waypoints.append(pyci.WayPoint(Affine3(pos=[r_sole_pose.translation[0]+0.05, r_sole_pose.translation[1], r_sole_pose.translation[2]]), 5.))
r_sole.setWayPoints(waypoints)



raw_input("Press for lower leg")


msg = ContactsStatus()
c = ContactStatus()
c.header.frame_id = 'l_foot'
c.status = True
msg.contacts_status.append(c)
c = ContactStatus()
c.header.frame_id = 'r_foot'
c.status = True
msg.contacts_status.append(c)
pub.publish(msg)


waypoints=[]
waypoints.append(pyci.WayPoint(Affine3(pos=[com_position.translation[0]+0.05, com_position.translation[1], com_position.translation[2]]), 20.))
com.setWayPoints(waypoints)
