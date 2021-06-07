import xbot_interface.config_options as xbot_opt
import rospy
from cartesian_interface.pyci_all import *
from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
from moveit_commander.roscpp_initializer import roscpp_initialize
# from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
from ci_solver import CartesianInterfaceSolver
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

import matplotlib.pyplot as plt

class FloatingBase:

    def __init__(self):

        # rospy.wait_for_message("/odometry/base_link/pose", PoseStamped)
        # rospy.wait_for_message("/odometry/base_link/twist", TwistStamped)

        self.fb_pose = Affine3()
        self.fb_twist = np.zeros([6,1])

        rospy.Subscriber("/odometry/base_link/pose", PoseStamped, self._fbPoseCallback)
        rospy.Subscriber("/odometry/base_link/twist", TwistStamped, self._fbTwistCallback)

        pos_msg = rospy.wait_for_message("/odometry/base_link/pose", PoseStamped)

        self.fb_pose.translation[0] = pos_msg.pose.position.x
        self.fb_pose.translation[1] = pos_msg.pose.position.y
        self.fb_pose.translation[2] = pos_msg.pose.position.z
        o = pos_msg.pose.orientation
        self.fb_pose.quaternion = [o.x, o.y, o.z, o.w]


        vel_msg = rospy.wait_for_message("/odometry/base_link/twist", TwistStamped)

        self.fb_twist[0] = vel_msg.twist.linear.x
        self.fb_twist[1] = vel_msg.twist.linear.y
        self.fb_twist[2] = vel_msg.twist.linear.z
        self.fb_twist[3] = vel_msg.twist.angular.x
        self.fb_twist[4] = vel_msg.twist.angular.y
        self.fb_twist[5] = vel_msg.twist.angular.z


    def _fbPoseCallback(self, data):

        self.fb_pose.translation[0] = data.pose.position.x
        self.fb_pose.translation[1] = data.pose.position.y
        self.fb_pose.translation[2] = data.pose.position.z

        o = data.pose.orientation
        self.fb_pose.quaternion = [o.x, o.y, o.z, o.w]


    def _fbTwistCallback(self, data):

        self.fb_twist[0] = data.twist.linear.x
        self.fb_twist[1] = data.twist.linear.y
        self.fb_twist[2] = data.twist.linear.z
        self.fb_twist[3] = data.twist.angular.x
        self.fb_twist[4] = data.twist.angular.y
        self.fb_twist[5] = data.twist.angular.z

    def getFbPose(self):
        return self.fb_pose

    def getFbTwist(self):
        return self.fb_twist

if __name__ == '__main__':

    rospy.init_node('fb_est')
    roscpp_initialize('fb_est')
    # roscpp_init('fb_est', [])

    rate = rospy.Rate(100)

    # PREPARE ROBOT
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('/xbotcore/robot_description')
    srdf = rospy.get_param('/xbotcore/robot_description_semantic')

    opt = co.ConfigOptions()
    opt.set_urdf(urdf)
    opt.set_srdf(srdf)
    opt.generate_jidmap()
    opt.set_bool_parameter('is_model_floating_base', True)
    opt.set_string_parameter('model_type', 'RBDL')
    opt.set_string_parameter('framework', 'ROS')
    model = xbot.ModelInterface(opt)
    robot = xbot.RobotInterface(opt)


    fb = FloatingBase()

    robot.sense()
    model.syncFrom(robot)
    model.setFloatingBaseState(fb.getFbPose(), fb.getFbTwist())
    model.update()


    # t_start = rospy.Time.now().to_sec()
    # t_plot = [t_start]
    # com_plot = [model.getCOM()[0]]
    threshold = 0.2
    print('starting to loop ...')
    t = 0
    while t < 5.:

        model.syncFrom(robot)
        model.setFloatingBaseState(fb.getFbPose(), fb.getFbTwist())
        model.update()

        # print(model.getCOM)
        if model.getCOMVelocity()[0] >= threshold:
            print('PORCODDIOTROPPOVELOCE')
        # t = rospy.Time.now().to_sec() - t_start

        # t_plot.append(t)
        # com_plot.append(model.getCOM()[0])


        rate.sleep()

