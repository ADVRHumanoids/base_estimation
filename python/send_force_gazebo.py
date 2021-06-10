import rospy
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench


def sendForceGazebo(mag, duration):

    rospy.wait_for_service('gazebo/apply_body_wrench')
    try:
        apply_wrench = rospy.ServiceProxy('gazebo/apply_body_wrench', ApplyBodyWrench)
        body_name = 'cogimon::base_link'
        reference_frame = ''

        wrench = Wrench()
        wrench.force.x = mag[0]
        wrench.force.y = mag[1]
        wrench.force.z = mag[2]

        duration = rospy.Duration(duration)

        apply_wrench(body_name=body_name, reference_frame=reference_frame, wrench=wrench, duration=duration)
        return 1

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)