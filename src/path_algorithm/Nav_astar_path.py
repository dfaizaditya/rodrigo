#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from rodrigo.srv import MakePath, RobotNav


class Nav_astar_path:

    def __init__(self):
        rospy.init_node('nav_astar_path', log_level=rospy.DEBUG)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        # Setup service proxys
        self.make_path = rospy.ServiceProxy('make_path', MakePath)
        self.robot_nav = rospy.ServiceProxy('robot_nav', RobotNav)

        self.goal = PoseStamped()

        self.pose = None

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        new_pose = PoseStamped()
        new_pose.pose.position = position
        self.pose = new_pose

    def goal_callback(self, goal):
        if self.pose is None:
            rospy.logwarn("No known pose!")
            return

        self.goal = goal

        # path_response = self.make_path(self.pose, self.goal)
        path_response = self.make_path(self.pose, goal)
        poses = path_response.path.poses
        rospy.logdebug("Response was %s" % path_response)
        rospy.logdebug("Response path was %s" % path_response.path)

        # Tell robot to drive to each point
        for pose in poses[1:-1]:
            rospy.logdebug("Sending pose %s" % pose)
            if not self.robot_nav(pose, True):
                rospy.logwarn("Robot navigation failed")
                return
            rospy.logdebug("At point %s" % self.pose)
        self.robot_nav(goal, False)


if __name__ == '__main__':
    nav_astar_path = Nav_astar_path()

    rospy.loginfo("Initializing Nav_astar_path")

    while not rospy.is_shutdown():
        pass

    rospy.spin()
    pass
