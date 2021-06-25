#!/usr/bin/env python3

from rodrigo.srv import MoveToPoint, MoveToPointResponse
from rodrigo.srv import MoveToPose, MoveToPoseResponse
from rodrigo.srv import StopMoving, StopMovingResponse
from rodrigo.srv import IsMoving, IsMovingResponse

import rospy
import threading
import yaml
import os
import sys
from turtle_lib import Turtle
from utils.commons import read_yaml_file


ROOT = os.path.dirname(os.path.abspath(__file__))+"/"
CONFIG_FILEPATH = ROOT + "config.yaml"
NODE_NAME = 'run_turtlebot_control_server'
SRV_NAMESPACE, turtle = None, None  # To be initialized later.

# ============================================================================== #


def _srv_callback_wrapper(callback_func):
    ''' Print messages before and after the callback function. '''

    def new_callback_func(self, req):
        '''Argument: `req` is the input of the ROS service call. '''
        rospy.loginfo("Service: " + self._srv_name +
                      ": Receive request: {}".format(req))
        response = callback_func(self, req)
        rospy.loginfo("Service: " + self._srv_name +
                      ": Request has been sent to turtlebot_lib.py!")
        return response
    return new_callback_func


class _SrvTemplate(object):
    ''' A template for creating ROS service. '''

    def __init__(self, srv_name,
                 srv_in_type,
                 srv_out_type):

        if SRV_NAMESPACE:
            srv_name = SRV_NAMESPACE + "/" + srv_name  # Add name space

        self._srv = rospy.Service(
            srv_name, srv_in_type, self._callback)
        rospy.loginfo("  Service starts: " + srv_name)
        self._srv_name = srv_name
        self._srv_in_type = srv_in_type
        self._srv_out_type = srv_out_type

    def _callback(self, req):
        raise NotImplementedError("Please overload this function!")

# ============================================================================== #


class RodrigoRosServices(object):
    def __init__(self):
        self._is_start = False

    def start(self):
        self._h1 = RodrigoRosServices.ServiceMoveToPoint()
        self._h2 = RodrigoRosServices.ServiceMoveToPose()
        self._h3 = RodrigoRosServices.ServiceBerhenti()
        self._h4 = RodrigoRosServices.ServiceIsMoving()
        self._is_start = True

    def __del__(self):
        if self._is_start:
            turtle.stop_moving()
    
    class ServiceMoveToPoint(_SrvTemplate):
        def __init__(self):
            super(RodrigoRosServices.ServiceMoveToPoint, self).__init__(
                srv_name='move_to_point',
                srv_in_type=MoveToPoint,
                srv_out_type=MoveToPointResponse,
            )

        def _callback(self, req):
            turtle.move_to_pose(x_goal_w=req.x,
                                y_goal_w=req.y)
            return self._srv_out_type()


    class ServiceMoveToPose(_SrvTemplate):
        def __init__(self):
            super(RodrigoRosServices.ServiceMoveToPose, self).__init__(
                srv_name='move_to_pose',
                srv_in_type=MoveToPose,
                srv_out_type=MoveToPoseResponse,
            )

        @_srv_callback_wrapper
        def _callback(self, req):
            turtle.move_to_pose(x_goal_w=req.x,
                                y_goal_w=req.y,
                                theta_goal_w=req.theta)
            return self._srv_out_type()

    class ServiceBerhenti(_SrvTemplate):
        def __init__(self):
            super(RodrigoRosServices.ServiceBerhenti, self).__init__(
                srv_name='berhenti',
                srv_in_type=StopMoving,
                srv_out_type=StopMovingResponse,
            )

        @_srv_callback_wrapper
        def _callback(self, req):
            turtle.stop_moving()
            return self._srv_out_type()


    class ServiceIsMoving(_SrvTemplate):
        def __init__(self):
            super(RodrigoRosServices.ServiceIsMoving, self).__init__(
                srv_name='is_moving',
                srv_in_type=IsMoving,
                srv_out_type=IsMovingResponse,
            )

        def _callback(self, req):
            is_moving = turtle.is_moving()
            return IsMovingResponse(is_moving)


def main():

    rospy.init_node(NODE_NAME)
    rospy.loginfo("Node starts: " + NODE_NAME)

    global turtle, SRV_NAMESPACE
    turtle = Turtle(CONFIG_FILEPATH)
    SRV_NAMESPACE = read_yaml_file(CONFIG_FILEPATH)["srv_namespace"]

    rospy.on_shutdown(lambda: turtle.stop_moving())

    # Start ROS services.
    turtle_services = RodrigoRosServices()
    turtle_services.start()

    rospy.spin()
    rospy.loginfo("Node stops: " + NODE_NAME)


if __name__ == "__main__":
    main()
