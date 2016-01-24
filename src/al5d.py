#!/usr/bin/env python

# TODO Required installation of ros-indigo-urdfdom-py

import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import math

# TODO Consider using quaternion for space representation


class Robot(object):
    def __init__(self):
        robot = URDF.from_parameter_server()
        self.kin = KDLKinematics(robot, 'base', 'mconn')

        # x, y, z, yaw, pitch
        self.cart_from_matrix = lambda T: np.array([
            T[0, 3],
            T[1, 3],
            T[2, 3],
            math.atan2(T[1, 0], T[0, 0]),
            math.asin(-T[0, 2])])

        self.q = [0, 0, 0, 0, 0]

    def forward(self, q):
        return(self.kin.forward(q))

# TODO Start using a finite difference Jacobian. Compare performance later
    def jacobian(self, q):
        return(self.kin.jacobian(q))

    def cart(self):
        return(self.cart_from_matrix(self.forward(self.q)))

    def set_joints(self, q):




# f = file('/home/hirolab/catkin_ws/src/vspgrid/urdf/widowx2.urdf', 'r')
# robot = URDF.from_xml_string(f.read())  # parsed URDF
# base_link = 'base'
# end_link = 'cam'
# # robot = URDF.from_parameter_server()
# kdl_kin = KDLKinematics(robot, base_link, end_link)
# print(kdl_kin)

# q = np.array([30 * np.pi / 180, 0, 0, 0, 0])
# # forward kinematics (returns homogeneous 4x4 numpy.mat)
# pose = kdl_kin.forward(q)
# q_ik = kdl_kin.inverse(pose, q + 0.3)  # inverse kinematics
# if q_ik is not None:
#     pose_sol = kdl_kin.forward(q_ik)  # should equal pose
#     J = kdl_kin.jacobian([0, 0, 0, 0])
#     print 'q:', q
#     print 'q_ik:', q_ik
#     print 'pose:', pose
#     if q_ik is not None:
#         print 'pose_sol:', pose_sol
#         print 'J:', J

r = Robot()

print(r.cart())
