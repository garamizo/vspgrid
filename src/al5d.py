#!/usr/bin/env python

# TODO Required installation of ros-indigo-urdfdom-py

import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import math
import scipy.optimize
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
import tf2_ros
from tf import transformations as tfs

# TODO Consider using quaternion for space representation

def matrix_from_StampedTransform(msg):
    T = msg.transform
    trans = [T.translation.x, T.translation.y, T.translation.z]
    quat = [T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]

    return tfs.concatenate_matrices(
        tfs.translation_matrix(trans),
        tfs.quaternion_matrix(quat))


class Robot(object):

    def __init__(self, sim=True):

        f = file('/home/hirolab/catkin_ws/src/vspgrid/urdf/widowx2.urdf', 'r')
        robot = URDF.from_xml_string(f.read())  # parsed URDF

        # robot = URDF.from_parameter_server()
        self.kin = KDLKinematics(robot, 'base', 'mconn')

        # x, y, z, yaw, pitch
        self.cart_from_matrix = lambda T: np.array([
            T[0, 3],
            T[1, 3],
            T[2, 3],
            math.atan2(T[1, 0], T[0, 0]),
            -math.asin(T[2, 0])])

        self.q = np.array([0, 0, 0, 0, 0])
        self.dt = 0.05
        self.sim = sim

        def publish(eventStop):

            # for arbotix
            pub1 = rospy.Publisher("q1/command", Float64, queue_size=5)
            pub2 = rospy.Publisher("q2/command", Float64, queue_size=5)
            pub3 = rospy.Publisher("q3/command", Float64, queue_size=5)
            pub4 = rospy.Publisher("q4/command", Float64, queue_size=5)
            pub5 = rospy.Publisher("q5/command", Float64, queue_size=5)

            # for visualization
            jointPub = rospy.Publisher(
                "/joint_states", JointState, queue_size=5)
            jmsg = JointState()
            jmsg.name = ['q1', 'q2', 'q3', 'q4', 'q5']

            while not rospy.is_shutdown() and not eventStop.is_set():

                jmsg.header.stamp = rospy.Time.now()
                jmsg.position = self.q
                if self.sim:
                    jointPub.publish(jmsg)

                pub1.publish(self.q[0])
                pub2.publish(self.q[1])
                pub3.publish(self.q[2])
                pub4.publish(self.q[3])
                pub5.publish(self.q[4])

                eventStop.wait(self.dt)

        rospy.init_node("robot")
        eventStop = threading.Event()
        threadJPub = threading.Thread(target=publish, args=(eventStop,))
        threadJPub.daemon = True
        threadJPub.start()

    def forward(self, q=None):
        if q is None:
            q = self.q

        return(self.kin.forward(q))

    def cart(self, q=None):
        if q is None:
            q = self.q

        return(self.cart_from_matrix(self.forward(q)))

    def jacobian(self, q=None):
        if q is None:
            q = self.q

        eps = 1e-3
        s0 = np.array(self.cart(q))
        dof = len(self.q)
        J = np.array(dof * [dof * [0.0]])
        for k in range(0, dof):
            dq = np.zeros(dof)
            dq[k] = eps

            J[:, k] = (self.cart(q + dq) - s0) / eps

        return J

    def move(self, s1):

        def plan(s1):
            # return tuples of joint coordinates to be moved at every self.dt
            # first calculate goal_joint
            def error_function(q): return (
                np.array(self.cart(q)) - np.array(s1))

            x, _, ler, message = scipy.optimize.fsolve(
                error_function, 0*self.q, full_output=True)

            if not ler == 1:
                # print "No solution found. Target: ", goal_space
                raise Exception('No IK solution found')

            q1 = (x + math.pi) % (2 * math.pi) - math.pi
            # q1 = x

            # break trajectory to intermediate points
            q0 = self.q
            s0 = self.cart(q0)
            speed = 1

            t1 = np.max(np.abs(q1 - q0)) / speed
            time = np.arange(0, t1, self.dt) + self.dt

            dof = len(self.q)
            qq = np.array([np.interp(time, [0, t1], [q0[k], q1[k]])
                           for k in range(dof)]).transpose()

            return qq

        traj = plan(s1)
        rate = rospy.Rate(1.0 / self.dt)

        for joints in traj:

            self.q = joints
            rate.sleep()

    def move2(self, s1):

        speed = 1.5
        TOL = 1e-8
        MAX_VEL = 0.2
        t0 = rospy.Time.now()
        while rospy.Time.now() - t0 < rospy.Duration(5):

            ds = s1 - self.cart()
            dev = np.linalg.norm(ds)
            if dev < TOL:
                print("Success. Deviation: ")
                print(dev)
                return

            J = self.jacobian()
            dq = np.linalg.solve(J, ds)
            if max(abs(dq)) > MAX_VEL:
                dq = dq * MAX_VEL / max(abs(dq))

            alfa = speed * self.dt * (rospy.Time.now() - t0).to_sec()**1.5

            self.q = self.q + dq * alfa
            rospy.sleep(self.dt)

        print("Failure. Deviation: ")
        print(dev)

    def reset(self):
        self.move(self.cart(self.q*0))


class VisualRobot(Robot):

    def __init__(self, sim=True):
        super(VisualRobot, self).__init__(sim)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def visual_move(self, target, Ki, Kd):

        speed = 1.5
        TOL = 1e-8
        MAX_VEL = 0.5
        t0 = rospy.Time.now()
        dev = float('nan')
        integ = 0*self.q
        pasterror = None
        while rospy.Time.now() - t0 < rospy.Duration(5):

            try:
                msg = self.tf_buffer.lookup_transform(
                    'base', target, rospy.Time(0), rospy.Duration(self.dt))

                s1 = self.cart_from_matrix(matrix_from_StampedTransform(msg))

                ds = s1 - self.cart()
                dev = np.linalg.norm(ds)
                if dev < TOL:
                    print("Success. Deviation: ")
                    print(dev)
                    return

                J = self.jacobian()
                dq = np.linalg.solve(J, ds)
                if max(abs(dq)) > MAX_VEL:
                    dq = dq * MAX_VEL / max(abs(dq))

                alfa = speed * self.dt * (rospy.Time.now() - t0).to_sec()**1.5

                if pasterror is None:
                    pasterror = dq
                deriv = (dq - pasterror)/self.dt
                pasterror = dq
                integ = integ + dq*self.dt

                self.q = self.q + dq * alfa + integ*Ki + deriv*Kd
                
                print(dq*alfa)
                rospy.sleep(self.dt)

            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

        print("Failure. Deviation: ")
        print(dev)

    def scan(self, target, sweep=np.arange(-math.pi/2, math.pi/2, 1e-2)):

        # center
        self.move([0.15*math.cos(sweep[0]), 0.15*math.sin(sweep[0]), 0.2, sweep[0], 0])

        dq = 1e-2
        eps = math.pi/12
        rate = rospy.Rate(1/self.dt)
        for self.q[0] in sweep:

            try:
                msg = self.tf_buffer.lookup_transform(
                    'mconn', target, rospy.Time(0), rospy.Duration(0))

                T = matrix_from_StampedTransform(msg)
                if abs(math.atan2(T[1,3], T[0,3])) < eps:
                    return True

            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            if rospy.is_shutdown():
                break
            rate.sleep()

        return False



