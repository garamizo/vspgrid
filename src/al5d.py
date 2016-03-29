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
from scipy.optimize import minimize

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

        # Selection of end-effector parameters for WidowX
        # x, y, z, yaw, pitch
        self.cart_from_matrix = lambda T: np.array([
            T[0, 3],
            T[1, 3],
            T[2, 3],
            math.atan2(T[1, 0], T[0, 0]),
            -math.asin(T[2, 0])])

        self.home = np.array([0.05, 0, 0.25, 0, 0])     # home end-effector pose

        self.q = np.array([0, 0, 0, 0, 0])              # joint angles
        self.dt = 0.05                                  # algorithm time step
        self.sim = sim                                  # true if virtual robot

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
        threadJPub.start()  # Update joint angles in a background process

    def act_gripper(self, grip):
    # Actuate gripper given gripper position
        pub = rospy.Publisher('q6/command', Float64, queue_size=5)
        rospy.sleep(1)
        pub.publish(grip)
        rospy.sleep(1)

    def forward(self, q=None):
    # End-effector transformation matrix, given joint angles
        if q is None:
            q = self.q

        return(self.kin.forward(q))

    def cart(self, q=None):
    # Cartesian coordinates [x, y, z, r1, r2, r3], given joint angles
        if q is None:
            q = self.q

        return(self.cart_from_matrix(self.forward(q)))

    def jacobian(self, q=None):
    # Inertial jacobian
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

    def move(self, s1, speed=1):

        def plan(s1):
            # return tuples of joint coordinates to be moved at every time step

            # Error function to find the zero
            def error_function(q): return (
                np.array(self.cart(q)) - np.array(s1))

            x, _, ler, message = scipy.optimize.fsolve(
                error_function, 0*self.q, full_output=True)

            if not ler == 1:

                print('No IK solution found')
                q1 = self.q
            else:
                q1 = (x + math.pi) % (2 * math.pi) - math.pi
            # q1 = x

            # break trajectory to intermediate points
            q0 = self.q         # initial joint angles
            s0 = self.cart(q0)  # initial end-effector pose

            t1 = np.max(np.abs(q1 - q0)) / speed        # time to reach goal
            time = np.arange(0, t1, self.dt) + self.dt  # time array

            dof = len(self.q)
            # Trajectory matrix
            qq = np.array([np.interp(time, [0, t1], [q0[k], q1[k]])
                           for k in range(dof)]).transpose()

            return qq

        traj = plan(s1)
        rate = rospy.Rate(1.0 / self.dt)

        for joints in traj:

            self.q = joints
            rate.sleep()
        

    def move2(self, s1):
    # Not used

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
        self.move(self.home)

    def find(self, target):
    # Return transformation matrix of the target in respect to robot base

        try:
            msg = self.tf_buffer.lookup_transform(
                'base', target, rospy.Time(0), rospy.Duration(0))

            T = matrix_from_StampedTransform(msg)
            return T, True

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return np.identity(4), False


    def visual_move(self, target, speed=0.5, duration=5):
    # Move end-effector to target using visual servoing

        # PID parameters
        Kp = 0.03
        Ki = 0.01*0
        Kd = 0.1*0
        dt = self.dt
        integ = 0*self.q
        pasterror = None

        S1 = None               # Averaged end-effector pose
        TOL = 1e-8              # Pose tolerance [m] and [rad]
        MAX_VEL = 0.5           # Maximum joint speed [rad/s]
        t0 = rospy.Time.now()   # Initial time
        
        while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(duration):

            # Get target instantaneous pose
            T, success = self.find(target)
            s1 = self.cart_from_matrix(T)
            if not success: continue

            # exponential average the target pose
            if S1 is None:
                S1 = s1
            else:
                S1 = 0.9*S1 + 0.1*s1

            error = S1 - self.cart()        # pose error vector
            dev = np.linalg.norm(error)     # pose error scalar

            if dev < TOL:
                print("Success. Deviation: ", dev)
                return

            # apply PID control
            if pasterror is None:
                pasterror = error

            deriv = (error - pasterror)/dt
            pasterror = error
            integ = integ + error*dt

            ds = Kp*error + Ki*integ + Kd*deriv     # pose increment

            J = self.jacobian()                     
            dq = np.linalg.solve(J, ds)             # convert pose to joint angle increment

            # limit joint speed
            if max(abs(dq)) > MAX_VEL:
                dq = dq * MAX_VEL / max(abs(dq))

            self.q = self.q + dq                    # actuate joints

            rospy.sleep(dt)


    def scan(self, target, sweep_range=math.pi/2):
    # Scan for target and stop when facing it

        # start from center, go left, then right, back to center
        sweep=np.arange(0, sweep_range, 3e-2)
        sweep = np.append(sweep, np.flipud(sweep)) 
        sweep = np.append(sweep, -sweep)

        # center 
        self.move([0.05*math.cos(sweep[0]), 0.05*math.sin(sweep[0]), 0.25, sweep[0], 0])

        EPS = math.pi/12
        rate = rospy.Rate(1/self.dt)
        for self.q[0] in sweep:

            rate.sleep()                        # settle vision

            Tf, success = self.find(target)     # female pose in base frame
            if not success: continue
            Tm = self.forward(self.q)           # male pose in base frame

            Tmf = np.linalg.inv(Tm) * Tf                    # female pose in male frame

            if abs(math.atan2(Tmf[1,3], Tmf[0,3])) < EPS:   # female is in front of connector
                return True

            if rospy.is_shutdown():
                break

        return False


    def connect(self):
    # Main connection routine. Scan for female connector and approach the target using

        self.act_gripper(0.5) # close

        self.move(self.home)

        success = self.scan('fconn')
        if not success: return

        self.visual_move('fconn_')
        self.visual_move('fconn')
        self.act_gripper(2) # open
        rospy.sleep(1)

        # s0 = self.find('fconn')
        # th0 = s0[3]
        # ps0 = s0[4]
        # tend = 3
        # for time in np.linspace(0, tend, tend*100):
        #     dth = 0.1*np.sin(2*np.pi*time/tend)
        #     dps = 0.1*np.cos(2*np.pi*time/tend)
        #     s = s0
        #     s[3] = th0 + dth
        #     s[4] = ps0 + dps
        #     self.move(s)
        #     rospy.sleep(0.01)

        self.visual_move('fconnp', 1, 3)
        self.visual_move('fconnp2', 1, 3)
        self.visual_move('fconn', 1, 2)
        self.visual_move('fconn_')
        self.move(self.home)
