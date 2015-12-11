#!/usr/bin/env python
import rospy
from tf import transformations as tfs
import numpy
import math
import robot
from transitions import Machine
import tf2_ros
import threading

import tf


class SeeingRobot(robot.Robot):

    states = ['resting', 'searching', 'gazing',
              'loading', 'connecting', 'unloading']

    def __init__(self):

        super(SeeingRobot, self).__init__()
        self.joints = numpy.array([0, -0.3, 0, 0, 0])
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def entry(self):
        print 'Enter ', self.state

    def exit(self):
        print 'Exit ', self.state

    def recover(self):
        super(SeeingRobot, self).recover()
        self.machine.set_state('resting')

    def get_transform(self, child, parent, when, timeout):

        def matrix_from_StampedTransform(msg):
            T = msg.transform
            trans = [T.translation.x, T.translation.y, T.translation.z]
            quat = [T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]

            return tfs.concatenate_matrices(
                tfs.translation_matrix(trans),
                tfs.quaternion_matrix(quat))

        try:
            msg = self.tf_buffer.lookup_transform(parent, child, when, timeout)
            pose = self.space_from_matrix(matrix_from_StampedTransform(msg))
            return pose, True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        except:
            raise
            return

        return None, False

    def track(self, target, stop_event):

        exp_factor = 0.1
        while not rospy.is_shutdown() and not stop_event.is_set():
            pose, success = self.get_transform(
                target, "base", rospy.Time.now() - rospy.Duration(0.1), rospy.Duration(0.1))
            if success:
                try:
                    self.target_pose = exp_factor * pose + \
                        (1 - exp_factor) * self.target_pose
                except AttributeError:
                    self.target_pose = pose
                except:
                    raise
                    return
            stop_event.wait(self.dt)

    def robust_move(self, target):

        # thread to average the pose in the background
        t_stop = threading.Event()
        t = threading.Thread(target=self.track, args=(target, t_stop))
        t.start()

        was_found = False
        while not rospy.is_shutdown():
            pose, on_sight = self.get_transform(
                target, "base", rospy.Time.now() - rospy.Duration(0.1), rospy.Duration(0))
            if on_sight:
                next_pose = pose
            else:
                try:
                    next_pose = self.target_pose
                except (AttributeError, NameError):  # never saw it
                    raise
                    was_found = False
                    break

            plan, reachable = self.plan(next_pose, 0.1)
            if reachable:
                print "Still on sight. Moving to it."
                self.move(plan)
                print "Plan length: ", len(plan)
                if len(plan) <= 1:  # nothing will change ever again
                    was_found = True
                    del self.target_pose
                    break

        t_stop.set()
        rospy.sleep(0.2)  # wait for thread to close
        return was_found

    def robust_move2(self, target):

        tinit = rospy.Time.now()
        while not rospy.is_shutdown():

            pose_s, ever_on_sight = self.get_transform(
                target, "base", rospy.Time(0), rospy.Duration(self.dt))
            if not ever_on_sight:
                return False

            # compute average
            count = 1
            time0 = rospy.Time.now() - rospy.Duration(0.2)
            for k in range(0, 20):
                pose, on_sight = self.get_transform(
                    target, "base", time0 - rospy.Duration(0.05 * k), rospy.Duration(0))
                if on_sight:
                    pose_s = pose_s + pose
                    count = count + 1
            target_pose = pose_s / count

            # move to inbetween point
            next_pose = 0.3 * target_pose + 0.7 * \
                self.space_coordinates(self.joints)
            half_plan, reachable = self.plan(next_pose, 0.1)
            if reachable:
                self.move(half_plan)

            error = numpy.linalg.norm(
                target_pose - self.space_coordinates(self.joints))
            if error < 0.0001:
                return True

            if rospy.Time.now() - tinit > rospy.Duration(30):
                return False

        return False

    def robust_move3(self, target_id):
        tinit = rospy.Time.now()
        while not rospy.is_shutdown():

            pose, on_sight = self.get_transform(
                target_id, "base", rospy.Time(0), rospy.Duration(0.1))
            if not on_sight:
                return False

            pose_error = pose - self.space_coordinates(self.joints)
            factor_p = 0.1
            next_pose = self.space_coordinates(
                self.joints) + factor_p * pose_error

            plan, reachable = self.plan(next_pose, 0.1)
            if reachable:
                self.move(plan)

            if numpy.linalg.norm(pose_error) < 0.02:
                return True

            if rospy.Time.now() - tinit > rospy.Duration(30):
                return False

        return False





    def robust_move4(self, target_id):

        br = tf.TransformBroadcaster()
        current_pose = self.space_coordinates(self.joints)
        t0 = rospy.Time.now()
        tol = 3e-3
        timeout = 20.0
        duration = 0
        ratio = 0.3
        while not rospy.is_shutdown() and duration < timeout:

            duration = (rospy.Time.now() - t0).to_sec()

            tmp, found = self.get_transform(target_id, "base", rospy.Time(0), rospy.Duration(0))
            if not found:
                print "Not found"
                continue
            try:
                target_pose = ratio*tmp + (1-ratio)*target_pose
            except (UnboundLocalError):
                target_pose = tmp
                continue

            next_pose = current_pose + (target_pose - current_pose)*duration/timeout

            br.sendTransform((next_pose[0], next_pose[1], next_pose[2]), 
                tf.transformations.quaternion_from_euler(next_pose[3], next_pose[4], 0), 
                rospy.Time.now(), "track %s" % target_id, "base")
            plan, reachable = self.plan(next_pose, 0.2)
            if reachable:
                self.move(plan)
            else:
                print "Not reachable for ", next_pose, ". Trying approximation... "
                plan, reachable, dist = self.plan_approx(next_pose, 0.2)
                if not reachable or dist > 1e-1:
                    print "Solution far off: ", dist
                else:
                    print plan
                    self.move(plan)

            current_pose = self.space_coordinates(self.joints)
            dist = numpy.linalg.norm(target_pose - current_pose)
            if dist < tol:
                print dist, target_pose, current_pose
                return True

        print "----------------------------"
        return False


def raise_exception():
    raise rospy.ROSInterruptException('Interrupted')



def main():

    # rospy.on_shutdown(raise_exception)

    try:
        r = SeeingRobot()
        move_func = r.robust_move4

        print "Setting up..."
        # get searching poses
        rospy.sleep(2)
        pose1, success = r.get_transform("search1", "base", rospy.Time(0), rospy.Duration(1))
        pose2, success = r.get_transform("search2", "base", rospy.Time(0), rospy.Duration(1))
        pose3, success = r.get_transform("search3", "base", rospy.Time(0), rospy.Duration(1))
        search_poses = [pose1, pose2, pose1, pose3]
        idx_poses = [1, 2, 1, 3]

        print("Scanning for connector...")

        pose_idx = 0
        near_female = False
        while not near_female and not rospy.is_shutdown():
            pose_idx = (pose_idx + 1) % len(search_poses)
            plan, success = r.plan(search_poses[pose_idx], 0.2)
            if success:
                print "Moving to search pose #", idx_poses[pose_idx], "..."
                r.move(plan)

            rospy.sleep(1)
            near_female = move_func("_female")
        rospy.sleep(2)

        print "Performing final connection..."
        if move_func("female"):
            print "Connector in place!"
            rospy.sleep(1)

            print "Stepping back..."
            poseb, success = r.get_transform("_female", "base", rospy.Time(0), rospy.Duration(1))
            plan, success = r.plan(poseb, 0.2)
            if success:
                r.move(plan)
            else:
                print "Can't step back. Unsecure return..."

            plan, success = r.plan(pose1, 0.3)
            r.move(plan)

        else:
            print "Can't connect!"


    except (rospy.ROSInterruptException, KeyboardInterrupt):
        return

    print 'Finished.'
    return

if __name__ == '__main__':
    main()
