#!/usr/bin/env python

import rospy
import sys
import tf2_ros
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench


class FakeDisturbanceManager:
    def __init__(self):
        pass

    def init(self):
        if not rospy.has_param('~world_frame_id'):
            return False
        self._world_frame_id = rospy.get_param('~world_frame_id')

        if not rospy.has_param('~body_frame_id'):
            return False
        self._body_frame_id = rospy.get_param('~body_frame_id')

        if not rospy.has_param('~topics/vis'):
            return False
        self._vis_pub = rospy.Publisher(
            rospy.get_param('~topics/vis'), Marker, queue_size=25)

        if not rospy.has_param('~gazebo_ns'):
            return False
        gazebo_ns = rospy.get_param('~gazebo_ns')

        rospy.loginfo("[FakeDisturbanceManager] Waiting for serice {}...".format(
            gazebo_ns + "/apply_body_wrench"))
        rospy.wait_for_service(gazebo_ns + "/apply_body_wrench")
        self._apply_body_wrench = rospy.ServiceProxy(
            gazebo_ns + "/apply_body_wrench", ApplyBodyWrench)

        if not rospy.has_param('~disturbances'):
            return False
        self._disturbances = rospy.get_param('~disturbances')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        return True

    def show_disturbances(self):
        for i, d in enumerate(self._disturbances):
            region_msg = Marker()
            region_msg.header.frame_id = self._world_frame_id
            region_msg.ns = "/fake_disturbance_manager/" + str(i)
            region_msg.id = 0
            region_msg.type = Marker.CUBE
            region_msg.pose.position.x = d['region']['center']['x']
            region_msg.pose.position.y = d['region']['center']['y']
            region_msg.pose.position.z = -0.674614  # TODO(eratner) Fix this
            region_msg.pose.orientation.x = 0
            region_msg.pose.orientation.y = 0
            region_msg.pose.orientation.z = 0
            region_msg.pose.orientation.w = 1
            region_msg.scale.x = d['region']['size']['x']
            region_msg.scale.y = d['region']['size']['y']
            region_msg.scale.z = 0.5
            region_msg.color.r = 1.0
            region_msg.color.g = 1.0
            region_msg.color.b = 0
            region_msg.color.a = 0.25
            self._vis_pub.publish(region_msg)

            dir_msg = Marker()
            dir_msg.header.frame_id = self._world_frame_id
            dir_msg.ns = "/fake_disturbance_manager/" + str(i)
            dir_msg.id = 1
            dir_msg.type = Marker.ARROW
            dir_msg.pose.position.x = d['region']['center']['x']
            dir_msg.pose.position.y = d['region']['center']['y']
            dir_msg.pose.position.z = -0.674614  # TODO(eratner) Fix this
            dir_msg.pose.orientation.x = 0
            dir_msg.pose.orientation.y = 0
            dir_msg.pose.orientation.z = 0
            dir_msg.pose.orientation.w = 1
            dir_msg.scale.x = 0.1
            dir_msg.scale.y = 0.15
            dir_msg.color.r = 1.0
            dir_msg.color.g = 1.0
            dir_msg.color.b = 0
            dir_msg.color.a = 0.5
            dir_msg.points = [Point(0, 0, 0),
                          Point(d['force']['x'], d['force']['y'], 0)]
            self._vis_pub.publish(dir_msg)

    def run(self):
        freq = 10.0
        rate = rospy.Rate(freq)

        while not rospy.is_shutdown():
            try:
                robot_pose = self._tf_buffer.lookup_transform(
                    self._world_frame_id, self._body_frame_id, rospy.Time())

                for d in self._disturbances:
                    x_rel = robot_pose.transform.translation.x - \
                        d['region']['center']['x']
                    y_rel = robot_pose.transform.translation.y - \
                        d['region']['center']['y']

                    # Does the region contain the robot's current position?
                    if abs(x_rel) < 0.5 * d['region']['size']['x'] and \
                       abs(y_rel) < 0.5 * d['region']['size']['y']:
                        w = Wrench()
                        w.force.x = d['force']['x']
                        w.force.y = d['force']['y']
                        w.force.z = 0
                        w.torque.x = 0
                        w.torque.y = 0
                        w.torque.z = 0
                        try:
                            res = self._apply_body_wrench(
                                'bsharp::body',
                                '',
                                Point(0, 0, 0),
                                w,
                                rospy.Time(),
                                rospy.Duration(1.0 / freq))
                            rospy.loginfo("[FakeDisturbanceManager] Applying force ({}, {}) succeeded? {} with message: {}".format(
                                d['force']['x'],
                                d['force']['y'],
                                str(res.success),
                                res.status_message))
                        except rospy.ServiceException as e:
                            rospy.logerr(
                                "[FakeDisturbanceManager] Error: {}".format(e))
            except tf2_ros.TransformException as e:
                rospy.logerr("[FakeDisturbanceManager] Error: {}".format(e))

            self.show_disturbances()
            rate.sleep()


def main():
    rospy.init_node('fake_disturbance_manager')

    mgr = FakeDisturbanceManager()
    if not mgr.init():
        rospy.logerr("Error: failed to initialize!")
        sys.exit(-1)

    mgr.run()


if __name__ == '__main__':
    main()
