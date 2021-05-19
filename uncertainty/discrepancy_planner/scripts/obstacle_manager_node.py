#!/usr/bin/env python

import rospy
import sys
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose


class ObstacleManager:
    def __init__(self):
        self._obstacles = []

    def init(self):
        if not rospy.has_param('~world_frame_id'):
            return False
        self._world_frame_id = rospy.get_param('~world_frame_id')

        if not rospy.has_param('~topics/vis'):
            return False
        self._vis_pub = rospy.Publisher(
            rospy.get_param('~topics/vis'), Marker, queue_size=25)

        if not rospy.has_param('~gazebo_ns'):
            return False
        gazebo_ns = rospy.get_param('~gazebo_ns')

        rospy.loginfo("[ObstacleManager] Waiting for serice {}...".format(
            gazebo_ns + "/spawn_sdf_model"))
        rospy.wait_for_service(gazebo_ns + "/spawn_sdf_model")
        self._spawn_sdf_model = rospy.ServiceProxy(
            gazebo_ns + "/spawn_sdf_model", SpawnModel)

        if not rospy.has_param('~obstacles'):
            return False
        self._obstacles = rospy.get_param('~obstacles')

        self.spawn_obstacles()

        self._model_states_sub = rospy.Subscriber(
            gazebo_ns + "/model_states", ModelStates,
            self.model_states_callback)

        return True

    def spawn_obstacles(self):
        for o in self._obstacles:
            sdf_str = self.get_box_sdf_str(
                o['name'], o['pose']['pos']['x'], o['pose']['pos']['y'],
                o['pose']['pos']['z'], o['pose']['orien']['roll'],
                o['pose']['orien']['pitch'], o['pose']['orien']['yaw'],
                o['size']['x'], o['size']['y'], o['size']['z'])
            rospy.loginfo("[ObstacleManager] Requesting to spawn:\n{}\n".format(
                sdf_str))
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            try:
                res = self._spawn_sdf_model(
                    o['name'], sdf_str, '/', pose, 'world')
                if not res.success:
                    rospy.logerr(
                        "[ObstacleManager] Failed to spawn obstacle \"{}\"".
                        format(o['name']))
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(
                    "[ObstacleManager] Failed to call spawn service: {}".
                    format(e))
                return False

        return True

    def get_box_sdf_str(self, model_name, x, y, z, roll, pitch, yaw,
                        size_x, size_y, size_z, mass=1.0):
        ixx = 0.083 * mass * (size_y * size_y + size_z * size_z)
        iyy = 0.083 * mass * (size_x * size_x + size_z * size_z)
        izz = 0.083 * mass * (size_x * size_x + size_y * size_y)
        sdf_str = """<sdf version="1.4">
        <model name="{}">
          <pose>{} {} {} {} {} {}</pose>
          <static>true</static>
          <link name="body">
            <inertial>
              <mass>{}</mass>
              <inertia>
                <ixx>{}</ixx>
                <ixy>0.0</ixy>
                <ixz>0.0</ixz>
                <iyy>{}</iyy>
                <iyz>0.0</iyz>
                <izz>{}</izz>
              </inertia>
            </inertial>
          <collision name="collision">
            <geometry>
              <box>
                <size>{} {} {}</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>{} {} {}</size>
              </box>
            </geometry>
          </visual>
        </link>
      </model>
    </sdf>""".format(model_name, x, y, z, roll, pitch, yaw, mass, ixx, iyy, izz,
                     size_x, size_y, size_z, size_x, size_y, size_z)
        return sdf_str

    def model_states_callback(self, msg):
        for o in self._obstacles:
            i = -1
            for j, name in enumerate(msg.name):
                if name == o['name']:
                    i = j
                    break

            if i < 0:
                rospy.logwarn(
                    "[ObstacleManager] Couldn't get state of \"{}\"".format(
                        o['name']))
                continue

            obstacle_msg = Marker()
            obstacle_msg.header.frame_id = self._world_frame_id
            obstacle_msg.ns = "/obstacle_manager/" + o['name']
            obstacle_msg.id = 0
            obstacle_msg.type = Marker.CUBE
            obstacle_msg.pose = msg.pose[i]
            obstacle_msg.scale.x = o['size']['x']
            obstacle_msg.scale.y = o['size']['y']
            obstacle_msg.scale.z = o['size']['z']
            obstacle_msg.color.r = 1
            obstacle_msg.color.g = 0
            obstacle_msg.color.b = 0
            obstacle_msg.color.a = 0.5
            self._vis_pub.publish(obstacle_msg)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('obstacle_manager')

    manager = ObstacleManager()
    if not manager.init():
        rospy.logerr("Error: failed to initialize!")
        sys.exit(-1)

    manager.run()


if __name__ == '__main__':
    main()
