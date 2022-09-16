#!/usr/bin/env python

import rospy
import sys
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from ellis_planner.srv import AddObstacle, AddObstacleResponse
from std_srvs.srv import Trigger, TriggerResponse


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

        rospy.loginfo("[ObstacleManager] Waiting for service {}...".format(
            gazebo_ns + "/delete_model"))
        rospy.wait_for_service(gazebo_ns + "/delete_model")
        self._delete_model = rospy.ServiceProxy(
            gazebo_ns + "/delete_model", DeleteModel)

        if not rospy.has_param('~obstacles'):
            return False
        self._obstacles = rospy.get_param('~obstacles')

        if not self.spawn_obstacles():
            rospy.logwarn("[ObstacleManager] Failed to spawn some obstacles")

        self._model_states_sub = rospy.Subscriber(
            gazebo_ns + "/model_states", ModelStates,
            self.model_states_callback)

        self._add_obstacle_srv = rospy.Service(
            '~add_obstacle', AddObstacle, self.handle_add_obstacle)

        self._clear_obstacles_srv = rospy.Service(
            '~clear_obstacles', Trigger, self.handle_clear_obstacles)

        return True

    def spawn_obstacles(self):
        for o in self._obstacles:
            moveable = False
            if 'moveable' in o:
                moveable = o['moveable']

            sdf_str = ""
            if moveable:
                sdf_str = self.get_moveable_box_sdf_str(
                    o['name'], o['pose']['pos']['x'], o['pose']['pos']['y'],
                    o['pose']['pos']['z'], o['pose']['orien']['roll'],
                    o['pose']['orien']['pitch'], o['pose']['orien']['yaw'],
                    o['size']['x'], o['size']['y'], o['size']['z'],
                    o['mass'], o['restitution_coeff'], o['friction_coeff'])
            else:
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

    def handle_clear_obstacles(self, req):
        for o in self._obstacles:
            try:
                res = self._delete_model(o['name'])
                if not res.success:
                    rospy.logerr(
                        "[ObstacleManager] Failed to delete obstacle \"{}\": {}".
                        format(o['name'], res.status_message))
            except rospy.ServiceException as e:
                rospy.logerr(
                    "[ObstacleManager] Failed to call delete model service: {}".
                    format(e))

        self._obstacles = []
        return TriggerResponse(True, "")

    def handle_add_obstacle(self, req):
        if req.type != "rectangle":
            rospy.logerr("[ObstacleManager] Type \"{}\" is not supported!".format(req.type))
            return AddObstacleResponse(False)

        roll, pitch, yaw = euler_from_quaternion((
            req.pose.orientation.x,
            req.pose.orientation.y,
            req.pose.orientation.z,
            req.pose.orientation.w
        ))

        # TODO(eratner) A little hacky...
        if len(req.name) >= 4 and req.name[0:4] == 'wire':
            req.size.z = 0.02
            # TODO Assign a special color to the wires?

        ########################################################################
        # FOR THE FIGURE #######################################################
        # if len(req.name) >= 4 and req.name[0:4] == 'wire':
        #     roll = 0.65
        #     pitch = 0.0
        #     # yaw = 0.0
        #     req.size.x = 0.02
        #     req.size.y = 1.0
        #     req.size.z = 0.02
        ########################################################################

        if req.moveable:
            sdf_str = self.get_moveable_box_sdf_str(
                req.name, req.pose.position.x, req.pose.position.y,
                req.pose.position.z, roll, pitch, yaw, req.size.x,
                req.size.y, req.size.z, req.mass, req.restitution_coeff,
                req.friction_coeff
            )
        else:
            sdf_str = self.get_box_sdf_str(
                req.name, req.pose.position.x, req.pose.position.y,
                req.pose.position.z, roll, pitch, yaw, req.size.x,
                req.size.y, req.size.z
            )

        o = {
            'name': req.name,
            'size': {
                'x': req.size.x,
                'y': req.size.y,
                'z': req.size.z
            },
            'pose': {
                'pos': {
                    'x': req.pose.position.x,
                    'y': req.pose.position.y,
                    'z': req.pose.position.z
                },
                'orien': {
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw
                }
            },
            'moveable': req.moveable,
            'mass': req.mass,
            'restitution_coeff': req.restitution_coeff,
            'friction_coeff': req.friction_coeff
        }
        self._obstacles.append(o)

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
                req.name, sdf_str, '/', pose, 'world')
            if not res.success:
                rospy.logerr(
                    "[ObstacleManager] Failed to spawn obstacle \"{}\"".
                    format(req.name))
                return AddObstacleResponse(False)
        except rospy.ServiceException as e:
            rospy.logerr(
                "[ObstacleManager] Failed to call spawn service: {}".
                format(e))
            return AddObstacleResponse(False)

        return AddObstacleResponse(True)

    def get_moveable_box_sdf_str(self, model_name, x, y, z, roll, pitch, yaw,
                                 size_x, size_y, size_z, mass=1.0,
                                 restitution_coeff=0.0,
                                 friction_coeff=1.0):
        ixx = 0.083 * mass * (size_y * size_y + size_z * size_z)
        iyy = 0.083 * mass * (size_x * size_x + size_z * size_z)
        izz = 0.083 * mass * (size_x * size_x + size_y * size_y)
        sdf_str = """<sdf version="1.4">
        <model name="{}">
          <pose>{} {} {} {} {} {}</pose>
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
            <surface>
              <bounce>
                <restitution_coefficient>{}</restitution_coefficient>
                <threshold>0</threshold>
              </bounce>
              <friction>
                <ode>
                  <mu>{}</mu>
                  <mu2>{}</mu2>
                </ode>
              </friction>
            </surface>
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
                     size_x, size_y, size_z,
                     restitution_coeff, friction_coeff, friction_coeff,
                     size_x, size_y, size_z)
        return sdf_str

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
            obstacle_msg.color.r = 0.42
            obstacle_msg.color.g = 0.42
            obstacle_msg.color.b = 0.42
            obstacle_msg.color.a = 0.85 # 1.0
            self._vis_pub.publish(obstacle_msg)

            if 'moveable' in o and o['moveable']:
                # Show the mass and friction coefficient.
                info_msg = Marker()
                info_msg.header.frame_id = self._world_frame_id
                info_msg.ns = "/obstacle_manager/" + o['name']
                info_msg.id = 1
                info_msg.type = Marker.TEXT_VIEW_FACING
                info_msg.pose = msg.pose[i]
                info_msg.pose.position.z += (0.55 * o['size']['z'])
                info_msg.scale.z = 0.02
                info_msg.color.r = 0.0
                info_msg.color.g = 0.0
                info_msg.color.b = 1.0
                info_msg.color.a = 1.0
                info_msg.text = "mass: {}\nfric: {}".format(
                    o['mass'], o['friction_coeff'])
                self._vis_pub.publish(info_msg)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('obstacle_manager')

    mgr = ObstacleManager()
    if not mgr.init():
        rospy.logerr("Error: failed to initialize!")
        sys.exit(-1)

    mgr.run()


if __name__ == '__main__':
    main()
