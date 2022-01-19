#!/usr/bin/env python
import carla
import rospy

from carla_birdeye_view.mask import PixelDimensions
from time import sleep

from paf_messages.msg import (
    PafObstacleList,
    PafLocalPath,
    PafLaneletRoute,
    PafTopDownViewPointSet,
    PafSpeedMsg,
    PafRouteSection,
)
from classes.TopDownView import TopDownView
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge


class TopDownRosNode(object):
    br = CvBridge()
    LOG_FPS_SECS = 60

    def __init__(self, _world, _client, _actor):
        self.params = rospy.get_param("/top_down_view/")
        self._actor = _actor
        self.point_sets = {}
        self._current_speed = 0
        self._spectator = _world.get_spectator()
        self.producer = TopDownView(
            _client,
            target_size=PixelDimensions(
                width=self.params["img_size"]["width"], height=self.params["img_size"]["height"]
            ),
            pixels_per_meter=self.params["pixels_per_meter"],
            show_whole_map=self.params["show_whole_map"],
            north_is_up=self.params["north_is_up"],
            dark_mode=self.params["dark_mode"],
        )
        print(f"top_down_view tracking {self._actor.type_id}")
        rospy.init_node(self.params["node"], anonymous=True)
        self.pub = rospy.Publisher(self.params["topic"], Image, queue_size=1)
        rospy.Subscriber(rospy.get_param("obstacles_topic"), PafObstacleList, self.update_obstacles)
        rospy.Subscriber("/paf/paf_local_planner/path", PafLocalPath, self.update_local_path)
        rospy.Subscriber(rospy.get_param("global_path_topic"), PafLaneletRoute, self.update_global_path)
        rospy.Subscriber("/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, self._update_pt_set)
        rospy.Subscriber("/paf/paf_validation/draw_map_lines", PafTopDownViewPointSet, self._update_line_set)
        rospy.Subscriber("/paf/paf_validation/speed_text", PafSpeedMsg, self._update_speed_str)

    def update_obstacles(self, msg: PafObstacleList):
        self.producer.update_obstacles(msg)

    def _update_speed_str(self, msg: PafSpeedMsg):
        self.producer.info_text = [
            int(self.velocity()),
            int(np.round(msg.target * 3.6)),
            int(np.round(msg.limit * 3.6)),
            self.location(),
        ]

    def _update_line_set(self, msg: PafTopDownViewPointSet):
        self.producer.line_sets[msg.label] = msg

    def _update_pt_set(self, msg: PafTopDownViewPointSet):
        self.producer.point_sets[msg.label] = msg

    def update_global_path(self, msg: PafLaneletRoute):
        path = []
        section: PafRouteSection
        nan = 0
        for section in msg.sections:
            try:
                point = section.points[section.target_lanes[0]]
            except IndexError:
                nan += 1
                point = section.points[0]
            if np.isnan(point.x):
                nan += 1
                continue
            path.append([point.x, -point.y])
        if nan > 0:
            rospy.logerr_throttle(
                5, f"[top down view] global path contains {nan} invalid points " f"(nan or target lanes index error)"
            )
        self.producer.set_path(coordinate_list_global_path=path)

    def update_local_path(self, msg: PafLocalPath):
        path = [[point.x, -point.y] for point in msg.points]
        self.producer.set_path(coordinate_list_local_path=path)

    def produce_map(self):
        birdview = self.producer.produce(agent_vehicle=self._actor)
        return self.producer.as_rgb(birdview)

    def start(self):
        rate = rospy.Rate(self.params["update_hz"])
        while not rospy.is_shutdown():
            t0 = rospy.Time.now().to_time()
            rgb = self.produce_map()
            self.pub.publish(self.br.cv2_to_imgmsg(rgb, "rgb8"))
            delta = rospy.Time.now().to_time() - t0
            self.producer.info_text[0] = int(self.velocity())
            self.producer.info_text[3] = self.location()
            if delta > 0:
                rospy.loginfo_throttle(self.LOG_FPS_SECS, f"[top_down_view] fps={np.round(1 / delta, 2)}")
            self._move_spectator()
            rate.sleep()

    def velocity(self):
        s = self._actor.get_velocity()
        return np.sqrt(s.x ** 2 + s.y ** 2 + s.z ** 2) * 3.6

    def location(self):
        s = self._actor.get_location()
        return np.round(s.x, 1), np.round(-s.y, 1)

    def _move_spectator(self):
        vehicle_tf = self._actor.get_transform()
        vehicle_tf.location += carla.Location(x=0, y=0, z=60)
        vehicle_tf.rotation = carla.Rotation(pitch=-90, roll=0, yaw=-90)
        spec_tf = self._spectator.get_transform()
        if (vehicle_tf.location.x - spec_tf.location.x) ** 2 + (
            vehicle_tf.location.y - spec_tf.location.y
        ) ** 2 > 30 ** 2:
            self._spectator.set_transform(vehicle_tf)


def main():
    client = carla.Client("127.0.0.1", 2000)
    search_name = rospy.get_param("~role_name", "ego_vehicle")

    while True:
        world = client.get_world()
        actors = world.get_actors()
        for actor in actors:
            if "role_name" in actor.attributes and actor.attributes["role_name"] == search_name:
                loc = actor.get_location()
                rospy.logwarn(f"Tracking {actor.type_id} ({actor.attributes['role_name']}) at {loc}")
                node = TopDownRosNode(world, client, actor)
                try:
                    node.start()
                finally:
                    world.get_spectator().set_transform(carla.Transform())
                return
        else:
            rospy.logwarn("ego vehicle not found, retrying")
            sleep(1)


if __name__ == "__main__":
    main()
