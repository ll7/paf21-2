#!/usr/bin/env python
from typing import Tuple

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
    """Implementation of TopDownView (CARLA) within ROS"""

    br = CvBridge()
    LOG_FPS_SECS = 60

    def __init__(self, _world: carla.World, _client: carla.Client, _actor: carla.Actor):
        """
        Init subscribers and connect to carla instance
        :param _world: carla world object
        :param _client: carla client object
        :param _actor: carla ego vehicle object
        """
        self.params = rospy.get_param("/top_down_view/")
        self._actor = _actor
        self.point_sets = {}
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
        rospy.Subscriber("/paf/paf_global_planner/routing_response_tdv", PafLaneletRoute, self.update_global_path)
        rospy.Subscriber("/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, self._update_pt_set)
        rospy.Subscriber("/paf/paf_validation/draw_map_lines", PafTopDownViewPointSet, self._update_line_set)
        rospy.Subscriber("/paf/paf_validation/speed_text", PafSpeedMsg, self._update_speed_str)

    def update_obstacles(self, msg: PafObstacleList):
        """
        Obstacle callback
        :param msg: obstacles list
        """
        self.producer.update_obstacles(msg)

    def _update_speed_str(self, msg: PafSpeedMsg):
        """
        Speed Information callback
        :param msg: info message
        """
        self.producer.info_text = [
            int(self.velocity()),
            int(np.round(msg.target * 3.6)),
            int(np.round(msg.limit * 3.6)),
            self.location(),
        ]

    def _update_line_set(self, msg: PafTopDownViewPointSet):
        """
        Update or fresh debug line/path for the map
        :param msg: list of points with label and color
        """
        self.producer.line_sets[msg.label] = msg

    def _update_pt_set(self, msg: PafTopDownViewPointSet):
        """
        Update or fresh debug points for the map
        :param msg: list of points with label and color
        """
        self.producer.point_sets[msg.label] = msg

    def update_global_path(self, msg: PafLaneletRoute):
        """
        global path processing. Updates drawable global paths:
        leftmost, rightmost lane and leftmost, rightmost target-lane
        :param msg: current route from paf_global_planner
        """
        paths = [[] for _ in range(1)]
        section: PafRouteSection
        nan = 0
        for section in msg.sections:
            try:
                point = section.points[section.target_lanes[0]]
                paths[0].append([point.x, -point.y])
                # point = section.points[section.target_lanes[-1]]
                # paths[1].append([point.x, -point.y])
                # point = section.points[0]
                # paths[3].append([point.x, -point.y])
                # point = section.points[-1]
                # paths[4].append([point.x, -point.y])
            except IndexError:
                nan += 1
                continue
        if nan > 0:
            rospy.logwarn_throttle(
                5, "[top down view] some of the points of the global path contain nan/undefined values."
            )
        self.producer.set_path(coordinate_list_global_path=paths)

    def update_local_path(self, msg: PafLocalPath):
        """
        local path update
        :param msg: current LP from paf_local_planner
        """
        path = [[point.x, -point.y] for point in msg.points]
        self.producer.set_path(coordinate_list_local_path=path)

    def produce_map(self) -> np.ndarray:
        """
        Create a new frame for RVIZ, combines all layers to a single image
        :return: created image (rgb)
        """
        birdview = self.producer.produce(agent_vehicle=self._actor)
        return self.producer.as_rgb(birdview)

    def start(self):
        """
        main loop. Produce and publish every update_hz.
        """
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

    def velocity(self) -> float:
        """
        Calculate velocity from CARLA-Information
        :return: speed (amount, not vector)
        """
        s = self._actor.get_velocity()
        return np.sqrt(s.x ** 2 + s.y ** 2 + s.z ** 2) * 3.6

    def location(self) -> Tuple[float, float]:
        """
        Retrieve position from CARLA-Information
        :return: (x,y) position
        """
        s = self._actor.get_location()
        return np.round(s.x, 1), np.round(-s.y, 1)

    def _move_spectator(self):
        """
        Follow ego vehicle around from above, update if out of range (same view as to TopDownView map)
        """
        vehicle_tf = self._actor.get_transform()
        vehicle_tf.location += carla.Location(x=0, y=0, z=60)
        vehicle_tf.rotation = carla.Rotation(pitch=-90, roll=0, yaw=-90)
        spec_tf = self._spectator.get_transform()
        if (vehicle_tf.location.x - spec_tf.location.x) ** 2 + (
            vehicle_tf.location.y - spec_tf.location.y
        ) ** 2 > 20 ** 2:
            self._spectator.set_transform(vehicle_tf)

    def __del__(self):
        try:
            self._spectator.set_transform(carla.Transform())
        except Exception:
            pass


if __name__ == "__main__":
    client = carla.Client("127.0.0.1", 2000)
    search_name = rospy.get_param("~role_name", "ego_vehicle")
    world = None
    # retrieve ego_vehicle actor from list of all actors, then start tdv. View centers around this actor.
    try:
        while True:
            world = client.get_world()
            actors = world.get_actors()
            for actor in actors:
                if "role_name" in actor.attributes and actor.attributes["role_name"] == search_name:
                    loc = actor.get_location()
                    rospy.logwarn(f"Tracking {actor.type_id} ({actor.attributes['role_name']}) at {loc}")
                    node = TopDownRosNode(world, client, actor)
                    node.start()
                    break
            else:
                rospy.logwarn("ego vehicle not found, retrying")
                sleep(1)
    finally:
        if world is not None:
            world.get_spectator().set_transform(carla.Transform())
