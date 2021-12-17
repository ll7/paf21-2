#!/usr/bin/env python
import carla
import rospy

from carla_birdeye_view.mask import PixelDimensions
from time import sleep

from paf_messages.msg import PafObstacleList, PafLocalPath, PafLaneletRoute, PafTopDownViewPointSet, PafSpeedMsg
from classes.TopDownView import TopDownView
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge


class TopDownRosNode(object):
    br = CvBridge()
    LOG_FPS_SECS = 60

    def __init__(self, _client, _actor):
        self.params = rospy.get_param("/top_down_view/")
        self.actor = _actor
        self.point_sets = {}
        self._current_speed = 0
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
        print(f"top_down_view tracking {self.actor.type_id}")
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
        path = [[point.x, -point.y] for point in msg.points[::10]]
        self.producer.set_path(coordinate_list_global_path=path)

    def update_local_path(self, msg: PafLocalPath):
        path = [[point.x, -point.y] for point in msg.points]
        self.producer.set_path(coordinate_list_local_path=path)

    def produce_map(self):
        birdview = self.producer.produce(agent_vehicle=self.actor)
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
                rospy.logwarn_throttle(self.LOG_FPS_SECS, f"[top_down_view] fps={1 / delta}")
            rate.sleep()

    def velocity(self):
        s = self.actor.get_velocity()
        return np.sqrt(s.x ** 2 + s.y ** 2 + s.z ** 2) * 3.6

    def location(self):
        s = self.actor.get_location()
        return np.round(s.x, 1), np.round(-s.y, 1)


def main():
    client = carla.Client("127.0.0.1", 2000)
    search_name = rospy.get_param("~role_name", "ego_vehicle")

    while True:
        actors = client.get_world().get_actors()
        for actor in actors:
            if "role_name" in actor.attributes and actor.attributes["role_name"] == search_name:
                loc = actor.get_location()
                rospy.logwarn(f"Tracking {actor.type_id} ({actor.attributes['role_name']}) at {loc}")
                TopDownRosNode(client, actor).start()
                return
        else:
            rospy.logwarn("ego vehicle not found, retrying")
            sleep(1)


if __name__ == "__main__":
    main()
