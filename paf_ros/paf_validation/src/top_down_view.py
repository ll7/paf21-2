#!/usr/bin/env python
import carla
import numpy as np
import rospy
from carla_birdeye_view.mask import PixelDimensions

from time import perf_counter, sleep
from paf_perception.msg import PafObstacleList
from classes.TopDownView import TopDownView
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class TopDownRosNode(object):
    br = CvBridge()
    LOG_FPS_SECS = 60

    def __init__(self, _client, _actor):
        self.params = rospy.get_param("/top_down_view/")
        self.actor = _actor
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
        self.update_local_path("temporary")
        print(f"top_down_view tracking {self.actor.type_id}")
        rospy.init_node(self.params["node"], anonymous=True)
        self.pub = rospy.Publisher(self.params["topic"], Image, queue_size=1)
        rospy.Subscriber(rospy.get_param("obstacles_topic"), PafObstacleList, self.update_obstacles)
        # rospy.Subscriber(rospy.get_param("local_path_topic"), MsgType, self.update_local_path)

    def update_obstacles(self, msg: PafObstacleList):
        self.producer.update_obstacles(msg)

    def update_local_path(self, msg):
        # todo temporary
        dummy_path = [
            [199.0, -9.5],
            [210.0, -9.5],
            [219.0, -9.5],
            [224.4, -9.9],
            [227.8, -12.3],
            [230.1, -15.7],
            [231.0, -20.2],
            [231.1, -27.6],
            [231.2, -34.7],
        ]
        dummy_path = np.array(dummy_path) + [1, 5]
        self.producer.set_path(coordinate_list_local_path=dummy_path)

    def produce_map(self):
        birdview = self.producer.produce(agent_vehicle=self.actor)
        return self.producer.as_rgb(birdview)

    def start(self):
        rate = rospy.Rate(self.params["update_hz"])
        while not rospy.is_shutdown():
            t0 = perf_counter()
            rgb = self.produce_map()
            self.pub.publish(self.br.cv2_to_imgmsg(rgb, "rgb8"))
            rospy.logwarn_throttle(self.LOG_FPS_SECS, f"[top_down_view] fps={1 / (perf_counter() - t0)}")
            rate.sleep()


def main():
    client = carla.Client("127.0.0.1", 2000)

    while True:
        actors = client.get_world().get_actors()
        for actor in actors:
            if "role_name" in actor.attributes and actor.attributes["role_name"] == rospy.get_param("role_name"):
                rospy.logwarn(f"Tracking {actor.type_id} ({actor.attributes['role_name']}) at {actor.get_location()}")
                TopDownRosNode(client, actor).start()
                return
        else:
            rospy.logwarn("ego vehicle not found, retrying")
            sleep(1)


if __name__ == "__main__":
    main()
