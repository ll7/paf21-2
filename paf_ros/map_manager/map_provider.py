#!/usr/bin/env python

from commonroad.scenario.lanelet import LaneletType
import rospy
import os

from io import BytesIO
from lxml import etree
from carla_msgs.msg import CarlaWorldInfo

"""
from opendrive2lanelet.opendriveparser.parser import parse_opendrive
from opendrive2lanelet.network import Network
from opendrive2lanelet.osm.lanelet2osm import L2OSMConverter
"""

from crdesigner.map_conversion.opendrive.opendrive_parser.parser import parse_opendrive
from crdesigner.map_conversion.opendrive.opendrive_conversion.network import Network

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Tag
from commonroad.scenario.scenario import Scenario

import tempfile


class MapProvider:

    def __init__(self, init_rospy: bool = False):
        self.osm_file = tempfile.NamedTemporaryFile(suffix=".osm")
        # opendrive map received from the _world_info_subscriber
        self.map: str = None
        # name of the map received from the _world_info_subscriber
        self.map_name: str = None
        # boolean that represents the availability of the opendrive map -> true if the map was received by the subscriber
        self.map_ready: bool = False
        # starts the logging node , normally only needed if the module is used independently
        if init_rospy:
            rospy.init_node('mapProvider', anonymous=True)

        # Subscriber to receive the currently loaded map
        self._world_info_subscriber = rospy.Subscriber(
            "/carla/world_info", CarlaWorldInfo, self.update_world)

    def __del__(self):
        # closing the TemporaryFile deletes it
        self.osm_file.close()

    def update_world(self, world_info: CarlaWorldInfo):
        """
        Check if a new map was sent and receive it
        :param world_info: Carla world info data
        """
        self.map_ready = False
        rospy.loginfo("MapProvider: Received new map info")
        if self.map_name == world_info.map_name:
            rospy.loginfo("MapProvider: Map already loaded")
        else:
            self.map_name = world_info.map_name
            self.map = world_info.opendrive
            self.map_ready = True
            rospy.loginfo("MapProvider: Received: " + self.map_name)

    def convert_od_to_lanelet(self) -> Scenario:
        """
        Create a CommonRoad scenario from the OpenDrive received OpenDrive map
        :return: Scenario if the map is ready, None instead
        """
        lanelet: Scenario = None
        if self.map_ready:
            rospy.loginfo("MapProvider: Start conversion...")
            opendrive = parse_opendrive(etree.parse(BytesIO(self.map.encode('utf-8'))).getroot())
            roadNetwork = Network()
            roadNetwork.load_opendrive(opendrive)
            lanelet = roadNetwork.export_commonroad_scenario()
            rospy.loginfo("MapProvider: Conversion done! Removing Sidewalks...")
            for lane in lanelet.lanelet_network.lanelets:
                if lane.lanelet_type == LaneletType.SIDEWALK:
                    lanelet.lanelet_network.remove_lanelet(lane.lanelet_id)
        return lanelet

    """
    The generate methods are just for debugging purposes 
    """

    def generate_com_road_file(self):
        lanelet = self.convert_od_to_lanelet()
        if lanelet is not None:
            writer = CommonRoadFileWriter(
                scenario=lanelet,
                planning_problem_set=PlanningProblemSet(),
                author="Psaf1",
                affiliation="",
                source="MapProvider",
                tags={Tag.URBAN, Tag.HIGHWAY},
            )
            # write CommonRoad data to xml file
            writer.write_to_file(self.map_name + "MapProvider" + ".xml", OverwriteExistingFile.ALWAYS)
            rospy.loginfo("MapProvider: Wrote file " + self.map_name + "MapProvider" + ".xml")
        else:
            rospy.logerr("MapProvider: lanelet not available")
            rospy.logerr("MapProvider: No file generated")


def main():
    provider = MapProvider(True)
    while not provider.map_ready:
        rospy.loginfo("Waiting")
    provider.convert_od_to_lanelet()
    provider.generate_com_road_file()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
