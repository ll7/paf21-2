#!/usr/bin/env python

import rospy
import os

import matplotlib.pyplot as plt
from IPython import display

# import functions to read xml file and visualize commonroad objects
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer


class MapManager:

    def __init__(self, init_rospy: bool = False):
    
        # name of the map received from the _world_info_subscriber
        self.map_name: str = None
        # boolean that represents the availability of the opendrive map -> true if the map was received by the subscriber
        self.map_ready: bool = False
        # starts the logging node , normally only needed if the module is used independently
        if init_rospy:
            rospy.init_node('mapProvider', anonymous=True)


    def __del__(self):
        # closing the TemporaryFile deletes it
        self.osm_file.close()

    def _load_scenario(file_path):
        # read in the scenario and planning problem set
        scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

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
    manager = MapManager(True)
    


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
