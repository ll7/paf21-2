#!/usr/bin/env python

from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_sign import TrafficSignIDGermany
import rospy

# import os

import matplotlib.pyplot as plt

# import functions to read xml file and visualize commonroad objects
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

# import necessary classes from different modules
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile
from commonroad.scenario.scenario import Tag


class MapManager:
    def __init__(self, init_rospy: bool = False):

        self.map_file_path: str = "CR Scenarios/DEU_Town04-1_1_T-1.xml"
        # name of the map received from the _world_info_subscriber
        self.map_name: str = "Town04"
        # boolean that represents the availability of the opendrive map, true if the map was received by the subscriber
        self.map_ready: bool = False

        self.scenario: Scenario = None
        self.planning_problem_set: PlanningProblemSet = None

        self.draw_params = {
            "scenario": {
                "dynamic_obstacle": {"trajectory": {"show_label": True, "draw_trajectory": False}},
                "lanelet_network": {
                    "traffic_sign": {
                        "draw_traffic_signs": True,
                        "show_traffic_signs": "all",
                    },
                    "intersection": {
                        "draw_intersections": False,
                        "draw_incoming_lanelets": True,
                        "incoming_lanelets_color": "#3ecbcf",
                        "draw_crossings": True,
                        "crossings_color": "#b62a55",
                        "draw_successors": True,
                        "successors_left_color": "#ff00ff",
                        "successors_straight_color": "blue",
                        "successors_right_color": "#ccff00",
                        "show_label": True,
                    },
                },
            }
        }

        # starts the logging node , normally only needed if the module is used independently
        if init_rospy:
            rospy.init_node("mapManager", anonymous=True)

    def _load_scenario(self):
        if self.map_file_path is not None:
            # read in the scenario and planning problem set
            self.scenario, self.planning_problem_set = CommonRoadFileReader(self.map_file_path).open()
            self.map_ready = True

    def _delete_all_signs_other_than_stop_and_speed(self):
        for sign in self.scenario.lanelet_network.traffic_signs:
            for sign_element in sign.traffic_sign_elements:
                if (
                    sign_element.traffic_sign_element_id == TrafficSignIDGermany.MAX_SPEED
                    or sign_element.traffic_sign_element_id == TrafficSignIDGermany.STOP
                ):
                    # do nothing
                    pass
                else:
                    sign.traffic_sign_elements.remove(sign_element)

    def generate_com_road_file(self):
        if self.scenario is not None:
            writer = CommonRoadFileWriter(
                scenario=self.scenario,
                planning_problem_set=self.planning_problem_set,
                author="paf21-2",
                affiliation="Augsburg University",
                source="CommonRoad Scenario Designer",
                tags={Tag.URBAN},
            )
            # write CommonRoad data to xml file
            writer.write_to_file(self.map_file_path, OverwriteExistingFile.ALWAYS)
            rospy.loginfo("MapManager: Wrote file " + self.map_file_path)
        else:
            rospy.logerr("MapManager: scenario not available")
            rospy.logerr("MapManager: No file generated")

    def _generate_image_file(self):
        print("MapManager: Creating .png file...")
        plt.figure(figsize=(70, 70))
        rnd = MPRenderer()
        self.scenario.draw(rnd, draw_params=self.draw_params)
        self.planning_problem_set.draw(rnd)
        rnd.render()
        plt.gca().set_aspect("equal")
        plt.savefig(self.map_name + ".png")
        plt.close()
        print("MapManager: .png file created.")

    """
    The following methods are only for adding speedlimits to town06
    """


def main():
    manager = MapManager(True)
    manager._load_scenario()
    manager._delete_all_signs_other_than_stop_and_speed()
    manager.generate_com_road_file()
    manager._generate_image_file()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
