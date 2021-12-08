#!/usr/bin/env python

from copy import deepcopy
import math
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignElement, TrafficSignIDGermany
import rospy
import numpy as np
from landmark_provider import LandMarkProvider, LandMarkPoint

# import os

import matplotlib.pyplot as plt

# import functions to read xml file and visualize commonroad objects
from commonroad.common.file_reader import CommonRoadFileReader
#from commonroad.visualization.mp_renderer import MPRenderer

# import necessary classes from different modules
from commonroad.common.file_writer import CommonRoadFileWriter, Point
from commonroad.common.file_writer import OverwriteExistingFile
from commonroad.scenario.scenario import Tag


class MapManager:
    def __init__(self, init_rospy: bool = False):

        self.map_file_path: str = "CR Scenarios/DEU_Town06-1_1_T-1.xml"
        # name of the map received from the _world_info_subscriber
        self.map_name: str = "Town06"
        # boolean that represents the availability of the opendrive map, true if the map was received by the subscriber
        self.map_ready: bool = False

        self.cur_mark_id = 10000
        self.max_angle_diff_sign = 45
        self.max_length_diff = 0.05  # in percent
        self.max_neighbour_angle_diff = 5  # in degree

        # define the standard intersection width, which is used to search for neighbouring lanelets in intersections
        self.inter_width = 20  # in meter

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
            self.scenario, self.planning_problem_set = CommonRoadFileReader(
                self.map_file_path).open()
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
            writer.write_to_file("Town06Speedlimits.xml",
                                 OverwriteExistingFile.ALWAYS)
            rospy.loginfo("MapManager: Wrote file Town06Speedlimits.xml")
        else:
            rospy.logerr("MapManager: scenario not available")
            rospy.logerr("MapManager: No file generated")

    # def _generate_image_file(self):
    #    print("MapManager: Creating .png file...")
    #    plt.figure(figsize=(70, 70))
    #    rnd = MPRenderer()
    #    self.scenario.draw(rnd, draw_params=self.draw_params)
    #    self.planning_problem_set.draw(rnd)
    #    rnd.render()
    #    plt.gca().set_aspect("equal")
    #    plt.savefig(self.map_name + ".png")
    #    plt.close()
    #    print("MapManager: .png file created.")

    """
    The following methods are only for adding speedlimits to town06
    """

    def _speed_signs_to_scenario(self, signs: list, speed: int):
        """
        Given a list of speed sing positions -> add speed signs to lanelets
        :param signs: list of speed sign positions
        :param speed: nomination of the sign (for example 50(kmh))
        """
        sign_added = False  # only true if a sign was added
        for sign in signs:
            mapped_lanelets = self._find_nearest_lanelet(sign.pos_as_point())
            if mapped_lanelets is not None:
                for lanelet in mapped_lanelets:
                    sign_pos_index = self._find_vertex_index(
                        lanelet, sign.pos_as_point())
                    lane_orientation = self.get_lanelet_orientation_at_index(
                        lanelet, sign_pos_index)
                    # angle_diff = 180 - \
                    #    abs(abs(sign.orientation - lane_orientation) - 180)
                    # add speed sign if the angle_diff matches and
                    # discard all other matches in mapped_lanelets since the sign can't be on that lanes
                    if True:
                        self._add_sign_to_lanelet(lanelet_id=lanelet.lanelet_id, pos_index=sign_pos_index,
                                                  typ=TrafficSignIDGermany.MAX_SPEED, additional=[speed])
                        neighbours = self._get_all_fitting_neighbour_lanelets(
                            lanelet.lanelet_id)
                        sign_added = True
                        # also add speed sign to neighbouring lanelets , that are heading in the same direction
                        for lane in neighbours:
                            self._add_sign_to_lanelet(lanelet_id=lane.lanelet_id, pos_index=sign_pos_index,
                                                      typ=TrafficSignIDGermany.MAX_SPEED, additional=[speed])
                        break
            else:
                rospy.logerr("MapSupervisorCommonRoads: Sing ID - " +
                             str(sign.mark_id) + " - did not match to any lanelet")
            if not sign_added:
                rospy.logerr("MapSupervisorCommonRoads: Sing ID - " +
                             str(sign.mark_id) + " - Orientation did not match")

    def _find_nearest_lanelet(self, goal: Point):
        """
        Given a Point (x,y,z) -> find nearest lanelet
        :param goal: point to which the nearest lanelet should be searched
        :return: nearest lanelet to point goal
        """
        nearest = None
        curr_radius = 1
        step_size = 0.5
        max_radius = 100
        while curr_radius < max_radius or nearest is not None:
            nearest = self.scenario.lanelet_network.lanelets_in_proximity(
                np.array([goal.x, goal.y]), curr_radius)
            if len(nearest) == 0:
                nearest = None
                curr_radius += step_size
            else:
                return nearest
        return None

    def _add_sign_to_lanelet(self, lanelet_id: int, pos_index: int, typ: TrafficSignIDGermany, additional: list = []):
        """
        Adds a sign to given lanelet
        :param lanelet_id: id of the given lanelet
        :param pos_index: index of the position of the sign within the lanelet
        :param typ: type of the sign (used for creating the TrafficSign object of CommonRoads)
        :param additional: additional of the sign (used for creating the TrafficSign object of CommonRoads)
        """
        self.cur_mark_id += 1
        pos = self.scenario.lanelet_network.find_lanelet_by_id(
            lanelet_id).center_vertices[pos_index]
        sign_element = TrafficSignElement(typ, additional)
        id_set = set()
        id_set.add(lanelet_id)
        sign = TrafficSign(self.cur_mark_id, first_occurrence=deepcopy(id_set), position=pos,
                           traffic_sign_elements=[sign_element])
        self.scenario.lanelet_network.add_traffic_sign(
            sign, lanelet_ids=deepcopy(id_set))

    def _get_all_fitting_neighbour_lanelets(self, lanelet_id: int) -> list:
        """
        Given a lanelet_id -> find all neighbouring lanelet to which are heading in the same direction
        :param lanelet_id: id of the lanelet under observation
        :return: list of fitting neighbouring lanelets
        """
        # get all right neighbours
        lane_under_obs = self.scenario.lanelet_network.find_lanelet_by_id(
            lanelet_id)
        neighbour_lanelets = []
        while lane_under_obs.adj_right_same_direction and lane_under_obs.adj_right is not None:
            lane_under_obs = self.scenario.lanelet_network.find_lanelet_by_id(
                lane_under_obs.adj_right)
            neighbour_lanelets.append(lane_under_obs)

        lane_under_obs = self.scenario.lanelet_network.find_lanelet_by_id(
            lanelet_id)
        # get all left neighbours
        while lane_under_obs.adj_left_same_direction and lane_under_obs.adj_left is not None:
            lane_under_obs = self.scenario.lanelet_network.find_lanelet_by_id(
                lane_under_obs.adj_left)
            neighbour_lanelets.append(lane_under_obs)
        return neighbour_lanelets

    @staticmethod
    def _find_vertex_index(lanelet: Lanelet, pos: Point):
        """
        Get the index of the closest point of a lanelet to a given comparison point
        :param lanelet: the given lanelet
        :param pos: the given comparison point
        :return: the closest index
        """
        # compute distances (we are not using the sqrt for computational effort)
        point = [pos.x, pos.y]
        distance = (lanelet.center_vertices - point) ** 2.
        distance = (distance[:, 0] + distance[:, 1])
        return np.argmin(distance).item()

    @staticmethod
    def get_lanelet_orientation_at_index(lanelet: Lanelet, index: int):
        """
        Get the orientation of the given lanelet at a certain index
        :param lanelet: the given lanelet
        :param index: index
        :return: orientation (as yaw angle)
        """
        angles = list()
        # check for bounds
        if index >= len(lanelet.center_vertices)-3:
            index = index - 2
        elif index < 2:
            index = 2
        for i in range(-2, 2, 1):
            prev_pos = 0
            pos = 0
            if i < 0:
                prev_pos = lanelet.center_vertices[index + i]
                pos = lanelet.center_vertices[index]
            elif i == 0:
                continue
            else:
                pos = lanelet.center_vertices[index + i]
                prev_pos = lanelet.center_vertices[index]

            # describes the relative position of the pos to the prev pos
            rel_x = 1 if (pos[0] - prev_pos[0]) >= 0 else -1
            rel_y = 1 if (prev_pos[1] - pos[1]) >= 0 else -1

            euler_angle_yaw = math.degrees(
                math.atan2(rel_y * abs(pos[1] - prev_pos[1]), rel_x * abs(pos[0] - prev_pos[0])))
            # get orientation without multiple rotations
            euler_angle_yaw = euler_angle_yaw % 360
            # get the positive angle
            if euler_angle_yaw < 0:
                euler_angle_yaw = euler_angle_yaw + 360
            angles.append(euler_angle_yaw)
        for i in range(0, len(angles) - 1):
            angle_diff = 180 - abs(abs(angles[i] - angles[i + 1]) - 180)
            if angle_diff < 20:
                return angles[i]
        return np.sum(angles) / len(angles)


def main():
    manager = MapManager(True)
    manager._load_scenario()
    # manager._delete_all_signs_other_than_stop_and_speed()

    # manager._generate_image_file()

    signs90 = []
    signs90.append(LandMarkPoint(x=-149, y=24, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=108, y=-253, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=-31, y=23, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=-371, y=-60, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=310, y=-253, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=597, y=26, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=497, y=26, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=195, y=-54, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=212, y=-53, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=283, y=-53, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=503, y=-53, orientation=0, id=0))
    signs90.append(LandMarkPoint(x=252, y=-153, orientation=0, id=0))

    signs60 = []
    signs60.append(LandMarkPoint(x=-245, y=24, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=-371, y=-187, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=-323, y=-251, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=-272, y=-251, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=-191, y=-251, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=-84, y=-251, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=41, y=-253, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=545, y=-253, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=664, y=-10, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=384, y=26, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=304, y=26, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=31, y=-151, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=106, y=-151, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=479, y=-153, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=47, y=-53, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=421, y=-36, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=550, y=-53, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=-42, y=-134, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=-131, y=-132, orientation=0, id=0))
    signs60.append(LandMarkPoint(x=-215, y=-133, orientation=0, id=0))

    signs40 = []
    signs40.append(LandMarkPoint(x=600, y=-254, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=-9, y=-267, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=673, y=-193, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=675, y=-107, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=333, y=8, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=205, y=25, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=120, y=25, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=576, y=-154, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=96, y=-36, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=582, y=-53, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=513, y=-28, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=488, y=-130, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=518, y=-78, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=185, y=-87, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=12, y=-211, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=-12, y=-179, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=-13, y=-121, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=10, y=-95, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=-14, y=-75, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=-226, y=-55, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=-116, y=-55, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=-156, y=-9, orientation=0, id=0))
    signs40.append(LandMarkPoint(x=-129, y=6, orientation=0, id=0))

    signs30 = []
    signs30.append(LandMarkPoint(x=124, y=-226, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=-71, y=-251, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=-11, y=-353, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=14, y=-327, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=14, y=-301, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=-82, y=-225, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=-75, y=-177, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=-249, y=-199, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=-231, y=-156, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=128, y=-122, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=504, y=-108, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=145, y=-23, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=10, y=-12, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=-14, y=-14, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=11, y=49, orientation=0, id=0))
    signs30.append(LandMarkPoint(x=-12, y=87, orientation=0, id=0))

    manager._speed_signs_to_scenario(signs90, 90)
    manager._speed_signs_to_scenario(signs60, 60)
    manager._speed_signs_to_scenario(signs40, 40)
    manager._speed_signs_to_scenario(signs30, 30)

    manager.generate_com_road_file()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
