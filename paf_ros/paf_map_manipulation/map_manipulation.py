import copy

from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
import rospy

from paf_ros.paf_planning.src.classes.MapManager import MapManager

from commonroad.scenario.scenario import Scenario, Tag
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile


class MapManipulator:
    def __init__(self) -> None:
        self.scenario: Scenario = None
        self.inverse_scenario: Scenario = None
        self.bidirectional_scenario: Scenario = None
        self.save_path_prefix: str = "paf_ros/paf_map_manipulation/Temporary Maps/"

    def generate_no_rules_map(self):
        self._load_scenario()
        self._delete_signs_and_lights()
        self._create_inverse_scenario()
        self._create_bidirectional_scenario()
        self._generate_bidirectional_cr_file()

    def _load_scenario(self):
        self.scenario = MapManager.get_current_scenario(True)

    def _generate_cr_file(self):
        if self.scenario is not None:
            map_file_path = self.save_path_prefix + "TEMP_" + "DEU_" + rospy.get_param("/carla/town") + "-1_1_T-1.xml"
            writer = CommonRoadFileWriter(
                scenario=self.scenario,
                planning_problem_set=PlanningProblemSet(),
                author="paf21-2",
                affiliation="Augsburg University",
                source="CommonRoad Scenario Designer",
                tags={Tag.URBAN},
            )
            writer.write_to_file(map_file_path, OverwriteExistingFile.ALWAYS)
            rospy.loginfo("MapManipulator: Wrote file!")
        else:
            rospy.logerr("MapManipulator: Error while generating XML-File: Scenario is None.")

    def _generate_inverse_cr_file(self):
        if self.scenario is not None:
            map_file_path = self.save_path_prefix + "INV_" + "DEU_" + rospy.get_param("/carla/town") + "-1_1_T-1.xml"
            writer = CommonRoadFileWriter(
                scenario=self.inverse_scenario,
                planning_problem_set=PlanningProblemSet(),
                author="paf21-2",
                affiliation="Augsburg University",
                source="CommonRoad Scenario Designer",
                tags={Tag.URBAN},
            )
            writer.write_to_file(map_file_path, OverwriteExistingFile.ALWAYS)
            rospy.loginfo("MapManipulator: Wrote file!")
        else:
            rospy.logerr("MapManipulator: Error while generating XML-File: Scenario is None.")

    def _generate_bidirectional_cr_file(self):
        if self.scenario is not None:
            map_file_path = self.save_path_prefix + "BIDIR_" + "DEU_" + rospy.get_param("/carla/town") + "-1_1_T-1.xml"
            writer = CommonRoadFileWriter(
                scenario=self.bidirectional_scenario,
                planning_problem_set=PlanningProblemSet(),
                author="paf21-2",
                affiliation="Augsburg University",
                source="CommonRoad Scenario Designer",
                tags={Tag.URBAN},
            )
            writer.write_to_file(map_file_path, OverwriteExistingFile.ALWAYS)
            rospy.loginfo("MapManipulator: Wrote file!")
        else:
            rospy.logerr("MapManipulator: Error while generating XML-File: Scenario is None.")

    def _delete_signs_and_lights(self):
        if self.scenario is not None:
            for sign in self.scenario.lanelet_network.traffic_signs:
                self.scenario.lanelet_network.remove_traffic_sign(sign.traffic_sign_id)
            self.scenario.lanelet_network.cleanup_traffic_sign_references()
            for light in self.scenario.lanelet_network.traffic_lights:
                self.scenario.lanelet_network.remove_traffic_light(light.traffic_light_id)
            self.scenario.lanelet_network.cleanup_traffic_light_references()
            rospy.loginfo("MapManipulator: Deleted all signs and lights.")
        else:
            rospy.logerr("MapManipulator: Error while deleting signs and lights: Scenario is None.")

    def _create_inverse_scenario(self):
        if self.scenario is not None:
            rospy.loginfo("MapManipulator: Creating inverse scenario...")
            self._delete_signs_and_lights()
            inverse_lanelet_list = []
            for lane in self.scenario.lanelet_network.lanelets:
                predecessors = lane.predecessor
                successors = lane.successor
                reversed_center_vertices = lane.center_vertices[::-1]
                reversed_right_vertices = lane.right_vertices[::-1]
                reversed_left_vertices = lane.left_vertices[::-1]
                inverse_lanelet = Lanelet(
                    left_vertices=reversed_right_vertices,
                    center_vertices=reversed_center_vertices,
                    right_vertices=reversed_left_vertices,
                    lanelet_id=lane.lanelet_id,
                    predecessor=successors,
                    successor=predecessors,
                    adjacent_left=lane.adj_right,
                    adjacent_left_same_direction=lane.adj_right_same_direction,
                    adjacent_right=lane.adj_left,
                    adjacent_right_same_direction=lane.adj_left_same_direction,
                    line_marking_left_vertices=lane.line_marking_left_vertices,
                    line_marking_right_vertices=lane.line_marking_right_vertices,
                    stop_line=None,
                    lanelet_type=lane.lanelet_type,
                    user_one_way=lane.user_one_way,
                    user_bidirectional=lane.user_bidirectional,
                    traffic_signs=None,
                    traffic_lights=None,
                )
                inverse_lanelet_list.append(inverse_lanelet)
            self.inverse_scenario = Scenario(dt=self.scenario.dt)
            self.inverse_scenario.lanelet_network = LaneletNetwork.create_from_lanelet_list(inverse_lanelet_list)
            rospy.loginfo("MapManipulator: Creating inverse scenario successfull!")
        else:
            rospy.logerr("MapManipulator: Error while creating inverse scenario: Scenario is None.")

    def _create_bidirectional_scenario(self):
        if self.scenario is not None and self.inverse_scenario is not None:
            rospy.loginfo("MapManipulator: Creating bidirectional scenario...")
            self.bidirectional_scenario = Scenario(dt=self.scenario.dt)
            self.bidirectional_scenario.lanelet_network = copy.deepcopy(self.scenario.lanelet_network)
            self._change_lanelet_ids_of_inverse_scenario()
            ret_value = self.bidirectional_scenario.lanelet_network.add_lanelets_from_network(
                self.inverse_scenario.lanelet_network
            )
            print(ret_value)
            rospy.loginfo("MapManipulator: Creating bidirectional scenario successfull!")
        else:
            rospy.logerr(
                "MapManipulator: Error while creating bidirectional scenario: Scenario or inverse scenario is None."
            )

    def _set_all_neighbours_same_direction(self):
        if self.scenario is not None:
            for lane in self.scenario.lanelet_network.lanelets:
                if lane.adj_left is not None:
                    lane._adj_left_same_direction = True
                if lane.adj_right is not None:
                    lane._adj_right_same_direction = True

            rospy.loginfo("MapManipulator: Set every neighbour direction to same.")
        else:
            rospy.logerr("MapManipulator: Error while setting neighbour directions: Scenario is None.")

    def _get_max_id_of_lanelet_network(self, network: LaneletNetwork) -> int:
        highest_id = 0
        for lane in network.lanelets:
            current_id = lane.lanelet_id
            if current_id > highest_id:
                highest_id = current_id
        return highest_id

    def _change_lanelet_ids_of_inverse_scenario(self):
        highest_id = self._get_max_id_of_lanelet_network(network=self.bidirectional_scenario.lanelet_network)
        for lane in self.inverse_scenario.lanelet_network.lanelets:
            old_id = lane.lanelet_id
            new_id = old_id + highest_id + 1
            lane._lanelet_id = new_id
            self._update_lanelet_refs(old_lanelet_id=old_id, new_lanelet_id=lane.lanelet_id)

    def _update_lanelet_refs(self, old_lanelet_id, new_lanelet_id):
        for lane in self.inverse_scenario.lanelet_network.lanelets:
            if old_lanelet_id in lane.predecessor:
                lane.remove_predecessor(old_lanelet_id)
                lane.add_predecessor(new_lanelet_id)
            if old_lanelet_id in lane.successor:
                lane.remove_successor(old_lanelet_id)
                lane.add_successor(new_lanelet_id)
            if old_lanelet_id in lane.successor:
                lane.remove_successor(old_lanelet_id)
                lane.add_successor(new_lanelet_id)
            if lane.adj_left == old_lanelet_id:
                lane._adj_left = new_lanelet_id
            if lane.adj_right == old_lanelet_id:
                lane._adj_right = new_lanelet_id


def main():
    manipulator = MapManipulator()

    manipulator.generate_no_rules_map()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
