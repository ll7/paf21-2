import copy
from math import dist

from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
import rospy

from paf_ros.paf_planning.src.classes.MapManager import MapManager

from commonroad.scenario.scenario import Scenario, Tag
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile


class MapManipulator:
    """
    The purpose of this class is to load, alter and save commonroad scenario files.
    How to manipulate a map: Implement your manipulation as a method in this class,
    then load the scenario file, call the implemented method and save the xml file in the main()-function below.
    """

    def __init__(self) -> None:
        """
        Initialize member variables here.
        """
        self.scenario: Scenario = None
        self.inverse_scenario: Scenario = None
        self.bidirectional_scenario: Scenario = None
        self.save_path_prefix: str = "paf_ros/paf_map_manipulation/Temporary Maps/"
        self.corresponding_lanelets_old_new = {}
        self.corresponding_lanelets_new_old = {}

    def generate_no_rules_map(self):
        """
        Loads a scenario, creates a bidirectional scenario from it and saves the bidirectional scenario as new file.
        """
        self._load_scenario()
        self._delete_signs_and_lights()
        self._create_inverse_scenario()
        self._create_bidirectional_scenario()
        self._connect_normal_and_inverse_lanelets()
        self._generate_bidirectional_cr_file()

    def _load_scenario(self):
        """
        Loads a commonroad scenario from a .xml-file. Uses the MapManager-Class from the paf_planning package.
        """
        self.scenario = MapManager.get_current_scenario(rules=False, map_name="Town03")

    def _generate_cr_file(self):
        """
        Saves the current scenario (self.scenario) as .xml-file in the directory given by self.save_path_prefix.
        """
        if self.scenario is not None:
            map_file_path = self.save_path_prefix + "REM_" + "DEU_" + "Town03" + "-1_1_T-1.xml"
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
        """
        Saves the current inverse scenario (self.inverse_scenario) as .xml-file in the
        directory given by self.save_path_prefix.
        """
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
        """
        Saves the current bidirectional scenario (self.bidirectional_scenario) as .xml-file in the
        directory given by self.save_path_prefix.
        """
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
        """
        Deletes all traffic signs and traffic lights from the current scenario (self.scenario).
        """
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
        """
        Creates a scenario that is inverse to self.scenario by swapping successors, predecessors
        and adjacent lanelets and inverting the left-, right- and center-vertices for every lanelet.
        The inverse scenario is stored in self.inverse_scenario.
        """
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
        """
        Creates bidirectional scenario by adding the lanelets of self.scenario and
        self.inverse_scenario to self.bidirectional_scenario.
        """
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

    def _connect_normal_and_inverse_lanelets(self):
        """
        Connects the lanelets of the normal scenario with the lanelets of the
        inverse scenario in self.bidirectional_scenario
        by creating adjacence references between the normal and the inverse lanelets.
        """
        if self.bidirectional_scenario is not None:
            rospy.loginfo("MapManipulator: Connecting normal and inverse lanelets...")
            for lane in self.scenario.lanelet_network.lanelets:
                if lane.adj_left is not None and not lane.adj_left_same_direction:
                    self.bidirectional_scenario.lanelet_network.find_lanelet_by_id(
                        lane.lanelet_id
                    )._adj_left = self.corresponding_lanelets_old_new[lane.adj_left]
                    self.bidirectional_scenario.lanelet_network.find_lanelet_by_id(
                        lane.lanelet_id
                    )._adj_left_same_direction = True
                if lane.adj_right is not None and not lane.adj_right_same_direction:
                    self.bidirectional_scenario.lanelet_network.find_lanelet_by_id(
                        lane.lanelet_id
                    )._adj_right = self.corresponding_lanelets_old_new[lane.adj_right]
                    self.bidirectional_scenario.lanelet_network.find_lanelet_by_id(
                        lane.lanelet_id
                    )._adj_right_same_direction = True
            for lane in self.inverse_scenario.lanelet_network.lanelets:
                if lane.adj_left is not None and not lane.adj_left_same_direction:
                    self.bidirectional_scenario.lanelet_network.find_lanelet_by_id(
                        lane.lanelet_id
                    )._adj_left = self.corresponding_lanelets_new_old[lane.adj_left]
                    self.bidirectional_scenario.lanelet_network.find_lanelet_by_id(
                        lane.lanelet_id
                    )._adj_left_same_direction = True
                if lane.adj_right is not None and not lane.adj_right_same_direction:
                    self.bidirectional_scenario.lanelet_network.find_lanelet_by_id(
                        lane.lanelet_id
                    )._adj_right = self.corresponding_lanelets_new_old[lane.adj_right]
                    self.bidirectional_scenario.lanelet_network.find_lanelet_by_id(
                        lane.lanelet_id
                    )._adj_right_same_direction = True
            rospy.loginfo("MapManipulator: Connecting normal and inverse lanelets successfull.")
        else:
            rospy.logerr(
                "MapManipulator: Error while Connecting normal and inverse lanelets: Bidirectional scenario is None."
            )

    def _get_max_id_of_lanelet_network(self, network: LaneletNetwork) -> int:
        """
        Finds and returns the highest lanelet id in a given lanelet network.

        :param network: The LaneletNetwork for which the highest lanelet id will be searched.
        :return: The id of the lanelet with the highest lanelet id in network.
        """
        highest_id = 0
        for lane in network.lanelets:
            current_id = lane.lanelet_id
            if current_id > highest_id:
                highest_id = current_id
        return highest_id

    def _change_lanelet_ids_of_inverse_scenario(self):
        """
        Assigns a new unique lanelet id to every lanelet in self.inverse_scenario that is not already taken by
        a lanelet in self.bidirectional_scenario.
        The old and the new id of every lanelet are being mapped onto each other in two maps and stored for later use.
        """
        highest_id = self._get_max_id_of_lanelet_network(network=self.bidirectional_scenario.lanelet_network)
        for lane in self.inverse_scenario.lanelet_network.lanelets:
            old_id = lane.lanelet_id
            new_id = old_id + highest_id + 1
            lane._lanelet_id = new_id
            self._update_lanelet_refs(old_lanelet_id=old_id, new_lanelet_id=lane.lanelet_id)
            self.corresponding_lanelets_old_new[old_id] = new_id
            self.corresponding_lanelets_new_old[new_id] = old_id

    def _update_lanelet_refs(self, old_lanelet_id, new_lanelet_id):
        """
        Changes all references of a lanelet_id in self.inverse_scenario to a new id.

        :param old_lanelet_id: The lanelet id to be updated.
        :param new_lanelet_id: The id the old lanelet id will be updated to.
        """
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

    def _remove_unconnected_refs(self):
        """
        Removes all wrong successor and predecessor references in self.scenario
        by checking the distance between the first/last center-vertices.
        """
        for lane in self.scenario.lanelet_network.lanelets:
            for succ in lane.successor:
                successor = self.scenario.lanelet_network.find_lanelet_by_id(succ)
                if 0.5 < dist(successor.center_vertices[0], lane.center_vertices[-1]):
                    lane.remove_successor(succ)
                    print("Removed successor " + str(succ) + " for lanelet " + str(lane.lanelet_id))
            for pred in lane.predecessor:
                predec = self.scenario.lanelet_network.find_lanelet_by_id(pred)
                if 0.5 < dist(predec.center_vertices[-1], lane.center_vertices[0]):
                    lane.remove_predecessor(predec)
                    print("Removed predecessor " + str(pred) + " for lanelet " + str(lane.lanelet_id))


def main():
    """
    Function that will be executed if the python script is launched.
    """
    manipulator = MapManipulator()

    manipulator._load_scenario()
    # Call your manipulation methods here

    manipulator._generate_cr_file()


if __name__ == "__main__":
    """Main function, calls main()."""
    try:
        main()
    except rospy.ROSInterruptException:
        pass
