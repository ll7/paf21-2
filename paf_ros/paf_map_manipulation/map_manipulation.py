from commonroad.planning.planning_problem import PlanningProblemSet
import rospy

from paf_ros.paf_planning.src.classes.MapManager import MapManager

from commonroad.scenario.scenario import Scenario, Tag
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile


class MapManipulator:
    def __init__(self) -> None:
        self.scenario: Scenario = None
        self.save_path_prefix: str = "paf_ros/paf_map_manipulation/Temporary Maps/"

    def load_scenario(self):
        self.scenario = MapManager.get_current_scenario(True)

    def generate_cr_file(self):
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

    def delete_signs_and_lights(self):
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

    def turn_lanelet_network_bidirectional(self):
        if self.scenario is not None:
            rospy.loginfo("MapManipulator: Turning lanelet network bidirectional...")
            for lane in self.scenario.lanelet_network.lanelets:
                predecessors = lane.predecessor
                successors = lane.successor
                for pre in predecessors:
                    if pre not in lane.successor:
                        lane.add_successor(pre)
                for suc in successors:
                    if suc not in lane.predecessor:
                        lane.add_successor(suc)

            rospy.loginfo("MapManipulator: Turning lanelet network bidirectional successfull!")
        else:
            rospy.logerr("MapManipulator: Error while turning lanelet network bidirectional: Scenario is None.")


def main():
    manipulator = MapManipulator()
    manipulator.load_scenario()

    # modify the map here:
    manipulator.delete_signs_and_lights()
    manipulator.turn_lanelet_network_bidirectional()

    manipulator.generate_cr_file()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
