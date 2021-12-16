import rospy
from os.path import expanduser

from commonroad.common.file_reader import CommonRoadFileReader


class MapManager:
    @staticmethod
    def get_scenario(rules=True):
        """
        Loads the commonroad scenario with or without traffic rules of town with number map_number

        Args:
            rules (bool): Defines which driving mode the map should be loaded for

        Returns:
            Scenario: CommonRoad-Scenario of current town
        """
        map_name = None
        try:
            map_name = rospy.get_param("/carla/town")
        except KeyError:
            rospy.logerr("MapManager: Town parameter not set.")
            exit(1)
        map_file_name = "DEU_" + map_name + "-1_1_T-1.xml"
        if rules:
            map_file_path = "Maps/Rules/" + map_file_name
        else:
            map_file_path = "Maps/No Rules/" + map_file_name
        scenario, _ = CommonRoadFileReader(expanduser(f"~/.ros/{map_file_path}")).open()
        return scenario
