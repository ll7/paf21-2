from os.path import expanduser

from commonroad.common.file_reader import CommonRoadFileReader

import rospy


class MapManager:
    @staticmethod
    def _get_town_path(rules=True):
        town = "UNKNOWN"
        try:
            town = rospy.get_param("/carla/town")
        except KeyError:
            rospy.logerr("town parameter not set")
            exit(1)
        rule = "Rules" if rules else "noRules"
        town = f"maps/{rule}/{town}.xml"
        return town

    @staticmethod
    def get_current_scenario():
        pth = MapManager._get_town_path()
        scenario, _ = CommonRoadFileReader(expanduser(f"~/.ros/{pth}")).open()
        return scenario
