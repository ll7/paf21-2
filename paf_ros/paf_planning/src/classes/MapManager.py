from commonroad.common.file_reader import CommonRoadFileReader

import rospy


class MapManager:
    @staticmethod
    def _get_town_path(rules=True):
        town = "UNKNOWN"
        try:
            town = rospy.get_param("/town")
        except KeyError:
            rospy.logerr("town parameter not set")
            exit(1)
        rule = "Rules" if rules else "noRules"
        town = f"maps/{rule}/{town}.xml"
        return town

    @staticmethod
    def get_current_scenario():
        scenario, _ = CommonRoadFileReader(MapManager._get_town_path()).open()
        return scenario
