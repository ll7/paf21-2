from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.common.file_reader import CommonRoadFileReader


class MapManager:
    def __init__(self, init_rospy: bool = False):

        self.map_file_path: str = None

        self.scenario: Scenario = None
        self.planning_problem_set: PlanningProblemSet = None

    def _load_scenario(self, map_number: int, rules: bool):
        """
        Loads the commonroad scenario with or without traffic rules of town with number map_number
        """
        map_files = {
            1: "DEU_Town01-1_1_T-1.xml",
            2: "DEU_Town02-1_1_T-1.xml",
            3: "DEU_Town03-1_1_T-1.xml",
            4: "DEU_Town04-1_1_T-1.xml",
            5: "DEU_Town05-1_1_T-1.xml",
            6: "DEU_Town06-1_1_T-1.xml",
            7: "DEU_Town07-1_1_T-1.xml",
            10: "DEU_Town10HD-1_1_T-1.xml",
        }

        if rules:
            self.map_file_path = "Maps/Rules/" + map_files.get(map_number)
        else:
            self.map_file_path = "Maps/No Rules/" + map_files.get(map_number)

        # read in the scenario and planning problem set
        self.scenario, self.planning_problem_set = CommonRoadFileReader(self.map_file_path).open()

    def get_scenario(self):
        return self.scenario
