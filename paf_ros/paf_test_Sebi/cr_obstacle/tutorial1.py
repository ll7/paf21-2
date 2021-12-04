
import os
#import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

# load the exemplary CommonRoad scenario using the CommonRoad file reader
scenario, planning_problem_set = CommonRoadFileReader('DEU_Hhr-1_1.xml').open()

"""# plot the scenario
rnd = MPRenderer(figsize=(25, 10))
scenario.draw(rnd)
planning_problem_set.draw(rnd)
rnd.render()"""
