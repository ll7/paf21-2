import matplotlib.pyplot as plt
import time

# import functions to read xml file and visualize commonroad objects
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

# generate path of the file to be opened
# file_path = os.path.join("..", "CR_Test.xml")
file_path = "/home/imech154/paf21-2/maps/Rules/Town03.xml"

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

# plot the scenario for 40 time step, here each time step corresponds to 0.1 second
for i in range(0, 3):
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    # plot the scenario at different time step
    scenario.draw(rnd, draw_params={"time_begin": i})
    # plot the planning problem set
    planning_problem_set.draw(rnd)
    rnd.render()
    plt.show()
    time.sleep(0.3)
