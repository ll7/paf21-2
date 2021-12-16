from os.path import expanduser

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Circle
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_route_planner.route import Route
from commonroad_route_planner.utility.visualization import obtain_plot_limits_from_reference_path, draw_state

import rospy


class MapManager:
    @staticmethod
    def _get_town_path(rules=True, town=None):
        if town is None:
            town = "UNKNOWN"
            try:
                town = rospy.get_param("/carla/town")
            except KeyError:
                # rospy.logerr("town parameter not set")
                # exit(1)
                town = "Town03"
        rule = "Rules" if rules else "noRules"
        town = f"maps/{rule}/{town}.xml"
        return town

    @staticmethod
    def get_current_scenario():
        pth = MapManager._get_town_path()
        scenario, _ = CommonRoadFileReader(expanduser(f"~/.ros/{pth}")).open()
        return scenario

    @staticmethod
    def get_scenario(town):
        pth = MapManager._get_town_path(town=town)
        scenario, _ = CommonRoadFileReader(expanduser(f"~/.ros/{pth}")).open()
        return scenario

    @staticmethod
    def visualize_route(route: Route, draw_route_lanelets=False, draw_reference_path=False, size_x=10):
        """Visualizes the given route.

        :param route: route object to be visualized
        :param draw_route_lanelets: flag to indicate if the lanelets should be visualized
        :param draw_reference_path: flag to indicate if the reference path should be visualized
        :param size_x: size of the x-axis of the figure
        """
        import matplotlib.pyplot as plt

        # obtain plot limits for a better visualization.
        # we can obtain them through the lanelets or the reference path
        plot_limits = obtain_plot_limits_from_reference_path(route)
        # plot_limits = obtain_plot_limits_from_routes(route)

        # set the figure size and ratio
        ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])

        # instantiate a renderer for plotting
        renderer = MPRenderer(plot_limits=plot_limits, figsize=(size_x, size_x / ratio_x_y))

        # draw scenario and planning problem
        route.scenario.draw(renderer)
        route.planning_problem.goal.draw(renderer)
        # draw the initial state of the planning problem
        draw_state(renderer, route.planning_problem.initial_state)

        # draw lanelets of the route
        if draw_route_lanelets:
            dict_param = {
                "lanelet": {
                    "unique_colors": False,  # colorizes center_vertices and labels of each lanelet differently
                    "draw_stop_line": False,
                    "stop_line_color": "#ffffff",
                    "draw_line_markings": True,
                    "draw_left_bound": False,
                    "draw_right_bound": False,
                    "draw_center_bound": True,
                    "draw_border_vertices": False,
                    "draw_start_and_direction": True,
                    "show_label": False,
                    "draw_linewidth": 1,
                    "fill_lanelet": True,
                    "facecolor": "#469d89",  # color for filling
                    "zorder": 30,  # put it higher in the plot, to make it visible
                    "center_bound_color": "#3232ff",  # color of the found route with arrow
                }
            }

            list_lanelets = []
            for id_lanelet in route.list_ids_lanelets:
                lanelet = route.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)
                list_lanelets.append(lanelet)
            lanelet_network = LaneletNetwork.create_from_lanelet_network(route.scenario.lanelet_network)
            # lanelet_network.traffic_signs = route.scenario.lanelet_network.traffic_signs
            # lanelet_network.traffic_lights = route.scenario.lanelet_network.traffic_signs
            lanelet_network.draw(renderer, draw_params=dict_param)

        # draw reference path with dots
        if draw_reference_path:
            for position in route.reference_path:
                occ_pos = Circle(radius=0.2, center=position)
                occ_pos.draw(renderer, draw_params={"shape": {"circle": {"facecolor": "#ff477e"}}})

        # render and show plot
        renderer.render()

        plt.margins(0, 0)
        plt.show()
