from typing import Tuple, List, Optional

import numpy as np
from commonroad.scenario.scenario import Scenario

import rospy
from os.path import expanduser

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Circle
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_route_planner.route import Route
from commonroad_route_planner.utility.visualization import obtain_plot_limits_from_reference_path, draw_state

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from paf_messages.msg import Point2D
from tf.transformations import quaternion_from_euler


class MapManager:
    """
    Class for loading and managing Commonroad Scenarios and Carla Maps at runtime
    """

    last_map = None

    @staticmethod
    def get_current_scenario(rules: bool = None, map_name: str = None) -> Scenario:
        """
        Loads the commonroad scenario with or without traffic rules of town with number map_number
        :param rules: Defines which driving mode the map should be loaded for
        :param map_name: Defines the name of the town. None loads the current carla town
        :return: CommonRoad-Scenario of current town
        """
        if rules is None:
            rules = MapManager.get_rules_enabled()
        if map_name is None:
            try:
                map_name = rospy.get_param("/carla/town")
            except ConnectionRefusedError:
                map_name = "Town04"

        if map_name is None:
            rospy.logerr("MapManager: Town parameter not set.")
            exit(1)

        MapManager.last_map = map_name
        map_file_name = "DEU_" + map_name + "-1_1_T-1.xml"
        if rules:
            map_file_path = "Maps/Rules/" + map_file_name
        else:
            map_file_path = "Maps/No Rules/" + map_file_name
        scenario, _ = CommonRoadFileReader(expanduser(f"~/.ros/{map_file_path}")).open()
        scenario.lanelet_network = MapManager.remove_u_turns(scenario.lanelet_network)
        return scenario

    @staticmethod
    def remove_u_turns(network: LaneletNetwork):
        """
        Remove adjacent lanes from CR-Lanelet-Network in opposite direction to remove U-turns from route calculation
        :param network: input network
        :return: modified network
        """
        for let in network.lanelets:
            if let.adj_left is not None and not let.adj_left_same_direction:
                let._adj_left = None
                let._adj_left_same_direction = None
            if let.adj_right is not None and not let.adj_right_same_direction:
                let._adj_right = None
                let._adj_right_same_direction = None
        return network

    @staticmethod
    def get_map() -> str:
        """
        Get current map from carla parameters
        :return: town name in format "TownX"
        """
        try:
            return rospy.get_param("/carla/town", None)
        except ConnectionRefusedError:
            print("WARN: unable to access '/carla/town'")
            print(MapManager.last_map)
            return MapManager.last_map

    @staticmethod
    def point_to_pose(pt: Tuple[float, float], yaw_deg: float, height=2) -> PoseWithCovarianceStamped:
        """
        Convert a point and a yaw value to a teleport position
        :param pt: teleporting target
        :param yaw_deg: teleporting angle in degrees (counterclockwise)
        :param height: height to spawn over sea leve(optional) if on bridge, set higher
        :return: teleport pose
        """
        initial_pose = PoseWithCovarianceStamped()
        x, y = pt
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = height
        x, z, y, w = quaternion_from_euler(0, np.deg2rad(yaw_deg + 90), 0)
        initial_pose.pose.pose.orientation = Quaternion(x, y, z, w)
        return initial_pose

    @staticmethod
    def light_is_opposite_stop_point():
        """
        Get information about traffic light placement
        :return: True if traffic lights are at opposite sides of the intersection (or not connected to ROS)
        """
        try:
            return MapManager.get_map() not in ["Town01", "Town02"]
        except ConnectionRefusedError:
            return True

    @staticmethod
    def get_demo_route() -> Tuple[Optional[PoseWithCovarianceStamped], Optional[List[Point2D]]]:
        """
        Get Standard Loop route on request
        :return: start pose, list of points
        """
        town = MapManager.get_map()

        # uncomment for custom path debugging
        # if "Town" in town:
        #     return MapManager.point_to_pose((-23, -134), -90), [
        #         Point2D(122, 201.6),
        #     ]

        if town == "Town01":
            return MapManager.point_to_pose((398, -61), 0), [
                Point2D(125, -55),
                Point2D(245, -60),
                Point2D(32, -325),
            ]
        elif town == "Town02":
            return MapManager.point_to_pose((-3.5, -159), 0), [
                Point2D(84, -237),
                Point2D(193.5, -213.5),
                Point2D(-3.5, -220.5),
            ]
        elif town == "Town03":
            return MapManager.point_to_pose((-6, -159), 180), [
                Point2D(-49, -135),
                Point2D(122, 201.6),
                Point2D(-7, 43.5),
                Point2D(8.8, 84),
                Point2D(-149, -38),
            ]
        elif town == "Town04":
            return MapManager.point_to_pose((389, 45), 180), [
                Point2D(64, -14),
                Point2D(-454, -18),
                Point2D(10, -243),
                Point2D(54, 326),
                Point2D(388, 199),
            ]
        elif town == "Town05":
            return MapManager.point_to_pose((211, -94), 0), [
                Point2D(151.6, 28.4),
                Point2D(-161.5, 143),
                Point2D(-276, -47.6),
                Point2D(40.4, -208.6),
            ]
        else:
            return None, None

    @staticmethod
    def get_rules_enabled() -> bool:
        """
        get rules enabled parameter
        :return: boolean
        """
        try:
            return rospy.get_param("rules_enabled", False)
        except ConnectionRefusedError:
            return False

    @staticmethod
    def visualize_route(route: Route, draw_route_lanelets=False, draw_reference_path=False, size_x=10):
        """
        Visualizes the given commonroad route with matplotlib.
        (NOT working for LocalPath and GlobalPath objects, use LocalPath.visualize function instead)

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
                    # colorizes center_vertices and labels of each lanelet differently
                    "unique_colors": False,
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

    @staticmethod
    def lanelet_on_bridge(let_id):
        """
        Tests for on-bridge-lanelets
        :param let_id: any lanelet id
        :return: True, if it is marked as "on bridge"
        """
        on_bridge_lanelets = {
            "Town04": [329, 327, 325, 323, 348, 350, 352, 354],
            "Town05": [437, 436, 435, 426, 429, 432],
        }

        current_map = MapManager.get_map()
        if current_map not in on_bridge_lanelets:
            return False
        return let_id in on_bridge_lanelets[current_map]
