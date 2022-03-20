from typing import Tuple, List, Optional

import numpy as np
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_sign import TrafficSignIDGermany

import rospy
from os.path import expanduser

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Circle
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_route_planner.route import Route
from commonroad_route_planner.utility.visualization import obtain_plot_limits_from_reference_path, draw_state

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Point
from paf_messages.msg import Point2D
from tf.transformations import quaternion_from_euler


class MapManager:
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
        return rospy.get_param("/carla/town", None)

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
        on_bridge_lanelets = {
            "Town04": [329, 327, 325, 323, 348, 350, 352, 354],
            "Town05": [437, 436, 435, 426, 429, 432],
        }

        current_map = MapManager.get_map()
        if current_map not in on_bridge_lanelets:
            return False
        return let_id in on_bridge_lanelets[current_map]

    @staticmethod
    def visualize_lp_and_gp(
        local_path_obj, cur_pt: Point, xmin: float = None, xmax: float = None, ymin: float = None, ymax: float = None
    ):
        """
        Visualize local and global path with matplotlib (for debugging planners)
        :param xmin: Optional graph axis limit
        :param xmax: Optional graph axis limit
        :param ymin: Optional graph axis limit
        :param ymax: Optional graph axis limit
        :param local_path_obj: LocalPath object
        :param cur_pt: location to start local plan on (x,y,z)
        """
        from matplotlib import pyplot as plt

        def pts_to_x_y(pts):
            if len(pts) == 0:
                return ([], []), (99999, 99999, -99999, -99999)
            _x = [p.x for p in pts]
            _y = [p.y for p in pts]
            return (_x, _y), (np.min(_x), np.min(_y), np.max(_x), np.max(_y))

        pts_gl_1 = []
        pts_gl_2 = []

        local_path_obj.global_path.as_msg()

        for s in local_path_obj.global_path.route.sections:
            pts_gl_1 += [p for i, p in enumerate(s.points) if i not in s.target_lanes]
            pts_gl_2 += [p for i, p in enumerate(s.points) if i in s.target_lanes]

        pts_loc_2 = local_path_obj.calculate_new_local_path(cur_pt)[0].points
        pts_loc_1 = local_path_obj.sparse_local_path
        sign_positions = [x[0].point for x in local_path_obj.traffic_signals]
        local_path_obj.set_alternate_speed_next_sign(0)

        xy1, minima1 = pts_to_x_y(pts_gl_1)
        xy2, minima2 = pts_to_x_y(pts_gl_2)
        xy3, minima3 = pts_to_x_y(pts_loc_1)
        xy4, minima4 = pts_to_x_y(pts_loc_2)
        xy5, minima5 = pts_to_x_y(sign_positions)
        xy6, minima6 = pts_to_x_y(local_path_obj.debug_pts)

        plt.scatter(*xy2, label="Global Path (target)", s=1)
        plt.scatter(*xy1, label="Global Path (other)", s=1)
        # plt.plot(*xy3, label="Local Path (sparse)")
        plt.plot(*xy4, label="Local Path (dense)")
        plt.scatter(*xy5, label="Traffic Signals (GP)", s=10)

        pts = []
        for i, (signal, index, distance, match) in enumerate(local_path_obj.get_signal_indices()):
            distance = np.round(distance, 1)
            try:
                name = TrafficSignIDGermany(signal.type).name
            except ValueError:
                name = signal.type
            if index < 0:
                plt.annotate(f"NOT{i}:{name}@{distance}m", (match.x, match.y))
                plt.annotate(f"NOT{i}:{name}@{distance}m", (signal.point.x, signal.point.y))
                continue
            # print(
            #     f"{name: <10}\t{distance: <10}{index: <10}"
            #     f"{(np.round(signal.point.x, 1), np.round(signal.point.y, 1))}")
            pts.append(match)
            # plt.annotate(f"{index}:{name}@{distance}m", (signal.point.x, signal.point.y))
            plt.annotate(f"{index}:{name}@{distance}m", (match.x, match.y))

        xy7, minima7 = pts_to_x_y(pts)
        plt.scatter(*xy7, label="Traffic Signals (LP)", s=10)
        if len(xy6[0]) > 0:
            plt.scatter(*xy6, label="Local Path (debug pts)", s=6)

        _, _, x_max, y_max = np.max([minima3, minima4, minima6, minima7], axis=0)
        x_min, y_min, _, _ = np.min([minima3, minima4, minima6, minima7], axis=0)

        if xmin is not None:
            x_min = xmin
        if ymin is not None:
            y_min = ymin
        if xmax is not None:
            x_max = xmax
        if ymax is not None:
            y_max = ymax

        plt.xlim([x_min - 10, x_max + 10])
        plt.ylim([y_min - 10, y_max + 10])
        plt.legend()
        plt.show()
