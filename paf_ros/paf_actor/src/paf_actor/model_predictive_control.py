"""
A file that contains the Stanley Lateral Controller (inspired by PSAF WS20/21 2)
"""
# from struct import error
import rospy

import numpy as np
from geometry_msgs.msg import PoseStamped

from paf_actor.helper_functions import calc_egocar_yaw, normalize_angle, calc_path_yaw
from paf_messages.msg import PafLocalPath


class ModelPredictiveController:
    """
    StanleyLateral Controller implements Lateral control using a stanley controller
    """

    def __init__(self, k: float = 0.5, Kp: float = 1.0, L: float = 2.9, max_steer: float = 30.0, min_speed: float = 10):
        """
        Constructor of the stanley lateral controller.

        Args:
            k (float, optional): Control gain. Defaults to .5.
            Kp (float, optional): Speed proportional gain. Defaults to 1.0.
            L (float, optional): [m] Wheel base of vehicle. Defaults to 2.9.
            max_steer (float, optional): Maximum steering angle (degrees). Defaults to 30.0.
            min_speed (float, optional): Minimum speed used for calculation, to avoid infinite values
            when standing still. Defaults to 10.
        """
        self.k: float = k
        self.Kp: float = Kp
        self.L: float = L
        self.max_steer: float = np.deg2rad(max_steer)
        self.min_speed: float = min_speed

        self.heading_error = 0
        self.cross_err = 0
        self._local_plan_publisher = rospy.Publisher("/paf/paf_actor/path", PafLocalPath, queue_size=1)

    def run_step(
        self, msg: PafLocalPath, pose: PoseStamped, speed: float, steering_rate: float, delta_t: float, is_reverse: bool
    ):
        """
        Runs the Stanley-Controller calculations once

        Args:
            currentPath (Path): Path to follow
            currentPose (PoseStamped): Pose of Ego Vehicle
            currentSpeed (float): speed of ego_vehicle
            is_reverse (bool): sets the stanley controller to steer backwards

        Returns:
           float: Steering angle
        """
        # Input: [v, φ]
        # Output: [v * cos(δ + θ), v * sin(δ + θ), v * sin(δ)/L, φ]

        # x_(t+1) = x_t + x_dot * Δt

        # y_(t+1) = y_t + y_dot * Δt

        # θ_(t+1) = θ_t + θ_dot * Δt

        # δ_(t+1) = δ_t + δ_dot * Δt

        x = pose.position.x
        y = pose.position.y

        # min_cost, best_solution, msg_str = self.get_best_steering_solution(
        #    msg, pose, speed, steering_rate, delta_t, is_reverse, x, y)

        min_cost, best_solution = self.get_best_steering(
            1, 0.0, [0, calc_egocar_yaw(pose), speed, 0, x, y], msg, pose, steering_rate, delta_t, is_reverse, None
        )

        rospy.loginfo_throttle(
            5,
            f"Best steering angle: {np.rad2deg(best_solution[0])}, distance: {best_solution[3]}, \
            for cost: {min_cost}\n",
        )
        self.heading_angle = best_solution[1]
        return best_solution[0], best_solution[2], best_solution[3]

    def get_best_steering_solution(self, msg, pose, speed, steering_rate, delta_t, is_reverse, x, y, heading_angle_new):
        min_cost = float("inf")
        best_solution = None
        heading_angle = heading_angle_new
        speed = max(self.min_speed, speed)
        for steering_angle in np.linspace(-self.max_steer, self.max_steer, 61):
            x_dot = speed * np.cos(heading_angle + steering_angle)
            y_dot = speed * np.sin(heading_angle + steering_angle)
            steering_angle_dot = speed * np.sin(heading_angle) / self.L
            heading_angle_dot = steering_rate

            new_x = x + x_dot * delta_t
            new_y = y + y_dot * delta_t
            steering_angle_new = steering_angle + steering_angle_dot * delta_t
            heading_angle_new = heading_angle + heading_angle_dot * delta_t

            cost, target_speed, distance = self.calculate_cost(
                new_x, new_y, heading_angle_new + steering_angle_new, msg, speed, pose, is_reverse
            )

            if cost < min_cost:
                best_solution = [
                    steering_angle,
                    heading_angle_new + steering_angle_new,
                    target_speed,
                    distance,
                    new_x,
                    new_y,
                ]
                min_cost = cost
        return min_cost, best_solution

    def get_best_steering(
        self, iteration, cost, best_solution, msg, pose, steering_rate, delta_t, is_reverse, best_start_solution
    ):
        if iteration == 0:
            return cost, best_start_solution

        else:
            costs, new_best_solution = self.get_best_steering_solution(
                msg,
                pose,
                best_solution[2],
                steering_rate,
                delta_t,
                is_reverse,
                best_solution[4],
                best_solution[5],
                best_solution[1],
            )

            next_iteration = iteration - 1
            if best_start_solution is None:
                return self.get_best_steering(
                    next_iteration,
                    cost + costs,
                    best_solution,
                    msg,
                    pose,
                    steering_rate,
                    delta_t,
                    is_reverse,
                    new_best_solution,
                )
            else:
                return self.get_best_steering(
                    next_iteration,
                    cost + costs,
                    best_solution,
                    msg,
                    pose,
                    steering_rate,
                    delta_t,
                    is_reverse,
                    best_start_solution,
                )

    def calculate_cost(self, new_x, new_y, yaw, msg, speed, pose, is_reverse):
        pose.position.x = new_x
        pose.position.y = new_y
        current_target_idx, error_front_axle, target_speed, distance = self.calc_target_index(
            msg, pose, yaw, is_reverse
        )

        theta_e = normalize_angle(calc_path_yaw(msg.points, current_target_idx) + (yaw if is_reverse else -yaw))

        theta_d = np.arctan2(self.k * error_front_axle / max([1, 0.4 * speed]), speed)

        cost = abs(theta_e + theta_d)

        return cost, target_speed, distance

    def calc_target_index(self, msg: PafLocalPath, pose: PoseStamped, yaw: float, is_reverse: bool):
        """
        Calculates the index of the closest Point on the Path relative to the front axle

        Args:
            currentPath (LocalPath): Path to follow
            currentPose (PoseStamped): Pose of Ego Vehicle
            is_reverse (bool): bool if we drive backwards

        Returns:
            target_idx [int]: Index of target point
            error_front_axle [float]: Distance from front axle to target point
        """
        path = msg.points
        if len(path) == 0 or len(msg.target_speed) == 0:
            return 0, 0, 0, 0

        # Calc front axle position
        # yaw = calc_egocar_yaw(pose)

        # msg.points = path
        # local_path1 = []
        #
        # pth = PafLocalPath()
        # pth.points = path_g
        # self._local_plan_publisher.publish(pth)

        fx, fy = 0, 0
        if is_reverse:
            fx = pose.position.x - self.L * np.cos(yaw)
            fy = pose.position.y - self.L * np.sin(yaw)
        else:
            fx = pose.position.x + self.L * np.cos(yaw)
            fy = pose.position.y + self.L * np.sin(yaw)

        # Search nearest point index
        px = [posen.x for posen in path]
        py = [posen.y for posen in path]
        dx = [fx - icx for icx in px]
        dy = [fy - icy for icy in py]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)
        distance = d[target_idx]

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(yaw + np.pi / 2), -np.sin(yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle, msg.target_speed[min([target_idx, len(msg.target_speed) - 1])], distance
