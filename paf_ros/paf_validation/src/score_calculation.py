#!/usr/bin/env python
import rospy
import numpy as np

from carla_msgs.msg import CarlaCollisionEvent, CarlaLaneInvasionEvent
from nav_msgs.msg import Odometry
from paf_messages.msg import PafLogScalar
from std_msgs.msg import Empty


class ScoreCalculationNode:
    EVENT_THRESHOLD_SECS = 2
    MIN_RUN_TIME = 15
    MAX_RUN_TIME = 300

    def __init__(self):
        rospy.init_node("semantic_lidar", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")
        self._start(init=True)
        rospy.Subscriber(f"/carla/{role_name}/collision", CarlaCollisionEvent, self._process_collision, queue_size=1)
        rospy.Subscriber(
            f"/carla/{role_name}/lane_invasion", CarlaLaneInvasionEvent, self._process_lane_event, queue_size=1
        )
        rospy.Subscriber("/paf/paf_validation/score/start", Empty, self._start, queue_size=1)
        rospy.Subscriber("/paf/paf_validation/score/stop", Empty, self._stop, queue_size=1)
        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated, queue_size=1)

        self._score_actor_collisions_pub = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )
        self._score_other_collisions_pub = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )
        self._score_red_traffic_light_pub = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )
        self._score_speed_limit_overridden_pub = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )
        self._score_wrong_side_of_road_pub = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )
        self._score_crossed_solid_line_pub = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )
        self._score_total_pub = rospy.Publisher("/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1)

    def _start(self, _: Empty = None, init=False):
        rospy.logerr("[score calculation] start recording")
        self._driven_distance = 0
        self._running = not init
        self.t0 = self._cur_time()
        self._last_odo_update = None
        self.actor_collisions = []
        self.other_collisions = []
        self.red_traffic_light = []
        self.wrong_side_of_road = []
        self.speed_limit_overridden = []  # per lanelet
        self.crossed_solid_line = []

    def _odometry_updated(self, odo: Odometry):

        current_speed = np.sqrt(
            odo.twist.twist.linear.x ** 2 + odo.twist.twist.linear.y ** 2 + odo.twist.twist.linear.z ** 2
        )
        cur_time = self._cur_time()
        if self._last_odo_update is not None:
            dt = cur_time - self._last_odo_update
            self._driven_distance += current_speed * dt
        self._last_odo_update = cur_time

    def _stop(self, _: Empty = None):
        if not self._running:
            return
        rospy.logerr("[score calculation] stop recording")
        value, t = self._update_score(True)
        self._running = False

        # if self.MIN_RUN_TIME <= t <= self.MAX_RUN_TIME :
        #     msg = PafLogScalar()
        #     msg.section = "PENALTY standard loop"
        #     msg.value = value
        #     self._score_total_pub.publish(msg)

    def _update_score(self, verbose=False):
        """
        Calculates the current score with the multipliers
        """

        if not self._running:
            return

        t1 = self._cur_time()

        score_time = int(t1 - self.t0)
        score_collision_actor = 120 * len(self.actor_collisions)
        score_collision_other = 30 * len(self.other_collisions)
        score_red_traffic_light = 60 * len(self.red_traffic_light)
        score_wrong_side_of_road = 30 * len(self.wrong_side_of_road)
        score_speed_limit = 20 * len(self.speed_limit_overridden)
        score_crossed_line = 10 * len(self.crossed_solid_line) + 30 * len(self.wrong_side_of_road)
        penalty_total = (
            score_collision_actor
            + score_collision_other
            + score_red_traffic_light
            + score_wrong_side_of_road
            + score_speed_limit
            + score_crossed_line
            + score_time
        )
        if verbose:
            self._print(
                score_time,
                penalty_total,
                len(self.actor_collisions),
                len(self.other_collisions),
                len(self.red_traffic_light),
                len(self.wrong_side_of_road),
                len(self.speed_limit_overridden),
                len(self.crossed_solid_line),
            )

        if self.MIN_RUN_TIME <= score_time <= self.MAX_RUN_TIME:
            try:
                msg = PafLogScalar()
                msg.section = "PENALTY total (time per km)"
                msg.value = penalty_total / self._driven_distance * 1000
                self._score_total_pub.publish(msg)
            except ZeroDivisionError:
                pass

        return penalty_total, score_time

    @staticmethod
    def _sec_format(secs, distance=None):
        if distance is None:
            s = int(secs) % 60
            m_int = int(secs // 60)
            if s < 10:
                s = f"0{s}"
            if m_int < 10:
                m = f"0{m_int}"
            else:
                m = str(m_int)
            if m_int < 60:
                return f"00:{m}:{s}"
            else:
                m = m_int % 60
                h = int(m_int // 60)
                m_int = m
                if m_int < 10:
                    m = f"0{m_int}"
                else:
                    m = str(m_int)
                if h < 10:
                    h = f"0{h}"
                else:
                    h = str(h)
                return f"{h}:{m}:{s}"
        else:
            try:
                return (
                    f"{ScoreCalculationNode._sec_format(secs)} "
                    f"({ScoreCalculationNode._sec_format(secs / distance * 1000)} per km)"
                )
            except (ZeroDivisionError, OverflowError):
                return f"{ScoreCalculationNode._sec_format(secs)}"

    def _print(
        self,
        score_time,
        penalty_total,
        score_collision_actor,
        score_collision_other,
        score_red_traffic_light,
        score_wrong_side_of_road,
        score_speed_limit,
        score_crossed_line,
    ):
        score_time = self._sec_format(score_time, self._driven_distance)
        penalty_total = self._sec_format(penalty_total, self._driven_distance)
        rospy.logwarn(
            f"[score calc] Time with penalties: {penalty_total}, "
            f"Time: {score_time}, Penalties: CollActor={score_collision_actor}, "
            f"CollOther={score_collision_other}, RedLight={score_red_traffic_light}, "
            f"RoadSide={score_wrong_side_of_road}, Speed={score_speed_limit}, Line={score_crossed_line}"
        )

    @staticmethod
    def _cur_time():
        return rospy.get_time()
        # self._world.wait_for_tick(1).timestamp.elapsed_seconds  # todo wait for tick retrieves a carla snapshot

    def _process_collision(self, msg: CarlaCollisionEvent):
        """
        Processes a CarlaCollisionEvent. Repeated events are ignored if within the threshold time.
        :param msg:
        """

        if not self._running:
            return

        try:
            if msg.other_actor_id == 0:
                last_event, last_time = self.other_collisions[-1]
            else:
                last_event, last_time = self.actor_collisions[-1]
            if last_event.other_actor_id != msg.other_actor_id:
                raise IndexError
            cur_time = msg.header.stamp.to_time()
            if cur_time - last_time < self.EVENT_THRESHOLD_SECS:
                if msg.other_actor_id == 0:
                    self.other_collisions[-1] = (last_event, cur_time)
                else:
                    self.actor_collisions[-1] = (last_event, cur_time)
            else:
                raise IndexError
        except IndexError:

            msgout = PafLogScalar()
            msgout.section = "PENALTY "

            if msg.other_actor_id == 0:
                self.other_collisions.append((msg, msg.header.stamp.to_time()))
                rospy.logerr(f"[score calculation] Collision with environment detected ({self._driven_distance:.1f}m)")
                try:
                    msgout.value = len(self.other_collisions) / self._driven_distance * 1000
                    msgout.section += "environment collision (count per km)"
                    self._score_other_collisions_pub.publish(msgout)
                except ZeroDivisionError:
                    pass
            else:
                self.actor_collisions.append((msg, msg.header.stamp.to_time()))
                rospy.logerr(f"[score calculation] Collision with actor detected ({self._driven_distance:.1f}m)")
                try:
                    msgout.value = len(self.actor_collisions) / self._driven_distance * 1000
                    msgout.section += "actor collision (count per km)"
                    self._score_actor_collisions_pub.publish(msgout)
                except ZeroDivisionError:
                    pass
            self._update_score()

    def _process_lane_event(self, msg: CarlaLaneInvasionEvent):
        """
        Processes a CarlaLaneInvasionEvent. Repeated events are ignored if within the threshold time.
        Other line types than solid lines will be discarded
        :param msg:
        """

        if not self._running:
            return

        if CarlaLaneInvasionEvent.LANE_MARKING_SOLID not in msg.crossed_lane_markings:
            return
        try:
            last_event, last_time = self.crossed_solid_line[-1]
            cur_time = msg.header.stamp.to_time()
            if cur_time - last_time < self.EVENT_THRESHOLD_SECS:
                self.crossed_solid_line[-1] = (last_event, cur_time)
                return
        except IndexError:
            pass
        self.crossed_solid_line.append((msg, msg.header.stamp.to_time()))
        self._update_score()

        msg = PafLogScalar()
        msg.section = "PENALTY line crossed (count per km)"
        try:
            msg.value = len(self.crossed_solid_line) / self._driven_distance * 1000
            self._score_crossed_solid_line_pub.publish(msg)
        except ZeroDivisionError:
            pass
        rospy.logerr(f"[score calculation] Crossed solid line ({self._driven_distance:.1f}m)")

    @staticmethod
    def start():
        rospy.spin()


if __name__ == "__main__":
    node = ScoreCalculationNode()
    node.start()
