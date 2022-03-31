#!/usr/bin/env python
from datetime import datetime

import carla
import rospy
import tensorflow as tf
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from paf_messages.msg import PafLogScalar, PafLogText, PafLogImage
import numpy as np


class TensorBoardNode:
    """Publishes text/scalar/image information to a tensorboard instance"""

    def __init__(self):
        """
        Init of subscribers and Tensorboard
        """
        self.path = f"tensorboard/{datetime.now()}"
        self.writer = tf.summary.create_file_writer(self.path)
        self.writer.init()
        self._driven_distance = -1
        self._last_odo_update = None
        self._distance_step = 0
        self._time_step = 0

        client = carla.Client("127.0.0.1", 2000)
        self._world = client.get_world()

        rospy.init_node("tensorboard", anonymous=True)
        self.bridge = CvBridge()
        self.t0 = self._cur_time()

        rospy.Subscriber("/paf/paf_validation/tensorboard/scalar", PafLogScalar, self._log_scalar)
        rospy.Subscriber("/paf/paf_validation/tensorboard/text", PafLogText, self._log_text)
        rospy.Subscriber("/paf/paf_validation/tensorboard/image", PafLogImage, self._log_image)
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self._odometry_updated)

        rospy.logwarn(f"tensorboard files saved in ~/.ros/{self.path}")

    def _cur_time(self):
        """
        Get current simulation time
        :return:
        """
        return self._world.wait_for_tick().timestamp.elapsed_seconds

    def _get_step_from_cur_time(self):
        """
        Get elapsed time from t0 in 0.1 second steps
        :return:
        """
        t1 = self._cur_time()
        n = int((t1 - self.t0) * 10)
        return n

    def _odometry_updated(self, odo: Odometry):
        """
        Odometry update callback
        :param odo: data
        """
        current_speed = np.sqrt(
            odo.twist.twist.linear.x ** 2 + odo.twist.twist.linear.y ** 2 + odo.twist.twist.linear.z ** 2
        )
        cur_time = rospy.Time.now().to_time()
        if self._last_odo_update is not None:
            self._driven_distance += current_speed * (cur_time - self._last_odo_update)
        self._last_odo_update = cur_time
        self._distance_step = int(self._driven_distance)
        self._time_step = self._get_step_from_cur_time()
        # self._current_pose = odo.pose.pose

    def _log_scalar(self, msg: PafLogScalar):
        """
        Log a scalar y value to time or distance x value
        :param msg: data
        """
        if self._last_odo_update is None:
            return
        step = self._distance_step if msg.step_as_distance else self._time_step
        descr = None if msg.description == "" else msg.description
        with self.writer.as_default():
            tf.summary.scalar(msg.section, msg.value, step=step, description=descr)
        self.writer.flush()

    def _log_text(self, msg: PafLogText):
        """
        Log a text to time or distance step value
        :param msg: data
        """

        if self._last_odo_update is None:
            return
        step = self._distance_step if msg.step_as_distance else self._time_step
        descr = None if msg.description == "" else msg.description
        with self.writer.as_default():
            tf.summary.text(msg.section, msg.value, step=step, description=descr)
        self.writer.flush()

    def _log_image(self, msg: PafLogImage):
        """
        Log an ROS-image to time or distance x value (warning: DO NOT DO THIS EVERY FRAME)
        :param msg: data
        """
        if self._last_odo_update is None:
            return
        step = self._distance_step if msg.step_as_distance else self._time_step
        img: np.array = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding="rgb8")
        # new_size = (100, 100)
        # img = cv2.resize(image_bgr, dsize=new_size, interpolation=cv2.INTER_CUBIC)
        img = img.astype(np.uint8)
        img = np.reshape(img, (1,) + img.shape)

        descr = None if msg.description == "" else msg.description
        with self.writer.as_default():
            tf.summary.image(msg.section, img, step, description=descr)
        self.writer.flush()

    @staticmethod
    def start():
        """
        start ros node
        """
        rospy.spin()

    def __del__(self):
        """
        destructor: flush and close opened file streams
        """
        self.writer.flush()
        self.writer.close()


if __name__ == "__main__":
    TensorBoardNode().start()
