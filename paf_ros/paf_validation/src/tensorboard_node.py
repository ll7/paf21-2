#!/usr/bin/env python
from datetime import datetime

import rospy
import tensorflow as tf
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from paf_messages.msg import PafLogScalar, PafLogText, PafLogImage
import numpy as np


class TensorBoardNode:
    def __init__(self):
        self.path = f"tensorboard/{datetime.now()}"
        self.writer = tf.summary.create_file_writer(self.path)
        self.writer.init()
        self._driven_distance = -1
        self._last_odo_update = None
        self._distance_step = 0
        self._time_step = 0

        rospy.init_node("tensorboard", anonymous=True)
        self.bridge = CvBridge()
        self.t0 = rospy.Time.now().to_time()

        rospy.Subscriber("/paf/paf_validation/tensorboard/scalar", PafLogScalar, self._log_scalar)
        rospy.Subscriber("/paf/paf_validation/tensorboard/text", PafLogText, self._log_text)
        rospy.Subscriber("/paf/paf_validation/tensorboard/image", PafLogImage, self._log_image)
        self._odometry_subscriber: rospy.Subscriber = rospy.Subscriber(
            "/carla/ego_vehicle/odometry", Odometry, self._odometry_updated
        )

        rospy.logwarn(f"tensorboard files saved in ~/.ros/{self.path}")

    def _get_step_from_cur_time(self):
        t1 = rospy.Time.now().to_time()
        n = int((t1 - self.t0) * 10)
        return n

    def _odometry_updated(self, odo: Odometry):
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
        if self._last_odo_update is None:
            return
        step = self._distance_step if msg.step_as_distance else self._time_step
        descr = None if msg.description == "" else msg.description
        with self.writer.as_default():
            tf.summary.scalar(msg.section, msg.value, step=step, description=descr)
        self.writer.flush()

    def _log_text(self, msg: PafLogText):
        if self._last_odo_update is None:
            return
        step = self._distance_step if msg.step_as_distance else self._time_step
        descr = None if msg.description == "" else msg.description
        with self.writer.as_default():
            tf.summary.text(msg.section, msg.value, step=step, description=descr)
        self.writer.flush()

    def _log_image(self, msg: PafLogImage):
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
        rospy.spin()

    def __del__(self):
        self.writer.flush()
        self.writer.close()


if __name__ == "__main__":
    TensorBoardNode().start()
