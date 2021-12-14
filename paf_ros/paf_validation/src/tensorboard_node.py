#!/usr/bin/env python
from datetime import datetime

import rospy
import tensorflow as tf
from cv_bridge import CvBridge
from paf_messages.msg import PafLogScalar, PafLogText, PafLogImage
import numpy as np


class TensorBoardNode:
    def __init__(self):
        self.path = f"tensorboard/{datetime.now()}"
        self.writer = tf.summary.create_file_writer(self.path)
        self.writer.init()

        rospy.init_node("tensorboard", anonymous=True)
        self.bridge = CvBridge()
        self.t0 = rospy.Time.now().to_time()

        rospy.Subscriber("/paf/paf_validation/tensorboard/scalar", PafLogScalar, self._log_scalar)
        rospy.Subscriber("/paf/paf_validation/tensorboard/text", PafLogText, self._log_text)
        rospy.Subscriber("/paf/paf_validation/tensorboard/image", PafLogImage, self._log_image)

        rospy.logwarn(f"tensorboard files saved in ~/.ros/{self.path}")

    def _get_step_from_cur_time(self):
        t1 = rospy.Time.now().to_time()
        n = int(t1 - self.t0)
        return n

    def _log_scalar(self, msg: PafLogScalar):
        rospy.logwarn_throttle(10, "logging float")
        step = msg.step if msg.step >= 0 else self._get_step_from_cur_time()
        descr = None if msg.description == "" else msg.description
        with self.writer.as_default():
            tf.summary.scalar(msg.section, msg.value, step=step, description=descr)
        self.writer.flush()

    def _log_text(self, msg: PafLogText):
        rospy.logwarn_throttle(10, "logging text")
        step = msg.step if msg.step >= 0 else self._get_step_from_cur_time()
        descr = None if msg.description == "" else msg.description
        with self.writer.as_default():
            tf.summary.text(msg.section, msg.value, step=step, description=descr)
        self.writer.flush()

    def _log_image(self, msg: PafLogImage):
        rospy.logwarn_throttle(10, "logging img")
        step = msg.step if msg.step >= 0 else self._get_step_from_cur_time()
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
