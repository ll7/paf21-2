import math
from typing import Callable, Set

import cv2
import numpy
import numpy as np
import rospy
from cv_bridge import CvBridge

from std_msgs.msg import Time
from SegmentationCamera import Tag as SegmentationTag, SegmentationCamera
from DepthCamera import DepthCamera
from RGBCamera import RGBCamera


class FusionCamera:
    """
    Abstraction layer for a fusion camera
    """

    def __init__(
        self,
        role_name: str = "ego_vehicle",
        camera_name: str = "front",
        visible_tags: Set[SegmentationTag] = None,
        queue_size=1,
    ):
        # 2d image with distance in meters max 1000

        self.time_diff = rospy.Time.from_sec(0.05)
        self.rgb_image_list = {}
        self.depth_image_list = {}

        self.segmentation_image = None
        self.rgb_image = None
        self.depth_image = None

        self.segmentation_time = None
        self.depth_time = None
        self.rgb_time = None

        self.segmentation_cam = SegmentationCamera()
        self.depth_cam = DepthCamera()
        self.rgb_cam = RGBCamera()

        self.segmentation_cam.set_on_image_listener(self.__on_segmentation_image_callback)
        self.depth_cam.set_on_image_listener(self.__on_depth_image_callback)
        self.rgb_cam.set_on_image_listener(self.__on_rgb_image_callback)

        self.visible_tags = visible_tags

        self.__listener = None
        self.bridge = CvBridge()

    def __on_segmentation_image_callback(self, image, time_stamp):
        self.segmentation_image = image
        self.segmentation_time = time_stamp
        min_time_diff_key_rgb = None
        min_time_diff_rgb = math.inf
        for k in self.rgb_image_list.keys():
            current_time_diff = abs(self.segmentation_time.to_nsec() - k.to_nsec())
            if current_time_diff < min_time_diff_rgb:
                min_time_diff_rgb = current_time_diff
                min_time_diff_key_rgb = k
        min_time_diff_key_dep = None
        min_time_diff_dep = math.inf
        for k in self.depth_image_list.keys():
            current_time_diff = abs(self.segmentation_time.to_nsec() - k.to_nsec())
            if current_time_diff < min_time_diff_dep:
                min_time_diff_dep = current_time_diff
                min_time_diff_key_dep = k
        if min_time_diff_rgb > self.time_diff.to_nsec():
            self.rgb_image = None
        else:
            self.rgb_image = self.rgb_image_list[min_time_diff_key_rgb]
            self.rgb_time = min_time_diff_key_rgb
        if min_time_diff_dep > self.time_diff.to_nsec():
            self.depth_image = None
        else:
            self.depth_image = self.depth_image_list[min_time_diff_key_dep]
            self.depth_time = min_time_diff_key_dep
        if self.segmentation_image is not None and self.depth_image is not None and self.rgb_image is not None:
            # print("Time stamp seg: " + str(self.segmentation_time.to_sec()))
            # print("Time stamp rgb: " + str(self.rgb_time.to_sec()))
            # print("Time stamp dep: " + str(self.depth_time.to_sec()))
            # print("Time diff seg and rgb: " + str(self.segmentation_time.to_sec() - self.rgb_time.to_sec()) +
            # " | Time Diff seg and depth: " + str(self.segmentation_time.to_sec() - self.depth_time.to_sec()))
            self.__update_image()
            for k in list(self.rgb_image_list.keys()):
                if k.to_nsec() < self.segmentation_time.to_nsec():
                    del self.rgb_image_list[k]
            for k in list(self.depth_image_list.keys()):
                if k.to_nsec() < self.segmentation_time.to_nsec():
                    del self.depth_image_list[k]
        for k in list(self.rgb_image_list.keys()):
            if abs(k.to_nsec() - rospy.Time.now().to_nsec()) > 3:
                del self.rgb_image_list[k]
        for k in list(self.depth_image_list.keys()):
            if abs(k.to_nsec() - rospy.Time.now().to_nsec()) > 3:
                del self.depth_image_list[k]

    def __on_rgb_image_callback(self, image, time_stamp):
        self.rgb_image_list[time_stamp] = image

    def __on_depth_image_callback(self, image, time_stamp):
        self.depth_image_list[time_stamp] = image

    def __update_image(self):
        """
        Internal method to update the distance data
        :return: None
        """

        # age = rospy.Time.now() - self.segmentation_time

        if self.visible_tags is not None:
            self.segmentation_image = SegmentationCamera.filter_for_tags(self.segmentation_image, self.visible_tags)

        if self.__listener is not None:
            self.__listener(self.segmentation_time, self.segmentation_image, self.rgb_image, self.depth_image)

    def get_image(self):
        """
        Return the current depth image
        :return:the current image
        """
        return self.position

    def set_on_image_listener(self, func: Callable[[Time, numpy.ndarray, numpy.ndarray, numpy.ndarray], None]):
        """
        Set function to be called with the time, segmentation, rgb and depth image as parameter
        :param func: the function
        :return: None
        """
        self.__listener = func


# Show case code
def show_image(title, image):
    max_width, max_height = 1200, 800

    limit = (max_height, max_width)
    fac = 1.0
    if image.shape[0] > limit[0]:
        fac = limit[0] / image.shape[0]
    elif image.shape[1] > limit[1]:
        fac = limit[1] / image.shape[1]
    image = cv2.resize(image, (int(image.shape[1] * fac), int(image.shape[0] * fac)))
    # show the output image
    cv2.imshow(title, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("FusionCameraService")

    def store_image(time_stamp, seg_image, rgb_image, depth_image):

        age = rospy.Time.now() - time_stamp
        print(f"Age {age.to_sec()}s")
        # Create one big image
        image = np.vstack(
            (
                seg_image,
                rgb_image,
                cv2.cvtColor((depth_image / DepthCamera.MAX_METERS * 255).astype("uint8"), cv2.COLOR_GRAY2BGR),
            )
        )
        show_image("Fusion", image)
        import time

        time.sleep(0.2)

    sensor = FusionCamera(queue_size=1)
    sensor.set_on_image_listener(store_image)
    rospy.spin()
