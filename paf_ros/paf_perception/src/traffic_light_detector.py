#!/usr/bin/env python
from enum import Enum
import json
import math
import os
from datetime import datetime
from typing import Callable, List, Tuple, Optional
from std_msgs.msg import Bool
from paf_messages.msg import PafDetectedTrafficLights, Point2D

import cv2
import numpy as np
import rospkg
import rospy
import torch
from torch.autograd import Variable
from torchvision.transforms import transforms
import torch.backends.cudnn as cudnn

from FusionCamera import FusionCamera, SegmentationTag


class Labels(Enum):
    """
    Enum that stores the possible traffic light states
    Source: PAF 2020/21 group 1 (AbstractDetector.py), adapted
    """

    def __init__(self, nr: int, label: str):
        """Source: PAF 2020/21 group 1"""
        self._value_ = nr
        self._label = label

    @property
    def label_text(self) -> str:
        """
        Returns the label name
        Source: PAF 2020/21 group 1

        :return: Label name
        """
        return self._label

    def __str__(self):
        """Source: PAF 2020/21 group 1"""
        return self.label_text

    TrafficLightUnknown = (20, "unknown")
    TrafficLightRed = (21, "red")
    TrafficLightYellow = (22, "yellow")
    TrafficLightGreen = (23, "green")

    Other = (99, "Other")


class DetectedObject:
    """
    Data class for detected elements
    Source: PAF 2020/21 group 1 (AbstractDetector.py), adapted
    """

    def __init__(
        self,
        x: float = 0,
        y: float = 0,
        width: float = 0,
        height: float = 0,
        distance: float = 0,
        label: Labels = 0,
        confidence: float = 0.01,
    ):
        """
        Source: PAF 2020/21 group 1

        :param x: relative x coord in image
        :param y: relative y coord in image
        :param height: relative height of bounding box
        :param width: relative weight of bounding box
        :param distance: distance in meters. 0 (default) stands for an unknown distance
        :param label: the class label
        :param confidence: the confidence value
        """
        self.x = x
        self.y = y
        self.h = height
        self.w = width
        self.distance = distance
        self.label = label
        self.confidence = confidence

    def __str__(self):
        """Source: PAF 2020/21 group 1"""
        return f"x={self.x},y={self.y},label={self.label}"


class TrafficLightDetector:
    """
    Detector for traffic lights
    Source: PAF 2020/21 group 1, altered and adapted
    """

    def __init__(self, use_gpu: bool = True):
        """
        Init the speed traffic light detector
        Source: PAF 2020/21 group 1, altered to be independet rosnode and
        added functionalities (Publisher/Subscriber/(De-)Activation)

        :param use_gpu: whether the classification model should be loaded to the gpu
        """

        self.map_name = None

        if self.map_name is None:
            try:
                self.map_name = rospy.get_param("/carla/town")
            except ConnectionRefusedError:
                self.map_name = "Town03"

        self.traffic_light_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_perception/detected_traffic_lights", PafDetectedTrafficLights, queue_size=1
        )

        self.detection_activated = True

        self.__listener = None

        self.confidence_min = 0.75
        self.threshold = 0.7
        self.canny_threshold = 100

        self.data_collect_path = None  # "/home/psaf1/Documents/traffic_light_data"

        rospack = rospkg.RosPack()
        root_path = rospack.get_path("paf_perception")

        # Deep learning config and inits
        self.confidence_min = 0.70
        self.threshold = 1.0
        rospy.loginfo(f"[traffic light detector] init device (use gpu={use_gpu})")
        # select the gpu if allowed and a gpu is available
        use_gpu = use_gpu and torch.cuda.is_available()
        self.device = torch.device("cuda:0" if use_gpu else "cpu")
        rospy.loginfo("[traffic light detector] Device: " + str(self.device))
        # load our model
        model_name = "traffic-light-classifiers-2021-03-27_23-55-48"
        rospy.loginfo("[traffic light detector] loading classifier model from disk...")
        map_location = None
        if not use_gpu:
            map_location = "cpu"
        model = torch.load(os.path.join(root_path, f"models/{model_name}.pt"), map_location=map_location)

        with open(os.path.join(root_path, f"models/{model_name}.names")) as f:
            class_names = json.load(f)
        self.labels = {
            class_names["back"]: None,
            class_names["green"]: Labels.TrafficLightGreen,
            class_names["red"]: Labels.TrafficLightRed,
            class_names["yellow"]: Labels.TrafficLightYellow,
        }
        self.transforms = transforms.Compose(
            [
                transforms.ToPILImage(),
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        model.to(self.device)
        if self.device.type == "cuda":
            rospy.loginfo("[traffic light detector] Enable cudnn")
            cudnn.enabled = True
            cudnn.benchmark = True
        self.net = model
        self.net.eval()
        torch.no_grad()  # reduce memory consumption and improve speed

        activation_topic = "/paf/paf_local_planner/activate_traffic_light_detection"
        rospy.Subscriber(activation_topic, Bool, self._activation_toggled, queue_size=1)

        # init image source = combination of segmentation, rgb and depth camera
        self.combinedCamera = FusionCamera(visible_tags={SegmentationTag.TrafficLight})
        self.combinedCamera.set_on_image_listener(self.__on_new_image_data)

        rospy.loginfo("[traffic light detector] initialization successful")

    def _activation_toggled(self, activate: Bool):
        """
        Turns traffic light detection on/off.

        :param activate: Bool that determines whether detection is activated or deactivated.
        """
        self.detection_activated = activate.data
        if activate.data:
            rospy.loginfo_throttle(10, "[traffic light detector] Detection activated")
        else:
            rospy.loginfo_throttle(10, "[traffic light detector] Detection deactivated")

    def __extract_label(self, image) -> Tuple[Labels, float]:
        """
        Analyze the given image of the traffic light and returns the corresponding label
        Source: PAF 2020/21 group 1

        :param image: the important part of the camera image
        :return: the Label
        """
        # global times
        # import time
        # start = time.time_ns()
        image = self.transforms(image).unsqueeze(dim=0)
        imgblob = Variable(image).to(self.device)
        pred = torch.nn.functional.softmax(self.net(imgblob).cpu(), dim=1).detach().numpy()[0, :]

        hit = np.argmax(pred)
        label = self.labels.get(hit, Labels.TrafficLightUnknown)
        # end = time.time_ns()
        # times.append((end-start)/10e6)
        # if len(times)>10000:
        #     times.pop(0)
        return label, float(pred[hit])

    def __on_new_image_data(self, time, segmentation_image, rgb_image, depth_image):
        """
        Handles the new image data from the cameras.
        Detects and classifies traffic lights in the image and
        publishes the results as PafDetectedTrafficLights-Message.
        Source: PAF 2020/21 group 1, altered for (de-)activation, handling of town04 and town 10 and more.

        :param segmentation_image: the segmentation image
        :param rgb_image: the rgb image
        :param depth_image: the depth data
        :param time: time stamp when the images where taken
        :return: none
        """
        if not self.detection_activated:
            rospy.loginfo_throttle(
                5, "[traffic light detector] skipping image detection, because detection is disabled"
            )
            self.inform_listener(time, None)
            return
        if (rospy.Time.now() - time).to_sec() > 0.6:
            # Ignore the new image data if its older than 600ms
            # -> Reduce message aging
            return
        (height_seg, width_seg) = segmentation_image.shape[:2]
        (height_rgb, width_rgb) = rgb_image.shape[:2]

        h_scale = width_rgb / width_seg
        v_scale = height_rgb / height_seg

        # List of detected elements
        detected = []

        canny_output = cv2.Canny(segmentation_image, self.canny_threshold, self.canny_threshold * 2)
        contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        classes = []
        confidences = []
        distances = []

        for i, c in enumerate(contours):
            contours_poly = cv2.approxPolyDP(c, 3, True)
            x1, y1, w, h = cv2.boundingRect(contours_poly)
            x2 = x1 + w
            y2 = y1 + h
            if self.map_name == "Town04":
                self.confidence_min = 0.54
                if w > 7:
                    x1 += 3
                    x2 -= 3
                if h > 7:
                    y1 += 3
                    y2 -= 3
            if w > 1 and h > 2:
                mask = segmentation_image[y1:y2, x1:x2] != (255, 255, 255)

                # get cropped depth image
                crop_depth = depth_image[y1:y2, x1:x2]
                masked_crop_depth = crop_depth[mask[:, :, 1]]

                if np.size(masked_crop_depth) < 1:
                    continue
                # use mask to extract the traffic sign distances
                distance = np.min(masked_crop_depth)

                if distance <= 100:
                    # get cropped rgb image
                    crop_rgb = rgb_image[
                        int(y1 * v_scale) : min([height_rgb, int(y2 * v_scale + 1)]),
                        max([int(x1 * h_scale - 5), 0]) : min([width_rgb, int(x2 * h_scale + 1)]),
                        :,
                    ]
                    if self.map_name == "Town10HD":
                        self.confidence_min = 0.45
                        crop_rgb_hsv = cv2.cvtColor(crop_rgb, cv2.COLOR_RGB2HSV)
                        # lower boundary RED color range values; Hue (0 - 10)
                        lower_red_1 = np.array([0, 100, 20])
                        upper_red_1 = np.array([10, 255, 255])
                        # upper boundary RED color range values; Hue (160 - 180)
                        lower_red_2 = np.array([160, 100, 20])
                        upper_red_2 = np.array([179, 255, 255])
                        # violet RED color range
                        lower_violet = np.array([170, 70, 50], dtype="uint8")
                        upper_violet = np.array([180, 255, 240], dtype="uint8")
                        lower_mask_red = cv2.inRange(crop_rgb_hsv, lower_red_1, upper_red_1)
                        upper_mask_red = cv2.inRange(crop_rgb_hsv, lower_red_2, upper_red_2)
                        red_mask_violet = cv2.inRange(crop_rgb_hsv, lower_violet, upper_violet)
                        # GREEN color range
                        lower_green = np.array([36, 0, 0], dtype="uint8")
                        upper_green = np.array([86, 255, 255], dtype="uint8")
                        green_mask = cv2.inRange(crop_rgb_hsv, lower_green, upper_green)
                        full_mask = lower_mask_red + upper_mask_red + red_mask_violet + green_mask
                        # Turn image to grayscale
                        h1, s1, v1 = cv2.split(crop_rgb_hsv)
                        h0 = np.zeros_like(h1)
                        s0 = np.zeros_like(s1)
                        crop_rgb_grayscale = cv2.merge([h0, s0, v1])
                        gray_masked = cv2.bitwise_and(crop_rgb_grayscale, crop_rgb_grayscale, mask=255 - full_mask)
                        colors = cv2.bitwise_and(crop_rgb_hsv, crop_rgb_hsv, mask=full_mask)
                        crop_rgb = cv2.cvtColor(gray_masked + colors, cv2.COLOR_HSV2RGB)
                        # To display the various steps with the code below, the debug screen needs to be deactivated!
                        # cv2.imshow("crop_rgb_hsv", cv2.cvtColor(crop_rgb_hsv, cv2.COLOR_HSV2BGR))
                        # cv2.waitKey(1)
                        # cv2.imshow("full_mask", full_mask)
                        # cv2.waitKey(1)
                        # cv2.imshow("rgb_grayscale", crop_rgb_grayscale)
                        # cv2.waitKey(1)
                        # cv2.imshow("gray_masked", gray_masked)
                        # cv2.waitKey(1)
                        # cv2.imshow("crop_rgb_after", cv2.cvtColor(crop_rgb, cv2.COLOR_RGB2BGR))
                        # cv2.waitKey(1)
                    # Classify the cropped image
                    label, confidence = self.__extract_label(crop_rgb)
                    if label is not None:
                        if self.map_name == "Town10HD" and (w > h or x2 < 750):
                            pass
                        else:
                            boxes.append(
                                np.array([x1, y1, w, h]) / np.array([width_seg, height_seg, width_seg, height_seg])
                            )
                            classes.append(label)
                            confidences.append(confidence)
                            distances.append(distance)

        # apply non-maxima suppression to suppress weak, overlapping bounding boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_min, self.threshold)

        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                distance = distances[i]
                label = classes[i]
                confidence = confidences[i]

                if confidence < self.confidence_min:
                    label = Labels.TrafficLightUnknown
                    confidence = 1.0

                keep_object = True
                diff_threshold = 0.005
                x = boxes[i][0]
                y = boxes[i][1]
                for j in range(0, len(detected)):
                    x2 = detected[j].x
                    y2 = detected[j].y
                    diff = math.dist([x, y], [x2, y2])
                    if diff < diff_threshold:
                        keep_object = False
                        break

                if keep_object:
                    detected.append(
                        DetectedObject(
                            x=boxes[i][0],
                            y=boxes[i][1],
                            width=boxes[i][2],
                            height=boxes[i][3],
                            distance=distance,
                            label=label,
                            confidence=confidence,
                        )
                    )
                    # Store traffic light data in folder to train a better network
                    if self.data_collect_path is not None and distance < 25:
                        x1, y1, w, h = boxes[i]
                        x1 = int(x1 * width_rgb)
                        y1 = int(y1 * height_rgb)
                        x2 = x1 + int(w * width_rgb)
                        y2 = y1 + int(h * height_rgb)
                        # get cropped rgb image
                        crop_rgb = rgb_image[y1:y2, x1:x2, :]
                        now = datetime.now().strftime("%H:%M:%S")
                        folder = os.path.abspath(
                            f"{self.data_collect_path}/{label.name if label is not None else 'unknown'}"
                        )
                        if not os.path.exists(folder):
                            os.mkdir(folder)
                        cv2.imwrite(os.path.join(folder, f"{now}-{i}.jpg"), cv2.cvtColor(crop_rgb, cv2.COLOR_RGB2BGR))

        states = []
        distances = []
        positions = []
        rospy.loginfo_throttle(5, f"[traffic light detector] publishing {len(detected)} detected lights.")
        if len(detected) > 0:
            for d in detected:
                states.append(d.label.label_text)
                distances.append(d.distance)
                point = Point2D()
                point.x = d.x
                point.y = d.y
                positions.append(point)
        msg = PafDetectedTrafficLights()
        msg.states = states
        msg.positions = positions
        msg.distances = distances
        self.traffic_light_publisher.publish(msg)
        self.inform_listener(time, detected)

    def inform_listener(self, time_stamp, detected_list):
        """
        Informs all listeners about the new list of detected objects
        Source: PAF 2020/21 group 1

        :param detected_list: the list of detected objects
        :param time_stamp: time stamp of the detection
        :return: None
        """
        if self.__listener is not None:
            self.__listener(time_stamp, detected_list)

    def set_on_detection_listener(self, func: Callable[[List[DetectedObject], rospy.Time], None]):
        """
        Set function to be called with detected objects
        Source: PAF 2020/21 group 1

        :param func: the function
        :return: None
        """
        self.__listener = func

    @staticmethod
    def start():
        """
        starts ROS node
        """
        rospy.spin()


def debug_screen():
    """
    Source: PAF 2020/21 group 1, adapted
    """
    from RGBCamera import RGBCamera
    from perception_util import show_image

    def store_image(image, _):
        global detected_r, time

        H, W = image.shape[:2]

        if detected_r is not None:
            # print("Detected Elements: " + str(len(detected_r)))
            for element in detected_r:
                # extract the bounding box coordinates
                (x, y) = (int(element.x * W), int(element.y * H))
                (w, h) = (int(element.w * W), int(element.h * H))
                # draw a bounding box rectangle and label on the image
                color_map = {
                    Labels.TrafficLightRed: (255, 0, 0),
                    Labels.TrafficLightGreen: (0, 255, 0),
                    Labels.TrafficLightYellow: (255, 255, 0),
                    Labels.TrafficLightUnknown: (0, 0, 0),
                }
                color = color_map.get(element.label, (0, 0, 0))
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}-{:.1f}m: {:.4f}".format(element.label.label_text, element.distance, element.confidence)
                cv2.putText(image, text, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

            # print(f"Age {(rospy.Time.now()-time).to_sec()}s")
        show_image("Traffic light detection", image)
        # print(f"took: {np.average(times)}ms")

    def on_detected(time_in, detected_list):
        global detected_r, time
        detected_r = detected_list
        time = time_in

    cam = RGBCamera()

    cam.set_on_image_listener(store_image)

    node.set_on_detection_listener(on_detected)


def dummy_detector():
    """
    Publishes dummy traffic light data
    """
    try:
        dummy_i = 0
        publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_perception/detected_traffic_lights", PafDetectedTrafficLights, queue_size=1
        )
        rate = rospy.Rate(1 / 5)  # 20s Interval
        while not rospy.is_shutdown():
            sig = ["green", "red"][dummy_i]
            rospy.logwarn(f"[traffic light detector] sending dummy signal: '{sig}'")
            msg = PafDetectedTrafficLights()
            msg.states = [sig]
            msg.positions = [Point2D(0.5, 0.5)]
            msg.distances = [20.0]
            publisher.publish(msg)
            dummy_i += 1
            dummy_i %= 2
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    """
    Source: PAF 2020/21 group 1, adapted
    """
    rospy.init_node("traffic_light_detector", anonymous=True)

    detected_r: Optional[list] = None
    time = None
    if rospy.get_param("~dummy"):
        dummy_detector()
        exit()

    try:
        node = TrafficLightDetector()
    except RuntimeError:
        rospy.logerr("[traffic light detector] Unable to use gpu (out of memory). Fallback to CPU...")
        node = TrafficLightDetector(use_gpu=False)
    if rospy.get_param("~debug"):
        debug_screen()
    node.start()
