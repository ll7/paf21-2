#!/usr/bin/env python
from enum import Enum
import json
import math
import os
from datetime import datetime
from typing import Callable, List, Tuple
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
    """

    def __init__(self, nr: int, label: str):
        self._value_ = nr
        self._label = label

    @property
    def label_text(self) -> str:
        """
        Returns the label name
        :return:
        """
        return self._label

    def __str__(self):
        return self.label_text

    TrafficLightUnknown = (20, "unknown")
    TrafficLightRed = (21, "red")
    TrafficLightYellow = (22, "yellow")
    TrafficLightGreen = (23, "green")

    Other = (99, "Other")


class DetectedObject:
    """
    Data class for detected elements
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
        return f"x={self.x},y={self.y},label={self.label}"


class TrafficLightDetector:
    """
    Detector for traffic lights
    """

    def __init__(self, use_gpu: bool = True):
        """
        Init the speed traffic light detector
        :param role_name: the name of the vehicle to access the cameras
        :param use_gpu: whether the classification model should be loaded to the gpu
        """

        rospy.init_node("traffic_light_detector", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        self.traffic_light_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_perception/detected_traffic_lights", PafDetectedTrafficLights, queue_size=1
        )

        self.detection_activated = True

        self.__listener = None

        self.logger_name = "traffic_light_detector"

        self.confidence_min = 0.75
        self.threshold = 0.7
        self.canny_threshold = 100

        self.data_collect_path = None  # "/home/psaf1/Documents/traffic_light_data"

        rospack = rospkg.RosPack()
        root_path = rospack.get_path("paf_perception")

        # Deep learning config and inits
        self.confidence_min = 0.70
        self.threshold = 1.0
        rospy.loginfo(f"[traffic light detector] init device (use gpu={use_gpu})", logger_name=self.logger_name)
        # select the gpu if allowed and a gpu is available
        self.device = torch.device("cuda:0" if use_gpu and torch.cuda.is_available() else "cpu")
        rospy.loginfo("[traffic light detector] Device:" + str(self.device), logger_name=self.logger_name)
        # load our model
        model_name = "traffic-light-classifiers-2021-03-27_23-55-48"
        rospy.loginfo("[traffic light detector] loading classifier model from disk...", logger_name=self.logger_name)
        model = torch.load(os.path.join(root_path, f"models/{model_name}.pt"))

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
            rospy.loginfo("[traffic light detector] Enable cudnn", logger_name=self.logger_name)
            cudnn.enabled = True
            cudnn.benchmark = True
        self.net = model
        self.net.eval()
        torch.no_grad()  # reduce memory consumption and improve speed

        activation_topic = "/paf/paf_local_planner/activate_traffic_light_detection"
        rospy.Subscriber(activation_topic, Bool, self._activation_toggled, queue_size=1)

        # init image source = combination of segmentation, rgb and depth camera
        self.combinedCamera = FusionCamera(role_name=role_name, visible_tags={SegmentationTag.TrafficLight})
        self.combinedCamera.set_on_image_listener(self.__on_new_image_data)

    def _activation_toggled(self, activate: bool):
        self.detection_activated = activate.data
        if activate.data:
            rospy.loginfo("[traffic light detector] Detection activated", logger_name=self.logger_name)
        else:
            rospy.loginfo("[traffic light detector] Detection deactivated", logger_name=self.logger_name)

    def __extract_label(self, image) -> Tuple[Labels, float]:
        """
        Analyze the given image of the traffic light and returns the corresponding label
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
        Handles the new image data from the cameras
        :param segmentation_image: the segmentation image
        :param rgb_image: the rgb image
        :param depth_image: the depth data
        :param time: time stamp when the images where taken
        :return: none
        """
        if self.detection_activated is False:
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
                    # Classify the cropped image
                    label, confidence = self.__extract_label(crop_rgb)
                    if label is not None:
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
        :param detected_list: the list of detected objects
        :param time_stamp: time stamp of the detection
        :return: None
        """
        if self.__listener is not None:
            self.__listener(time_stamp, detected_list)

    def set_on_detection_listener(self, func: Callable[[List[DetectedObject], rospy.Time], None]):
        """
        Set function to be called with detected objects
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


if __name__ == "__main__":

    node = TrafficLightDetector()
    debug = True
    # Show case code:
    if debug:
        from RGBCamera import RGBCamera
        from perception_util import show_image

        detected_r = None
        time = None

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

    node.start()
