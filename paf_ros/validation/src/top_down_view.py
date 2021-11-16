#!/usr/bin/env python

from argparse import Namespace
from enum import IntEnum

import cv2
from carla_birdeye_view import RGB, BirdViewProducer, BirdViewCropType, BirdView, actors, rotate, SegregatedActors
from carla_birdeye_view.mask import (
    MAP_BOUNDARY_MARGIN,
    PixelDimensions,
    MapMaskGenerator,
    CroppingRect,
    RenderingWindow,
    Coord,
    COLOR_ON,
)

import carla
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class MaskPriority(IntEnum):
    PEDESTRIANS = 10
    RED_LIGHTS = 9
    YELLOW_LIGHTS = 8
    GREEN_LIGHTS = 7
    AGENT = 6
    VEHICLES = 5
    LOCAL_PATH = 4
    GLOBAL_PATH = 3
    CENTERLINES = 2
    LANES = 1
    ROAD = 0

    @staticmethod
    def top_to_bottom():
        return list(MaskPriority)

    @staticmethod
    def bottom_to_top():
        return list(reversed(MaskPriority.top_to_bottom()))


RGB_BY_MASK = {  # (red, green, blue)
    MaskPriority.PEDESTRIANS: RGB.VIOLET,
    MaskPriority.RED_LIGHTS: RGB.RED,
    MaskPriority.YELLOW_LIGHTS: RGB.YELLOW,
    MaskPriority.GREEN_LIGHTS: RGB.GREEN,
    MaskPriority.AGENT: RGB.CHAMELEON,
    MaskPriority.VEHICLES: RGB.ORANGE,
    MaskPriority.CENTERLINES: RGB.CHOCOLATE,
    MaskPriority.LANES: RGB.WHITE,
    MaskPriority.ROAD: RGB.DIM_GRAY,
    MaskPriority.LOCAL_PATH: (255, 0, 0),
    MaskPriority.GLOBAL_PATH: (0, 0, 255),
}


class TopDownView(BirdViewProducer):
    def __init__(
        self,
        client: carla.Client,
        target_size: PixelDimensions = None,
        pixels_per_meter: int = 6,
        crop_type: BirdViewCropType = BirdViewCropType.FRONT_AND_REAR_AREA,
        north_is_up=True,
        show_whole_map=True,
        dark_mode=True,
    ):
        self.north_is_up = north_is_up
        self.dark_mode = dark_mode
        self.center_on_agent = not show_whole_map
        self.global_path, self.local_path = None, None
        self.path_width_px = 10
        if show_whole_map:
            self.north_is_up = True
            gen = MapMaskGenerator(client, pixels_per_meter)
            target_size_ = gen._mask_size
            target_size = PixelDimensions(
                width=int((target_size_.width + 2 * MAP_BOUNDARY_MARGIN) / 2),
                height=int((target_size_.height + 2 * MAP_BOUNDARY_MARGIN) / 2),
            )
        super(TopDownView, self).__init__(client, target_size, pixels_per_meter, crop_type)

    def produce(self, agent_vehicle: carla.Actor) -> BirdView:
        all_actors = actors.query_all(world=self._world)
        segregated_actors = actors.segregate_by_type(actors=all_actors)
        agent_vehicle_loc = agent_vehicle.get_location() if self.center_on_agent else Namespace(**{"x": 0, "y": 0})

        # same as super class below
        self.masks_generator.disable_local_rendering_mode()
        agent_global_px_pos = self.masks_generator.location_to_pixel(agent_vehicle_loc)
        cropping_rect = CroppingRect(
            x=int(agent_global_px_pos.x - self.rendering_area.width / 2),
            y=int(agent_global_px_pos.y - self.rendering_area.height / 2),
            width=self.rendering_area.width,
            height=self.rendering_area.height,
        )
        masks = np.zeros(
            shape=(
                len(MaskPriority),
                self.rendering_area.height,
                self.rendering_area.width,
            ),
            dtype=np.uint8,
        )
        masks[MaskPriority.ROAD.value] = self.full_road_cache[cropping_rect.vslice, cropping_rect.hslice]
        masks[MaskPriority.LANES.value] = self.full_lanes_cache[cropping_rect.vslice, cropping_rect.hslice]
        masks[MaskPriority.CENTERLINES.value] = self.full_centerlines_cache[cropping_rect.vslice, cropping_rect.hslice]
        rendering_window = RenderingWindow(origin=agent_vehicle_loc, area=self.rendering_area)
        self.masks_generator.enable_local_rendering_mode(rendering_window)
        masks = self._render_actors_masks(agent_vehicle, segregated_actors, masks)
        masks[MaskPriority.GLOBAL_PATH] = self._create_path_mask(self.global_path)
        masks[MaskPriority.LOCAL_PATH] = self._create_path_mask(self.local_path)
        cropped_masks = self.apply_agent_following_transformation_to_masks(
            agent_vehicle,
            masks,
        )
        ordered_indices = [mask.value for mask in MaskPriority.bottom_to_top()]
        return cropped_masks[ordered_indices]

    def apply_agent_following_transformation_to_masks(
        self,
        agent_vehicle: carla.Actor,
        masks: np.ndarray,
    ) -> np.ndarray:
        agent_transform = agent_vehicle.get_transform()
        angle = (0 if self.north_is_up else agent_transform.rotation.yaw) + 90

        # same as super class below
        crop_with_car_in_the_center = masks
        masks_n, h, w = crop_with_car_in_the_center.shape
        rotation_center = Coord(x=w // 2, y=h // 2)
        crop_with_centered_car = np.transpose(crop_with_car_in_the_center, axes=(1, 2, 0))
        rotated = rotate(crop_with_centered_car, angle, center=rotation_center)
        rotated = np.transpose(rotated, axes=(2, 0, 1))
        half_width = self.target_size.width // 2
        hslice = slice(rotation_center.x - half_width, rotation_center.x + half_width)
        if self._crop_type is BirdViewCropType.FRONT_AREA_ONLY:
            vslice = slice(rotation_center.y - self.target_size.height, rotation_center.y)
        elif self._crop_type is BirdViewCropType.FRONT_AND_REAR_AREA:
            half_height = self.target_size.height // 2
            vslice = slice(rotation_center.y - half_height, rotation_center.y + half_height)
        else:
            raise NotImplementedError
        assert (
            vslice.start > 0 and hslice.start > 0
        ), "Trying to access negative indexes is not allowed, check for calculation errors!"
        return rotated[:, vslice, hslice]

    def set_path(self, coordinate_list_global_path=None, coordinate_list_local_path=None, width_px=None):
        if coordinate_list_global_path is not None:
            self.global_path = coordinate_list_global_path
        if coordinate_list_local_path is not None:
            self.local_path = coordinate_list_local_path
        if width_px is not None:
            self.path_width_px = width_px

    def clear_path(self, clear_global_path=True, clear_local_path=True):
        if clear_local_path:
            self.local_path = None
        if clear_global_path:
            self.global_path = None

    def _create_path_mask(self, path):
        mask = self.masks_generator.make_empty_mask()
        if path is None:
            return mask
        points = [self.masks_generator.location_to_pixel(Namespace(**{"x": x, "y": y})) for x, y in path]
        points = np.array([[p.x, p.y] for p in points])
        mask = cv2.polylines(mask, [points.reshape((-1, 1, 2))], False, COLOR_ON, self.path_width_px)
        return mask

    def _render_actors_masks(
        self,
        agent_vehicle: carla.Actor,
        segregated_actors: SegregatedActors,
        masks: np.ndarray,
    ) -> np.ndarray:
        # same as super class with new MaskPriorities
        lights_masks = self.masks_generator.traffic_lights_masks(segregated_actors.traffic_lights)
        red_lights_mask, yellow_lights_mask, green_lights_mask = lights_masks
        masks[MaskPriority.RED_LIGHTS.value] = red_lights_mask
        masks[MaskPriority.YELLOW_LIGHTS.value] = yellow_lights_mask
        masks[MaskPriority.GREEN_LIGHTS.value] = green_lights_mask
        masks[MaskPriority.AGENT.value] = self.masks_generator.agent_vehicle_mask(agent_vehicle)
        masks[MaskPriority.VEHICLES.value] = self.masks_generator.vehicles_mask(segregated_actors.vehicles)
        masks[MaskPriority.PEDESTRIANS.value] = self.masks_generator.pedestrians_mask(segregated_actors.pedestrians)
        return masks

    def as_rgb(self, birdview: BirdView):
        _, h, w = birdview.shape
        rgb_canvas = np.zeros(shape=(h, w, 3), dtype=np.uint8)

        def nonzero_indices(arr):
            return arr == COLOR_ON

        if self.dark_mode:
            color_inversion = []
        else:
            color_inversion = [MaskPriority.PEDESTRIANS]
        for mask_type in MaskPriority.bottom_to_top():
            rgb_color = RGB_BY_MASK[mask_type]
            if mask_type in color_inversion:
                r, g, b = rgb_color
                rgb_color = 255 - r, 255 - g, 255 - b
            rgb_canvas[nonzero_indices(birdview[mask_type])] = rgb_color
        if not self.dark_mode:
            rgb_canvas = np.where(rgb_canvas.any(-1, keepdims=True), rgb_canvas, 255)
        return rgb_canvas


class TopDownRosNode(object):
    br = CvBridge()

    def __init__(self, _actor):
        self.params = rospy.get_param("/top_down_view/")
        self.actor = _actor
        self._init()
        rate = 1000 / self.params["update_ms"]
        rospy.init_node(self.params["node"], anonymous=True)
        print(self.actor.type_id)
        self.pub = rospy.Publisher(self.params["topic"], Image, queue_size=1)
        self.loop_rate = rospy.Rate(rate)

    def _init(self):
        client = carla.Client("127.0.0.1", 2000)
        self.producer = TopDownView(
            client,
            target_size=PixelDimensions(
                width=self.params["img_size"]["width"], height=self.params["img_size"]["height"]
            ),
            pixels_per_meter=self.params["pixels_per_meter"],
            show_whole_map=self.params["show_whole_map"],
            north_is_up=self.params["north_is_up"],
            dark_mode=self.params["dark_mode"],
        )
        self.producer.set_path([[-500, -500], [500, 500]], [[500, -500], [-500, 500]])
        print(f"top_down_view tracking {self.actor.type_id}")

    def produce_map(self):
        birdview = self.producer.produce(agent_vehicle=self.actor)
        return self.producer.as_rgb(birdview)

    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo("publishing image")
            rgb = self.produce_map()
            self.pub.publish(self.br.cv2_to_imgmsg(rgb, "rgb8"))
            self.loop_rate.sleep()


if __name__ == "__main__":
    client = carla.Client("127.0.0.1", 2000)
    _actors = client.get_world().get_actors()
    vehicles = []
    for actor in _actors:
        if "vehicle." in actor.type_id:
            vehicles.append(actor)
        if "role_name" in actor.attributes and actor.attributes["role_name"] == rospy.get_param(
            "/top_down_view/role_name"
        ):
            break
    else:
        if not len(vehicles):
            raise RuntimeError("No random vehicle to track!")
        actor = np.random.choice(vehicles)
    TopDownRosNode(actor).start()
