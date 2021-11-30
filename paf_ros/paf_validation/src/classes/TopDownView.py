import cv2
import carla
import numpy as np
import rospy

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

from argparse import Namespace
from enum import IntEnum
from paf_perception.msg import PafObstacleList, PafObstacle


class MaskPriority(IntEnum):
    VEH_OBSTACLES = 12
    PED_OBSTACLES = 11
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
    MaskPriority.PED_OBSTACLES: (255, 0, 0),
    MaskPriority.VEH_OBSTACLES: (0, 0, 255),
}


class TopDownView(BirdViewProducer):
    def __init__(
        self,
        client: carla.Client,
        target_size: PixelDimensions = None,
        pixels_per_meter: int = 6,
        north_is_up=True,
        show_whole_map=True,
        dark_mode=True,
    ):
        """
        Image Producer
        :param client: carla client instance
        :param target_size: size of output image
        :param pixels_per_meter: resolution of output image
        :param north_is_up: true: north is always up, false: direction of travel is always up
        :param show_whole_map: shows whole map (overrides target_size parameter)
        :param dark_mode: resulting map theme dark/light
        """
        self.north_is_up = north_is_up
        self.dark_mode = dark_mode
        self.center_on_agent = not show_whole_map
        self.global_path, self.local_path = None, None
        self.obstacles_pedestrians, self.obstacles_vehicles = None, None
        self.path_width_px = None
        self.set_path(width_px=10)
        if show_whole_map:
            self.north_is_up = True
            gen = MapMaskGenerator(client, pixels_per_meter)
            target_size_ = gen._mask_size
            target_size = PixelDimensions(
                width=int((target_size_.width + 2 * MAP_BOUNDARY_MARGIN) / 2),
                height=int((target_size_.height + 2 * MAP_BOUNDARY_MARGIN) / 2),
            )
        super(TopDownView, self).__init__(client, target_size, pixels_per_meter, BirdViewCropType.FRONT_AND_REAR_AREA)

    def produce(self, agent_vehicle: carla.Actor) -> BirdView:
        """
        Reads Carla information and translates them to a series of mask layers
        :param agent_vehicle: actor the map centers on (if not show_whole_map)
        :return: mask layers
        """
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
        if self.global_path is not None:
            masks[MaskPriority.GLOBAL_PATH] = self._create_path_mask(self.global_path)
        if self.local_path is not None:
            masks[MaskPriority.LOCAL_PATH] = self._create_path_mask(self.local_path)

        mask_obstacles = None
        if self.obstacles_pedestrians is not None:
            masks[MaskPriority.PED_OBSTACLES] = self._create_obstacle_mask(self.obstacles_pedestrians, mask_obstacles)
        if self.obstacles_vehicles is not None:
            masks[MaskPriority.VEH_OBSTACLES] = self._create_obstacle_mask(self.obstacles_vehicles, mask_obstacles)
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
        """
        Cropping and rotating of output masks
        :param agent_vehicle: actor the map centers on (if not show_whole_map)
        :param masks: mask layers
        :return:
        """
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

    def set_path(
        self, coordinate_list_global_path: list = None, coordinate_list_local_path: list = None, width_px: float = None
    ):
        """
        Setter for global path, local path and path width
        :param coordinate_list_global_path: [[ x,y ], [ x,y ], ...]
        :param coordinate_list_local_path: [[ x,y ], [ x,y ], ...]
        :param width_px: pixel width at resolution 10px/m (scaled to other resolutions)
        """
        if coordinate_list_global_path is not None:
            self.global_path = coordinate_list_global_path
        if coordinate_list_local_path is not None:
            self.local_path = coordinate_list_local_path
        if width_px is not None:
            width_px = width_px / 10 * self._pixels_per_meter
            self.path_width_px = int(np.ceil(width_px))

    def clear_path(self, clear_global_path=True, clear_local_path=True):
        """
        Remove path(s) from map
        :param clear_global_path: remove global flag
        :param clear_local_path: remove local flag
        """
        if clear_local_path:
            self.local_path = None
        if clear_global_path:
            self.global_path = None

    def update_obstacles(self, msg: PafObstacleList):
        """
        Update obstacle mask
        :param msg: msg from Obstacle topic
        """
        if msg.type == "Pedestrians":
            self.obstacles_pedestrians = self._update_obstacles(msg)
        elif msg.type == "Vehicles":
            self.obstacles_vehicles = self._update_obstacles(msg)
        else:
            rospy.logwarn_once(f"obstacle type '{msg.type}' is unknown to top_down_view node")

    @staticmethod
    def _update_obstacles(msg: PafObstacleList) -> list:
        """
        Updates a obstacle mask (pedestrians or vehicles) with new data (old data is discarded)
        :param msg: Obstacle List
        :return: [ [[ b1_x,b1_y ], [ b2_x,b2_y ], [ cl_x,cl_y ]], ...] list of all obstacles and their points
        """
        obs: PafObstacle
        ret = []
        for obs in msg.obstacles:
            obs_pts = []
            for x, y in [obs.bound_1, obs.bound_2, obs.closest]:
                obs_pts.append(Namespace(**{"x": x, "y": y}))
            ret.append(obs_pts)
        return ret

    def _create_obstacle_mask(self, objects: list, mask: np.array = None) -> np.array:
        """
        Draw obstacles on mask
        :param objects: list of obstacles (3 pts per obstacle)
        :param mask: layer to draw on. if None, a new one will be created
        :return: updated mask
        """
        if mask is None:
            mask = self.masks_generator.make_empty_mask()
        for obs in objects:
            corner_pixels = []
            for point in obs:
                pixel = self.masks_generator.location_to_pixel(point)
                corner_pixels.append([pixel.x, pixel.y])
            mask = cv2.circle(mask, tuple(corner_pixels[-1]), 4, COLOR_ON, -1)
            mask = cv2.polylines(mask, [np.array(corner_pixels).reshape((-1, 1, 2))], True, COLOR_ON, 1)
        return mask

    def _create_path_mask(self, path: list = None) -> np.array:
        """
        Draws a path on a new mask
        :param path: list of path points (x,y)
        :return: updated mask
        """
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
        """
        Create Actor masks (uses new MaskPriority class, else same as super class)
        :param agent_vehicle:
        :param segregated_actors:
        :param masks:
        :return:
        """
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

    def as_rgb(self, birdview: BirdView) -> np.array:
        """
        Translates list of masks to a rgb image
        :param birdview: masks list
        :return: rgb image
        """
        _, h, w = birdview.shape
        rgb_canvas = np.zeros(shape=(h, w, 3), dtype=np.uint8)

        if self.dark_mode:
            color_inversion = []
        else:
            color_inversion = [MaskPriority.PEDESTRIANS]
        for mask_type in MaskPriority.bottom_to_top():
            rgb_color = RGB_BY_MASK[mask_type]
            if mask_type in color_inversion:
                r, g, b = rgb_color
                rgb_color = 255 - r, 255 - g, 255 - b
            rgb_canvas[self.nonzero_indices(birdview[mask_type])] = rgb_color
        if not self.dark_mode:
            rgb_canvas = np.where(rgb_canvas.any(-1, keepdims=True), rgb_canvas, 255)
        return rgb_canvas

    @staticmethod
    def nonzero_indices(arr):
        """
        mask2rgb conversion filter
        :param arr: mask
        :return: true/false array of pixels to color in
        """
        return arr == COLOR_ON
