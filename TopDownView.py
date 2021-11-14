from argparse import Namespace
from enum import IntEnum

import cv2
from carla_birdeye_view import RGB, BirdViewProducer, BirdViewCropType, BirdView, actors, rotate
from carla_birdeye_view.mask import MAP_BOUNDARY_MARGIN, PixelDimensions, MapMaskGenerator, CroppingRect, \
    RenderingWindow, Coord, COLOR_ON

import carla
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class MaskPriority(IntEnum):
    PEDESTRIANS = 9
    RED_LIGHTS = 8
    YELLOW_LIGHTS = 7
    GREEN_LIGHTS = 6
    AGENT = 5
    VEHICLES = 4
    PATH = 3
    CENTERLINES = 2
    LANES = 1
    ROAD = 0

    @staticmethod
    def top_to_bottom():
        return list(MaskPriority)

    @staticmethod
    def bottom_to_top():
        return list(reversed(MaskPriority.top_to_bottom()))


RGB_BY_MASK = {
    MaskPriority.PEDESTRIANS: RGB.VIOLET,
    MaskPriority.RED_LIGHTS: RGB.RED,
    MaskPriority.YELLOW_LIGHTS: RGB.YELLOW,
    MaskPriority.GREEN_LIGHTS: RGB.GREEN,
    MaskPriority.AGENT: RGB.CHAMELEON,
    MaskPriority.VEHICLES: RGB.ORANGE,
    MaskPriority.CENTERLINES: RGB.CHOCOLATE,
    MaskPriority.LANES: RGB.WHITE,
    MaskPriority.ROAD: RGB.DIM_GRAY,
    MaskPriority.PATH: (0, 0, 255)
}


class TopDownView(BirdViewProducer):
    def __init__(self, client: carla.Client, target_size: PixelDimensions = None, pixels_per_meter: int = 6,
                 crop_type: BirdViewCropType = BirdViewCropType.FRONT_AND_REAR_AREA,
                 north_is_up=True, center_on_agent=True
                 ):
        self.north_is_up = north_is_up
        self.center_on_agent = center_on_agent
        self.path = None
        self.path_width_px = 10
        if not center_on_agent:
            self.north_is_up = True
            gen = MapMaskGenerator(client, pixels_per_meter)
            target_size_ = gen._mask_size
            target_size = PixelDimensions(width=int((target_size_.width + 2 * MAP_BOUNDARY_MARGIN) / 2),
                                          height=int((target_size_.height + 2 * MAP_BOUNDARY_MARGIN) / 2))
            print(target_size_)
        super(TopDownView, self).__init__(client, target_size, pixels_per_meter, crop_type)
        # agent_vehicle_loc = agent_vehicle.get_location() if self.center_on_agent
        # else Namespace(**{'x': 0, 'y': 0}) -> produce
        # angle = (0 if self.north_is_up else agent_transform.rotation.yaw) + 90
        # -> apply_agent_following_transformation_to_masks

    def produce(self, agent_vehicle: carla.Actor) -> BirdView:
        all_actors = actors.query_all(world=self._world)
        segregated_actors = actors.segregate_by_type(actors=all_actors)
        agent_vehicle_loc = agent_vehicle.get_location() if self.center_on_agent else Namespace(**{'x': 0, 'y': 0})

        # Reusing already generated static masks for whole map
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
        masks[MaskPriority.ROAD.value] = self.full_road_cache[
            cropping_rect.vslice, cropping_rect.hslice
        ]
        masks[MaskPriority.LANES.value] = self.full_lanes_cache[
            cropping_rect.vslice, cropping_rect.hslice
        ]
        masks[MaskPriority.CENTERLINES.value] = self.full_centerlines_cache[
            cropping_rect.vslice, cropping_rect.hslice
        ]

        # Dynamic masks
        rendering_window = RenderingWindow(
            origin=agent_vehicle_loc, area=self.rendering_area
        )
        self.masks_generator.enable_local_rendering_mode(rendering_window)
        masks = self._render_actors_masks(agent_vehicle, segregated_actors, masks)
        masks[MaskPriority.PATH] = self.create_path_mask()
        cropped_masks = self.apply_agent_following_transformation_to_masks(
            agent_vehicle, masks,
        )
        ordered_indices = [mask.value for mask in MaskPriority.bottom_to_top()]
        return cropped_masks[ordered_indices]

    def apply_agent_following_transformation_to_masks(
            self, agent_vehicle: carla.Actor, masks: np.ndarray,
    ) -> np.ndarray:
        agent_transform = agent_vehicle.get_transform()
        angle = (0 if self.north_is_up else agent_transform.rotation.yaw) + 90

        # Rotating around the center
        crop_with_car_in_the_center = masks
        masks_n, h, w = crop_with_car_in_the_center.shape
        rotation_center = Coord(x=w // 2, y=h // 2)

        # warpAffine from OpenCV requires the first two dimensions to be in order: height, width, channels
        crop_with_centered_car = np.transpose(
            crop_with_car_in_the_center, axes=(1, 2, 0)
        )
        rotated = rotate(crop_with_centered_car, angle, center=rotation_center)
        rotated = np.transpose(rotated, axes=(2, 0, 1))

        half_width = self.target_size.width // 2
        hslice = slice(rotation_center.x - half_width, rotation_center.x + half_width)

        if self._crop_type is BirdViewCropType.FRONT_AREA_ONLY:
            vslice = slice(rotation_center.y - self.target_size.height, rotation_center.y)
        elif self._crop_type is BirdViewCropType.FRONT_AND_REAR_AREA:
            half_height = self.target_size.height // 2
            vslice = slice(
                rotation_center.y - half_height, rotation_center.y + half_height
            )
        else:
            raise NotImplementedError
        assert (
                vslice.start > 0 and hslice.start > 0
        ), "Trying to access negative indexes is not allowed, check for calculation errors!"
        car_on_the_bottom = rotated[:, vslice, hslice]
        return car_on_the_bottom

    def set_path(self, coordinate_list, width_px=None):
        self.path = coordinate_list
        if width_px is not None:
            self.path_width_px = width_px

    def create_path_mask(self):
        mask = self.masks_generator.make_empty_mask()
        points = [self.masks_generator.location_to_pixel(Namespace(**{'x': x, 'y': y})) for x, y in self.path]
        points = np.array([[p.x, p.y] for p in points])
        points = points.reshape((-1, 1, 2))
        mask = cv2.polylines(mask, [points], False, COLOR_ON, self.path_width_px)
        return mask

    @staticmethod
    def as_rgb(birdview: BirdView, dark=True):
        _, h, w = birdview.shape
        rgb_canvas = np.zeros(shape=(h, w, 3), dtype=np.uint8)
        nonzero_indices = lambda arr: arr == COLOR_ON
        ignore_inversion = [MaskPriority.ROAD, MaskPriority.GREEN_LIGHTS, MaskPriority.YELLOW_LIGHTS,
                            MaskPriority.RED_LIGHTS, MaskPriority.AGENT, MaskPriority.VEHICLES, MaskPriority.PATH]

        for mask_type in MaskPriority.bottom_to_top():
            rgb_color = RGB_BY_MASK[mask_type]
            if not dark and mask_type not in ignore_inversion:
                r, g, b = rgb_color
                rgb_color = 255 - r, 255 - g, 255 - b
            mask = birdview[mask_type]
            # If mask above contains 0, don't overwrite content of canvas (0 indicates transparency)
            rgb_canvas[nonzero_indices(mask)] = rgb_color
        if not dark:
            rgb_canvas = np.where(rgb_canvas.any(-1, keepdims=True), rgb_canvas, 255)
        return rgb_canvas


class TopDownRosNode(object):
    br = CvBridge()

    def __init__(self):
        client = carla.Client('127.0.0.1', 2000)
        self.producer = TopDownView(
            client,
            target_size=PixelDimensions(width=2000, height=1000),
            pixels_per_meter=6,
            center_on_agent=True,
            north_is_up=True,
        )
        self.producer.set_path([[-500, -500], [500, 500]])
        _actors = client.get_world().get_actors()
        vehicles = []
        self.actor = None
        for actor in _actors:
            if "vehicle." in actor.type_id:
                # if actor.type_id == "vehicle.tesla.model3":
                vehicles.append(actor)
        self.actor = np.random.choice(vehicles)
        if self.actor is None:
            raise RuntimeError("No vehicle to track!")
        print(self.actor.type_id)
        self.pub = rospy.Publisher('aaa', Image, queue_size=1)
        self.loop_rate = rospy.Rate(1)

    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            print(self.actor.get_location())
            birdview = self.producer.produce(agent_vehicle=self.actor)
            rgb = self.producer.as_rgb(birdview)
            self.pub.publish(self.br.cv2_to_imgmsg(rgb, 'rgb8'))
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("aaa", anonymous=True)
    TopDownRosNode().start()
