#!/usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# ==============================================================================
# ---PAFTutorial----------------------------------------------------------------
# ==============================================================================

import rospy

from carla_msgs.msg import CarlaEgoVehicleStatus
from std_msgs.msg import Float32


class PAFTutorial(object):
    def __init__(self):
        self.loop_rate = rospy.Rate(10)

        # ==========================================
        # -- Subscriber ----------------------------
        # ==========================================

        # register your subscribed topics here
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_status_subscriber = rospy.Subscriber(
            "carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, self.vehicle_status_updated
        )

        # ==========================================
        # -- Publisher ----------------------------
        # ==========================================

        # register your published topics here
        self.velocity_publisher = rospy.Publisher("paf_tutorial/velocity", Float32, queue_size=50)

    def vehicle_status_updated(self, vehicle_status):
        self.vehicle_status = vehicle_status
        self.velocity_publisher.publish(self.vehicle_status.velocity)
        rospy.loginfo("New velocity published!")

    def __del__(self):
        """
        Unregister all publisher and subscriber
        """
        rospy.loginfo("Unregister topics...")
        self.vehicle_status_subscriber.unregister()
        self.velocity_publisher.unregister()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    rospy.init_node("paf_starter", anonymous=True)

    paf_tutorial = None

    try:
        paf_tutorial = PAFTutorial()
        while not rospy.is_shutdown():
            # call an optional loop function here
            pass
    finally:
        if paf_tutorial is not None:
            del paf_tutorial
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
